// Pixhawk MAVLink <-> ROS 2 Humble bridge. Units: SI; MAVLink NED/FRD -> ROS ENU/FLU.
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <pixhawk_interfaces/srv/command_long.hpp>
#include <pixhawk_interfaces/srv/param_get.hpp>
#include <pixhawk_interfaces/srv/param_set.hpp>
#include <pixhawk_interfaces/srv/goto_local.hpp>
#include <pixhawk_interfaces/srv/goto_global.hpp>
#include <pixhawk_interfaces/srv/set_mode.hpp>
#include <pixhawk_interfaces/srv/velocity_vector.hpp>
#include <pixhawk_interfaces/srv/message_interval.hpp>
#include <pixhawk_interfaces/srv/mission_download.hpp>
#include <pixhawk_interfaces/srv/mission_upload.hpp>
#include <pixhawk_interfaces/msg/mission_item.hpp>
#include <pixhawk_interfaces/msg/avoidance_advisory.hpp>
#include "../../../avoidance_core/AvoidanceWire.hpp"
#include <development/mavlink.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/file.h>
#include <poll.h>
#include <cerrno>
#include <cstring>
#include <cmath>
#include <chrono>
#include <limits>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <unordered_map>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <memory>
#include <array>
#include <vector>
using namespace std::chrono_literals;
namespace iface = pixhawk_interfaces::srv;

struct Quaternion { double w=1.,x=0.,y=0.,z=0.; };
static Quaternion multiply(const Quaternion &a,const Quaternion &b) {
  return {a.w*b.w-a.x*b.x-a.y*b.y-a.z*b.z,
    a.w*b.x+a.x*b.w+a.y*b.z-a.z*b.y,
    a.w*b.y-a.x*b.z+a.y*b.w+a.z*b.x,
    a.w*b.z+a.x*b.y-a.y*b.x+a.z*b.w};
}
static Quaternion to_ros_orientation(const Quaternion &ned_frd) {
  constexpr double s=0.7071067811865475244;
  return multiply(multiply({0.,s,s,0.},ned_frd),{0.,1.,0.,0.});
}
static geometry_msgs::msg::Quaternion to_msg(const Quaternion &q) {
  geometry_msgs::msg::Quaternion o; o.w=q.w; o.x=q.x; o.y=q.y; o.z=q.z;return o;
}
static bool is_finite_value(double v) {return std::isfinite(v);}
static std::string json_num(double v){
  if(!std::isfinite(v))return "null";
  std::ostringstream ss;ss<<std::setprecision(10)<<v;return ss.str();
}
static std::array<char,16> param_id_buf(const std::string &s){
  std::array<char,16> v{};std::memcpy(v.data(),s.data(),std::min(s.size(),v.size()));return v;
}
static std::string safe_param_name(const std::string &s) {
  if(s.empty()||s.size()>16)return {};
  const bool invalid=std::any_of(s.begin(),s.end(),[](unsigned char c){
    return !((c>='A'&&c<='Z')||(c>='0'&&c<='9')||c=='_');
  });
  if(invalid)return {};
  return s;
}
static double decode_param(float f,uint8_t type) {
  uint32_t raw=0;std::memcpy(&raw,&f,sizeof(raw));
  switch(type) {
    case MAV_PARAM_TYPE_UINT8: return static_cast<uint8_t>(raw);
    case MAV_PARAM_TYPE_INT8: return static_cast<int8_t>(raw);
    case MAV_PARAM_TYPE_UINT16:return static_cast<uint16_t>(raw);
    case MAV_PARAM_TYPE_INT16:return static_cast<int16_t>(raw);
    case MAV_PARAM_TYPE_UINT32:return static_cast<uint32_t>(raw);
    case MAV_PARAM_TYPE_INT32:return static_cast<int32_t>(raw);
    default:return f;
  }
}
static float encode_param(double value,uint8_t type) {
  if(type==MAV_PARAM_TYPE_REAL32)return static_cast<float>(value);
  uint32_t bits=0;
  if(type==MAV_PARAM_TYPE_UINT8||type==MAV_PARAM_TYPE_UINT16||type==MAV_PARAM_TYPE_UINT32)
    bits=static_cast<uint32_t>(value);
  else bits=static_cast<uint32_t>(static_cast<int32_t>(value));
  float out;std::memcpy(&out,&bits,sizeof(out));return out;
}
struct ParamValue {double value=0;uint8_t type=0;uint64_t seq=0;};
struct Setpoint {bool active=false;bool global=false;bool velocity=false;float x=0,y=0,z=0,yaw=0;
  int32_t lat=0,lon=0;std::chrono::steady_clock::time_point expires{},activated{};};
class PixhawkBridge final : public rclcpp::Node {
public:
  PixhawkBridge():Node("pixhawk_bridge") {
    device_=declare_parameter<std::string>("device","/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v2.x_0-if00");
    baud_=declare_parameter<int>("baud",115200);
    enable_motion_=declare_parameter<bool>("enable_motion_commands",false);
    allow_param_write_=declare_parameter<bool>("allow_parameter_writes",false);
    allow_mode_=declare_parameter<bool>("allow_mode_changes",false);
    allow_mission_=declare_parameter<bool>("allow_mission_writes",false);
    allow_velocity_=declare_parameter<bool>("allow_stock_rover_velocity",false);
    rover_confirmed_=declare_parameter<bool>("stock_rover_interface_confirmed",false);
    raw_enabled_=declare_parameter<bool>("publish_raw_mavlink",false);
    avoidance_tx_=declare_parameter<bool>("enable_avoidance_advisory_tx",false);
    avoidance_fw_confirmed_=declare_parameter<bool>("avoidance_firmware_confirmed",false);
    if(avoidance_tx_ && !avoidance_fw_confirmed_)
      throw std::runtime_error("Avoidance TX requires explicit firmware confirmation; independent of motion-service permissions");
    const auto now_session=static_cast<uint32_t>(std::chrono::steady_clock::now().time_since_epoch().count());
    avoidance_session_=now_session?now_session:1;
    auto_sensor_streams_=declare_parameter<bool>("request_auxiliary_sensor_streams",true);
    max_accuracy_=declare_parameter<double>("max_position_accuracy_m",5.0);
    max_speed_=declare_parameter<double>("max_plausible_speed_mps",30.0);
    hb_timeout_=declare_parameter<double>("heartbeat_timeout_sec",3.0);
    imu_pub_=create_publisher<sensor_msgs::msg::Imu>("imu",rclcpp::SensorDataQoS());
    for(size_t k=0;k<3;k++){
      imu_banks_[k]=create_publisher<sensor_msgs::msg::Imu>("imu/instance_"+std::to_string(k),rclcpp::SensorDataQoS());
    }
    estimator_pub_=create_publisher<std_msgs::msg::String>("estimator/status",10);
    vibration_pub_=create_publisher<std_msgs::msg::String>("sensors/vibration",10);
    sensor_health_pub_=create_publisher<std_msgs::msg::String>("sensors/health",10);
    imu_availability_pub_=create_publisher<std_msgs::msg::String>("imu/availability",10);
    mission_pub_=create_publisher<std_msgs::msg::String>("mission/events",10);
    version_pub_=create_publisher<std_msgs::msg::String>("firmware/version",rclcpp::QoS(1).transient_local());
    rover_feedback_pub_=create_publisher<std_msgs::msg::String>("rover/feedback",10);
    raw_odometry_pub_=create_publisher<std_msgs::msg::String>("estimator/odometry_raw",10);
    mag_pub_=create_publisher<sensor_msgs::msg::MagneticField>("mag",rclcpp::SensorDataQoS());
    baro_pub_=create_publisher<sensor_msgs::msg::FluidPressure>("barometer",rclcpp::SensorDataQoS());
    attitude_pub_=create_publisher<geometry_msgs::msg::QuaternionStamped>("attitude",rclcpp::SensorDataQoS());
    gps_pub_=create_publisher<sensor_msgs::msg::NavSatFix>("gps/fix",rclcpp::SensorDataQoS());
    battery_pub_=create_publisher<sensor_msgs::msg::BatteryState>("battery",rclcpp::SensorDataQoS());
    odom_pub_=create_publisher<nav_msgs::msg::Odometry>("odometry",rclcpp::SensorDataQoS());
    status_pub_=create_publisher<std_msgs::msg::String>("status",10);
    param_pub_=create_publisher<std_msgs::msg::String>("parameters",10);
    event_pub_=create_publisher<std_msgs::msg::String>("events",20);
    if(raw_enabled_)raw_pub_=create_publisher<std_msgs::msg::UInt8MultiArray>("raw_mavlink",rclcpp::SensorDataQoS());
    avoidance_sub_=create_subscription<pixhawk_interfaces::msg::AvoidanceAdvisory>(
      "avoidance/advisory",rclcpp::QoS(5),
      [this](const pixhawk_interfaces::msg::AvoidanceAdvisory::ConstSharedPtr m) {
        const bool finite=std::isfinite(m->vx_limit_mps) &&
          std::isfinite(m->yaw_correction_radps)&&
          std::isfinite(m->front_clearance_m)&&
          std::isfinite(m->left_clearance_m)&&
          std::isfinite(m->right_clearance_m);
        if(!finite||m->vx_limit_mps<0||m->vx_limit_mps>5||
           std::abs(m->yaw_correction_radps)>3||
           m->front_clearance_m<0||m->left_clearance_m<0||m->right_clearance_m<0)return;
        std::lock_guard<std::mutex> lk(avoidance_mutex_);
        avoidance_last_=*m;
        avoidance_last_rx_=std::chrono::steady_clock::now();
        avoidance_received_=true;
      });
    setup_services();
    running_=true;
    io_thread_=std::thread(&PixhawkBridge::io_loop,this);
    periodic_=create_wall_timer(100ms,[this]{tick();});
    RCLCPP_INFO(get_logger(),"MAVLink bridge using %s baud=%d; motion=%s, parameter writes=%s, mode changes=%s",
      device_.c_str(),baud_,enable_motion_?"ENABLED":"DISABLED",
      allow_param_write_?"ENABLED":"DISABLED",allow_mode_?"ENABLED":"DISABLED");
  }
  ~PixhawkBridge() override {
    running_=false; cv_.notify_all();
    if(io_thread_.joinable())io_thread_.join();
    close_port();
  }
private:
  std::string device_;int baud_=115200;bool enable_motion_=false,allow_param_write_=false,allow_mode_=false,allow_mission_=false,raw_enabled_=false,auto_sensor_streams_=true;
  bool allow_velocity_=false,rover_confirmed_=false;
  bool avoidance_tx_{false},avoidance_fw_confirmed_{false};
  uint32_t avoidance_session_{0};
  uint16_t avoidance_seq_{0};
  bool avoidance_received_{false};
  std::mutex avoidance_mutex_;
  std::chrono::steady_clock::time_point avoidance_last_rx_{};
  pixhawk_interfaces::msg::AvoidanceAdvisory avoidance_last_;
  rclcpp::Subscription<pixhawk_interfaces::msg::AvoidanceAdvisory>::SharedPtr avoidance_sub_;
  double max_accuracy_=5.,max_speed_=30.,hb_timeout_=3.;
  int fd_=-1;std::mutex fd_mutex_;std::thread io_thread_;std::atomic<bool> running_{false};
  std::atomic<bool> port_open_{false};std::atomic<uint8_t> target_sys_{0},target_comp_{0};
  std::atomic<bool> armed_{false},gps_valid_{false},local_valid_{false},airframe_valid_{false};
  std::atomic<uint8_t> mav_state_{0},mav_type_{0},base_mode_{0};std::atomic<uint32_t> custom_mode_{0};
  std::atomic<bool> manual_takeover_latched_{false},px4_autopilot_{false};
  std::atomic<uint64_t> rx_packets_{0},tx_packets_{0},parse_errors_{0};
  std::mutex state_mutex_;std::chrono::steady_clock::time_point last_heartbeat_{},last_local_{},last_gps_{},last_rc_{};
  bool rc_message_valid_=false;
  Quaternion attitude_{};bool have_attitude_=false;double pos_accuracy_=1.e9;double local_x_=0,local_y_=0,local_z_=0;
  double gps_lat_=0,gps_lon_=0,gps_alt_=0;
  std::mutex cv_mutex_;std::condition_variable cv_;
  std::unordered_map<std::string,ParamValue> param_cache_;uint64_t param_seq_=0;
  uint64_t ack_seq_=0;uint16_t ack_cmd_=0;uint8_t ack_result_=255;
  std::unordered_map<uint16_t,std::pair<uint64_t,uint8_t>> ack_by_command_;
  std::mutex command_gate_,param_gate_,mission_gate_;
  std::atomic<int> aux_stream_idx_{0};
  uint16_t mission_count_=0;
  uint64_t mission_count_seq_=0,mission_item_seq_=0,mission_ack_seq_=0,mission_request_seq_=0;
  uint16_t mission_request_index_=0;
  uint8_t mission_count_type_=255,mission_item_type_=255,mission_ack_type_=255,mission_ack_result_=255,mission_request_type_=255;
  mavlink_mission_item_int_t last_mission_item_{};
  std::array<std::atomic<uint64_t>,3> imu_bank_counts_{};
  std::array<std::chrono::steady_clock::time_point,3> imu_bank_last_{};

  std::mutex sp_mutex_;Setpoint setpoint_{};
  std::chrono::steady_clock::time_point last_send_heartbeat_{},last_status_{},last_sp_{},
    last_avoidance_send_{},start_=std::chrono::steady_clock::now();
  rclcpp::TimerBase::SharedPtr periodic_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  std::array<rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr,3> imu_banks_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr estimator_pub_,vibration_pub_,sensor_health_pub_,imu_availability_pub_,mission_pub_,raw_odometry_pub_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
  rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr baro_pub_;
  rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr attitude_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_,param_pub_,event_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr raw_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rover_feedback_pub_,version_pub_;
  std::vector<rclcpp::ServiceBase::SharedPtr> services_;

  static speed_t speed(int n){
    switch(n){case 9600:return B9600;case 57600:return B57600;case 115200:return B115200;
      case 230400:return B230400;
#ifdef B460800
      case 460800:return B460800;
#endif
      default:return 0;}
  }
  void close_port(){
    std::lock_guard<std::mutex> lk(fd_mutex_);
    if(fd_>=0){::flock(fd_,LOCK_UN);::close(fd_);fd_=-1;}port_open_=false;
    target_sys_=0;target_comp_=0;gps_valid_=false;local_valid_=false;
    aux_stream_idx_=0;
    std::lock_guard<std::mutex> sp(sp_mutex_);setpoint_.active=false;
  }
  bool open_port(){
    speed_t baud=speed(baud_);if(!baud){RCLCPP_ERROR(get_logger(),"Unsupported baud %d",baud_);return false;}
    int fd=::open(device_.c_str(),O_RDWR|O_NOCTTY|O_NONBLOCK|O_CLOEXEC);
    if(fd<0)return false;
    if(::flock(fd,LOCK_EX|LOCK_NB)<0){::close(fd);return false;}
    struct termios tty{};
    if(tcgetattr(fd,&tty)<0){::close(fd);return false;}
    cfmakeraw(&tty);cfsetispeed(&tty,baud);cfsetospeed(&tty,baud);
    tty.c_cflag|=CLOCAL|CREAD;tty.c_cflag&=~CRTSCTS;
    tty.c_cc[VMIN]=0;tty.c_cc[VTIME]=0;
    if(tcsetattr(fd,TCSANOW,&tty)<0){::close(fd);return false;}
    {std::lock_guard<std::mutex> lk(fd_mutex_);fd_=fd;port_open_=true;}
    RCLCPP_INFO(get_logger(),"Serial opened %s",device_.c_str());return true;
  }
  bool send(const mavlink_message_t &msg){
    std::lock_guard<std::mutex> lk(fd_mutex_);
    if(fd_<0)return false;
    uint8_t bytes[MAVLINK_MAX_PACKET_LEN];auto size=mavlink_msg_to_send_buffer(bytes,&msg);size_t sent=0;
    while(sent<size){
      ssize_t n=::write(fd_,bytes+sent,size-sent);
      if(n>0){sent+=n;continue;}
      if(n<0&&errno==EINTR)continue;
      if(n<0&&(errno==EAGAIN||errno==EWOULDBLOCK)){
        struct pollfd p{fd_,POLLOUT,0};if(::poll(&p,1,150)>0)continue;
      }
      return false;
    }
    ++tx_packets_;return true;
  }
  bool connected(){
    std::lock_guard<std::mutex> lk(state_mutex_);
    return port_open_ && target_sys_!=0 &&
      std::chrono::duration<double>(std::chrono::steady_clock::now()-last_heartbeat_).count()<hb_timeout_;
  }
  void io_loop(){
    bool warned=false;
    while(running_){
      if(!port_open_){
        if(!open_port()){
          if(!warned){RCLCPP_WARN(get_logger(),"Serial unavailable or in use: %s",device_.c_str());warned=true;}
          std::this_thread::sleep_for(1s);continue;
        }warned=false;
      }
      int f;{std::lock_guard<std::mutex> lk(fd_mutex_);f=fd_;}
      struct pollfd pfd{f,static_cast<short>(POLLIN|POLLERR|POLLHUP),0};
      int ready=::poll(&pfd,1,100);
      if(ready<0&&errno==EINTR)continue;
      if(ready<0||(ready>0&&(pfd.revents&(POLLERR|POLLHUP|POLLNVAL)))){close_port();continue;}
      if(ready>0&&(pfd.revents&POLLIN)){
        uint8_t bytes[4096];ssize_t n=::read(f,bytes,sizeof(bytes));
        if(n<0&&(errno==EINTR||errno==EAGAIN))continue;
        if(n<0){close_port();continue;}
        for(ssize_t i=0;i<n;i++){
          mavlink_message_t m{};mavlink_status_t status{};
          if(mavlink_parse_char(MAVLINK_COMM_0,bytes[i],&m,&status)){++rx_packets_;receive(m);}
          if(status.parse_error)++parse_errors_;
        }
      }
    }
  }

  void receive(const mavlink_message_t &m){
    // Reject messages from any other system once a target is identified.
    if(target_sys_!=0 && m.sysid!=target_sys_)return;
    const auto now=std::chrono::steady_clock::now();
    const auto stamp=get_clock()->now();
    if(raw_pub_){
      uint8_t buf[MAVLINK_MAX_PACKET_LEN];auto n=mavlink_msg_to_send_buffer(buf,&m);
      std_msgs::msg::UInt8MultiArray arr;arr.data.assign(buf,buf+n);raw_pub_->publish(arr);
    }
    switch(m.msgid){
      case MAVLINK_MSG_ID_HEARTBEAT:{
        mavlink_heartbeat_t v{};mavlink_msg_heartbeat_decode(&m,&v);
        if(v.autopilot!=MAV_AUTOPILOT_PX4 && v.autopilot!=MAV_AUTOPILOT_ARDUPILOTMEGA)break;
        bool first=target_sys_==0;
        target_sys_=m.sysid;target_comp_=m.compid;
        mav_type_=v.type;mav_state_=v.system_status;custom_mode_=v.custom_mode;
        base_mode_=v.base_mode;px4_autopilot_=(v.autopilot==MAV_AUTOPILOT_PX4);
        // The RC flight-mode switch belongs to PX4, never to the ROS process.
        // Latch manual takeover until a controlled new bridge session.
        if(px4_autopilot_ && ((v.custom_mode>>16)&0xff)==1) {
          manual_takeover_latched_=true;
          std::lock_guard<std::mutex> lock(sp_mutex_);
          setpoint_.active=false;
        }
        armed_=(v.base_mode&MAV_MODE_FLAG_SAFETY_ARMED)!=0;
        {std::lock_guard<std::mutex> lk(state_mutex_);last_heartbeat_=now;}
        if(first){
          RCLCPP_INFO(get_logger(),"MAVLink HEARTBEAT sys=%u comp=%u autopilot=%u vehicle_type=%u",
                      m.sysid,m.compid,v.autopilot,v.type);
          mavlink_message_t req{};
          // MAVLink generic stream request; no flight-state changes.
          mavlink_msg_request_data_stream_pack(245,191,&req,m.sysid,m.compid,MAV_DATA_STREAM_ALL,2,1);send(req);
          auto par=param_id_buf("SYS_AUTOSTART");
          mavlink_msg_param_request_read_pack(245,191,&req,m.sysid,m.compid,par.data(),-1);send(req);
          // Version discovery is read-only; single existing serial owner.
          mavlink_msg_command_long_pack(245,191,&req,m.sysid,m.compid,
            MAV_CMD_REQUEST_MESSAGE,0,
            static_cast<float>(MAVLINK_MSG_ID_AUTOPILOT_VERSION),0,0,0,0,0,0);
          send(req);
        }
        break;
      }
      case MAVLINK_MSG_ID_AUTOPILOT_VERSION:{
        mavlink_autopilot_version_t v{};
        mavlink_msg_autopilot_version_decode(&m,&v);
        std::ostringstream o;
        o << "version_raw=" << v.flight_sw_version
          << " board=" << v.board_version
          << " capabilities=" << v.capabilities << " git=";
        for (int k=0;k<8;++k)
          o << std::hex << std::setw(2) << std::setfill('0')
            << static_cast<unsigned int>(v.flight_custom_version[k]);
        std_msgs::msg::String result;
        result.data=o.str();version_pub_->publish(result);
        RCLCPP_INFO(get_logger(),"Read-only AUTOPILOT_VERSION: %s",result.data.c_str());
        break;
      }
      case MAVLINK_MSG_ID_PARAM_VALUE:{
        mavlink_param_value_t v{};mavlink_msg_param_value_decode(&m,&v);
        std::string key(v.param_id,strnlen(v.param_id,16));
        if(key.empty())break;
        double value=decode_param(v.param_value,v.param_type);
        {std::lock_guard<std::mutex> lk(cv_mutex_);param_cache_[key]={value,v.param_type,++param_seq_};}
        cv_.notify_all();
        if(key=="SYS_AUTOSTART")airframe_valid_=(value>0.5);
        std_msgs::msg::String out;std::ostringstream s;
        s<<"{\"name\":\""<<key<<"\",\"type\":"<<static_cast<int>(v.param_type)
         <<",\"value\":"<<std::setprecision(12)<<value<<",\"index\":"<<v.param_index
         <<",\"count\":"<<v.param_count<<"}";out.data=s.str();param_pub_->publish(out);
        break;
      }
      case MAVLINK_MSG_ID_MISSION_CURRENT:{
        mavlink_mission_current_t v{};mavlink_msg_mission_current_decode(&m,&v);
        std_msgs::msg::String evt;
        evt.data="{\"type\":\"MISSION_CURRENT\",\"seq\":"+
          std::to_string(v.seq)+",\"total\":"+std::to_string(v.total)+"}";
        mission_pub_->publish(evt);break;
      }
      case MAVLINK_MSG_ID_MISSION_ITEM_REACHED:{
        mavlink_mission_item_reached_t v{};mavlink_msg_mission_item_reached_decode(&m,&v);
        std_msgs::msg::String evt;evt.data=
          "{\"type\":\"MISSION_ITEM_REACHED\",\"seq\":"+std::to_string(v.seq)+"}";
        mission_pub_->publish(evt);break;
      }
      case MAVLINK_MSG_ID_RC_CHANNELS:{
        mavlink_rc_channels_t v{};mavlink_msg_rc_channels_decode(&m,&v);
        const bool plausible=(v.chan1_raw>=800 && v.chan1_raw<=2200) ||
          (v.chan2_raw>=800 && v.chan2_raw<=2200) ||
          (v.chan3_raw>=800 && v.chan3_raw<=2200) ||
          (v.chan4_raw>=800 && v.chan4_raw<=2200);
        {std::lock_guard<std::mutex> lock(state_mutex_);
          last_rc_=now;rc_message_valid_=plausible;}
        std_msgs::msg::String msg;std::ostringstream o;
        o<<"{\"type\":\"RC_CHANNELS\",\"ch1\":"<<v.chan1_raw
         <<",\"ch2\":"<<v.chan2_raw<<",\"ch3\":"<<v.chan3_raw
         <<",\"ch4\":"<<v.chan4_raw<<",\"rssi\":"<<static_cast<int>(v.rssi)<<"}";
        msg.data=o.str();rover_feedback_pub_->publish(msg);break;
      }
      case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:{
        mavlink_servo_output_raw_t v{};mavlink_msg_servo_output_raw_decode(&m,&v);
        std_msgs::msg::String msg;std::ostringstream o;
        // Servo PWM is command/actuator output, NOT a measured wheel steering angle.
        o<<"{\"type\":\"SERVO_OUTPUT_RAW\",\"port\":"<<static_cast<int>(v.port)
         <<",\"servo1_pwm\":"<<v.servo1_raw
         <<",\"servo2_pwm\":"<<v.servo2_raw
         <<",\"servo3_pwm\":"<<v.servo3_raw
         <<",\"servo4_pwm\":"<<v.servo4_raw<<"}";
        msg.data=o.str();rover_feedback_pub_->publish(msg);break;
      }
      case MAVLINK_MSG_ID_VFR_HUD:{
        mavlink_vfr_hud_t v{};mavlink_msg_vfr_hud_decode(&m,&v);
        std_msgs::msg::String msg;std::ostringstream o;
        o<<"{\"type\":\"VFR_HUD\",\"ground_speed_mps\":"<<json_num(v.groundspeed)
         <<",\"heading_deg\":"<<v.heading<<"}";
        msg.data=o.str();rover_feedback_pub_->publish(msg);break;
      }
      case MAVLINK_MSG_ID_MISSION_COUNT:{
        mavlink_mission_count_t v{};mavlink_msg_mission_count_decode(&m,&v);
        {std::lock_guard<std::mutex> lk(cv_mutex_);mission_count_=v.count;mission_count_type_=v.mission_type;++mission_count_seq_;}
        cv_.notify_all();break;
      }
      case MAVLINK_MSG_ID_MISSION_ITEM_INT:{
        mavlink_mission_item_int_t v{};mavlink_msg_mission_item_int_decode(&m,&v);
        {std::lock_guard<std::mutex> lk(cv_mutex_);last_mission_item_=v;mission_item_type_=v.mission_type;++mission_item_seq_;}
        cv_.notify_all();break;
      }
      case MAVLINK_MSG_ID_MISSION_ACK:{
        mavlink_mission_ack_t v{};mavlink_msg_mission_ack_decode(&m,&v);
        {std::lock_guard<std::mutex> lk(cv_mutex_);
          mission_ack_type_=v.mission_type;mission_ack_result_=v.type;++mission_ack_seq_;}
        cv_.notify_all();
        std_msgs::msg::String event;event.data="MISSION_ACK type="+std::to_string(v.type);
        mission_pub_->publish(event);break;
      }
      case MAVLINK_MSG_ID_MISSION_REQUEST_INT:
      case MAVLINK_MSG_ID_MISSION_REQUEST:{
        uint16_t seq=0;uint8_t mission_type=0;
        if(m.msgid==MAVLINK_MSG_ID_MISSION_REQUEST_INT){
          mavlink_mission_request_int_t v{};mavlink_msg_mission_request_int_decode(&m,&v);
          seq=v.seq;mission_type=v.mission_type;
        }else{
          mavlink_mission_request_t v{};mavlink_msg_mission_request_decode(&m,&v);
          seq=v.seq;mission_type=v.mission_type;
        }
        {std::lock_guard<std::mutex> lk(cv_mutex_);mission_request_index_=seq;mission_request_type_=mission_type;++mission_request_seq_;}
        cv_.notify_all();break;
      }
      case MAVLINK_MSG_ID_COMMAND_ACK:{
        mavlink_command_ack_t v{};mavlink_msg_command_ack_decode(&m,&v);
        {std::lock_guard<std::mutex> lk(cv_mutex_);ack_cmd_=v.command;ack_result_=v.result;
          ++ack_seq_;ack_by_command_[v.command]={ack_seq_,v.result};}
        cv_.notify_all();
        break;
      }
      case MAVLINK_MSG_ID_STATUSTEXT:{
        mavlink_statustext_t v{};mavlink_msg_statustext_decode(&m,&v);
        std_msgs::msg::String evt;
        std::string text(v.text,strnlen(v.text,50));
        evt.data="PX4 ["+std::to_string(v.severity)+"] "+text;
        event_pub_->publish(evt);
        RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),1000,"%s",evt.data.c_str());
        break;
      }
      case MAVLINK_MSG_ID_ATTITUDE_QUATERNION:{
        mavlink_attitude_quaternion_t v{};mavlink_msg_attitude_quaternion_decode(&m,&v);
        Quaternion q{v.q1,v.q2,v.q3,v.q4};
        {std::lock_guard<std::mutex> lk(state_mutex_);attitude_=q;have_attitude_=true;}
        geometry_msgs::msg::QuaternionStamped out;
        out.header.stamp=stamp;out.header.frame_id="pixhawk_local_enu";
        out.quaternion=to_msg(to_ros_orientation(q));
        attitude_pub_->publish(out);
        std_msgs::msg::String feedback;
        feedback.data="{\"type\":\"ATTITUDE_QUATERNION\",\"yaw_rate_flu_radps\":"+
          json_num(-v.yawspeed)+"}";
        rover_feedback_pub_->publish(feedback);
        break;
      }
      case MAVLINK_MSG_ID_HIGHRES_IMU:{
        mavlink_highres_imu_t v{};mavlink_msg_highres_imu_decode(&m,&v);
        sensor_msgs::msg::Imu imu;imu.header.stamp=stamp;imu.header.frame_id="pixhawk_imu_flu";
        imu.linear_acceleration.x=v.xacc;imu.linear_acceleration.y=-v.yacc;imu.linear_acceleration.z=-v.zacc;
        imu.angular_velocity.x=v.xgyro;imu.angular_velocity.y=-v.ygyro;imu.angular_velocity.z=-v.zgyro;
        {std::lock_guard<std::mutex> lk(state_mutex_);
          if(have_attitude_)imu.orientation=to_msg(to_ros_orientation(attitude_));
          else imu.orientation_covariance[0]=-1.;
        }
        imu_pub_->publish(imu);
        sensor_msgs::msg::MagneticField mag;mag.header=imu.header;
        // MAVLink HIGHRES_IMU magnetic flux is in gauss, ROS requires tesla.
        mag.magnetic_field.x=v.xmag*1.e-4;mag.magnetic_field.y=-v.ymag*1.e-4;mag.magnetic_field.z=-v.zmag*1.e-4;
        mag_pub_->publish(mag);
        if(v.abs_pressure>0 && is_finite_value(v.abs_pressure)){
          sensor_msgs::msg::FluidPressure baro;baro.header=imu.header;
          baro.fluid_pressure=v.abs_pressure*100.;baro_pub_->publish(baro);
        }
        break;
      }
      case MAVLINK_MSG_ID_GPS_RAW_INT:{
        mavlink_gps_raw_int_t v{};mavlink_msg_gps_raw_int_decode(&m,&v);
        bool fix=v.fix_type>=3&&v.lat!=0&&v.lon!=0;
        gps_valid_=fix;
        {std::lock_guard<std::mutex> lk(state_mutex_);
          last_gps_=now;gps_lat_=v.lat*1.e-7;gps_lon_=v.lon*1.e-7;gps_alt_=v.alt*.001;}
        sensor_msgs::msg::NavSatFix out;
        out.header.stamp=stamp;out.header.frame_id="pixhawk_gps";
        out.status.status=fix?sensor_msgs::msg::NavSatStatus::STATUS_FIX:sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
        out.status.service=sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
        out.latitude=v.lat*1.e-7;out.longitude=v.lon*1.e-7;out.altitude=v.alt*1.e-3;
        if(v.eph!=UINT16_MAX){double e=v.eph*.01;out.position_covariance[0]=e*e;
          out.position_covariance[4]=e*e;out.position_covariance_type=sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;}
        gps_pub_->publish(out);break;
      }
      case MAVLINK_MSG_ID_SCALED_IMU:
      case MAVLINK_MSG_ID_SCALED_IMU2:
      case MAVLINK_MSG_ID_SCALED_IMU3:{
        size_t slot=m.msgid==MAVLINK_MSG_ID_SCALED_IMU?0:(m.msgid==MAVLINK_MSG_ID_SCALED_IMU2?1:2);
        int16_t ax=0,ay=0,az=0,gx=0,gy=0,gz=0;
        if(slot==0){mavlink_scaled_imu_t v{};mavlink_msg_scaled_imu_decode(&m,&v);
          ax=v.xacc;ay=v.yacc;az=v.zacc;gx=v.xgyro;gy=v.ygyro;gz=v.zgyro;}
        if(slot==1){mavlink_scaled_imu2_t v{};mavlink_msg_scaled_imu2_decode(&m,&v);
          ax=v.xacc;ay=v.yacc;az=v.zacc;gx=v.xgyro;gy=v.ygyro;gz=v.zgyro;}
        if(slot==2){mavlink_scaled_imu3_t v{};mavlink_msg_scaled_imu3_decode(&m,&v);
          ax=v.xacc;ay=v.yacc;az=v.zacc;gx=v.xgyro;gy=v.ygyro;gz=v.zgyro;}
        sensor_msgs::msg::Imu out;out.header.stamp=stamp;
        out.header.frame_id="pixhawk_imu_instance_"+std::to_string(slot)+"_flu";
        // SCALED_IMU: milli-g -> m/s^2; millirad/s -> rad/s, FRD to FLU.
        constexpr double a=9.80665e-3,g=1.e-3;
        out.linear_acceleration.x=ax*a;out.linear_acceleration.y=-ay*a;out.linear_acceleration.z=-az*a;
        out.angular_velocity.x=gx*g;out.angular_velocity.y=-gy*g;out.angular_velocity.z=-gz*g;
        out.orientation_covariance[0]=-1.0; // This message carries no orientation solution.
        imu_bank_counts_[slot]++;
        {std::lock_guard<std::mutex> lk(state_mutex_);imu_bank_last_[slot]=now;}
        imu_banks_[slot]->publish(out);
        break;
      }
      case MAVLINK_MSG_ID_VIBRATION:{
        mavlink_vibration_t v{};mavlink_msg_vibration_decode(&m,&v);
        std::ostringstream o;o<<"{\"x\":"<<v.vibration_x<<",\"y\":"<<v.vibration_y
          <<",\"z\":"<<v.vibration_z<<",\"clipping\":["
          <<v.clipping_0<<","<<v.clipping_1<<","<<v.clipping_2<<"]}";
        std_msgs::msg::String result;result.data=o.str();vibration_pub_->publish(result);break;
      }
      case MAVLINK_MSG_ID_ESTIMATOR_STATUS:{
        mavlink_estimator_status_t v{};mavlink_msg_estimator_status_decode(&m,&v);
        std::ostringstream o;o<<"{\"flags\":"<<v.flags
          <<",\"horizontal_accuracy_m\":"<<json_num(v.pos_horiz_accuracy)
          <<",\"vertical_accuracy_m\":"<<json_num(v.pos_vert_accuracy)
          <<",\"velocity_test_ratio\":"<<json_num(v.vel_ratio)
          <<",\"position_horiz_test_ratio\":"<<json_num(v.pos_horiz_ratio)
          <<",\"position_vert_test_ratio\":"<<json_num(v.pos_vert_ratio)
          <<",\"mag_test_ratio\":"<<json_num(v.mag_ratio)<<"}";
        std_msgs::msg::String result;result.data=o.str();estimator_pub_->publish(result);
        {std::lock_guard<std::mutex> lk(state_mutex_);
          pos_accuracy_=is_finite_value(v.pos_horiz_accuracy)?v.pos_horiz_accuracy:1.e9;
        }
        break;
      }
      case MAVLINK_MSG_ID_LOCAL_POSITION_NED:{
        mavlink_local_position_ned_t v{};mavlink_msg_local_position_ned_decode(&m,&v);
        double acc;Quaternion q;bool have_q;
        {std::lock_guard<std::mutex> lk(state_mutex_);
          local_x_=v.x;local_y_=v.y;local_z_=v.z;last_local_=now;
          acc=pos_accuracy_;q=attitude_;have_q=have_attitude_;
        }
        bool ok=is_finite_value(v.x)&&is_finite_value(v.y)&&is_finite_value(v.z)&&
          is_finite_value(v.vx)&&is_finite_value(v.vy)&&is_finite_value(v.vz)&&
          std::hypot(v.vx,v.vy)<max_speed_ &&
          std::hypot(v.x,v.y)<100000. && acc<max_accuracy_;
        local_valid_=ok;
        nav_msgs::msg::Odometry out;out.header.stamp=stamp;
        out.header.frame_id="pixhawk_local_enu";out.child_frame_id="pixhawk_body_flu";
        out.pose.pose.position.x=v.y;out.pose.pose.position.y=v.x;out.pose.pose.position.z=-v.z;
        out.twist.twist.linear.x=v.vy;out.twist.twist.linear.y=v.vx;out.twist.twist.linear.z=-v.vz;
        if(have_q)out.pose.pose.orientation=to_msg(to_ros_orientation(q));
        else out.pose.pose.orientation.w=1;
        // Mark estimator drift as unusable to downstream fusers.
        out.pose.covariance[0]=ok?acc*acc:1.e8;
        out.pose.covariance[7]=ok?acc*acc:1.e8;
        out.pose.covariance[14]=ok?acc*acc:1.e8;
        odom_pub_->publish(out);break;
      }
      case MAVLINK_MSG_ID_ODOMETRY:{
        mavlink_odometry_t v{};mavlink_msg_odometry_decode(&m,&v);
        std_msgs::msg::String msg;std::ostringstream out;
        out<<"{\"frame_id\":"<<static_cast<int>(v.frame_id)
          <<",\"child_frame_id\":"<<static_cast<int>(v.child_frame_id)
          <<",\"estimator_type\":"<<static_cast<int>(v.estimator_type)
          <<",\"quality\":"<<static_cast<int>(v.quality)
          <<",\"reset_counter\":"<<static_cast<int>(v.reset_counter)
          <<",\"position_ned_m\":["<<json_num(v.x)<<","<<json_num(v.y)
          <<","<<json_num(v.z)<<"]"
          <<",\"velocity_ned_mps\":["<<json_num(v.vx)<<","<<json_num(v.vy)
          <<","<<json_num(v.vz)<<"]"
          <<",\"pose_covariance_x\":"<<json_num(v.pose_covariance[0])
          <<",\"pose_covariance_y\":"<<json_num(v.pose_covariance[6])
          <<",\"pose_covariance_z\":"<<json_num(v.pose_covariance[11])
          <<"}";
        msg.data=out.str();raw_odometry_pub_->publish(msg);
        break;
      }
      case MAVLINK_MSG_ID_SYS_STATUS:{
        mavlink_sys_status_t v{};mavlink_msg_sys_status_decode(&m,&v);
        std_msgs::msg::String stat;
        std::ostringstream health;
        health<<"{\"present\":"<<v.onboard_control_sensors_present
          <<",\"enabled\":"<<v.onboard_control_sensors_enabled
          <<",\"health\":"<<v.onboard_control_sensors_health
          <<",\"errors_comm\":"<<v.errors_comm
          <<",\"drop_rate_comm\":"<<v.drop_rate_comm
          <<",\"cpu_load_percent\":"<<(v.load*0.1)<<"}";
        stat.data=health.str();sensor_health_pub_->publish(stat);
        sensor_msgs::msg::BatteryState out;out.header.stamp=stamp;
        out.voltage=v.voltage_battery==UINT16_MAX?std::numeric_limits<float>::quiet_NaN():v.voltage_battery*.001f;
        out.current=v.current_battery<0?std::numeric_limits<float>::quiet_NaN():v.current_battery*.01f;
        out.percentage=v.battery_remaining<0?std::numeric_limits<float>::quiet_NaN():v.battery_remaining*.01f;
        out.present=(v.voltage_battery!=UINT16_MAX);
        battery_pub_->publish(out);break;
      }
      default:break;
    }
  }
  void tick(){
    auto now=std::chrono::steady_clock::now();
    if(auto_sensor_streams_ && connected() && aux_stream_idx_<5){
      constexpr std::array<uint32_t,5> ids={26,116,129,230,241};
      constexpr std::array<float,5> intervals={100000,100000,100000,500000,500000};
      mavlink_message_t req{};
      mavlink_msg_command_long_pack(245,191,&req,target_sys_,target_comp_,
        MAV_CMD_SET_MESSAGE_INTERVAL,0,
        static_cast<float>(ids[aux_stream_idx_]),intervals[aux_stream_idx_],0,0,0,0,0);
      if(send(req))++aux_stream_idx_;
    }
    if(now-last_send_heartbeat_>=1s){
      last_send_heartbeat_=now;
      if(port_open_){
        mavlink_message_t msg{};
        mavlink_msg_heartbeat_pack(245,MAV_COMP_ID_ONBOARD_COMPUTER,&msg,
          MAV_TYPE_ONBOARD_CONTROLLER,MAV_AUTOPILOT_INVALID,0,0,MAV_STATE_ACTIVE);
        send(msg);
      }
    }
    if(now-last_status_>=1s){
      std_msgs::msg::String imuinfo;std::ostringstream ii;
      ii<<"{";
      for(size_t k=0;k<3;k++){
        if(k)ii<<",";
        bool fresh=false;
        {std::lock_guard<std::mutex> lk(state_mutex_);
          fresh=imu_bank_counts_[k]>0 && now-imu_bank_last_[k]<3s;
        }
        ii<<"\"imu"<<k<<"\":{\"messages\":"<<imu_bank_counts_[k]
          <<",\"fresh\":"<<(fresh?"true":"false")<<"}";
      }
      ii<<"}";imuinfo.data=ii.str();imu_availability_pub_->publish(imuinfo);
      last_status_=now;
      bool live=connected();
      const uint32_t cm=custom_mode_.load();
      const int px4_main=px4_autopilot_?static_cast<int>((cm>>16)&0xff):-1;
      const int px4_sub=px4_autopilot_?static_cast<int>((cm>>24)&0xff):-1;
      bool rc_telemetry_fresh=false;
      {std::lock_guard<std::mutex> lock(state_mutex_);
        rc_telemetry_fresh=rc_message_valid_ && last_rc_!=std::chrono::steady_clock::time_point{} &&
          now-last_rc_<1500ms;
      }
      double acc;{
        std::lock_guard<std::mutex> lk(state_mutex_);acc=pos_accuracy_;
      }
      std_msgs::msg::String out;std::ostringstream o;
      o<<"{\"connected\":"<<(live?"true":"false")<<",\"port_open\":"<<(port_open_?"true":"false")
       <<",\"sysid\":"<<static_cast<int>(target_sys_)
       <<",\"armed\":"<<(armed_?"true":"false")
       <<",\"mav_state\":"<<static_cast<int>(mav_state_)
       <<",\"vehicle_type\":"<<static_cast<int>(mav_type_)
       <<",\"custom_mode\":"<<custom_mode_
       <<",\"base_mode\":"<<static_cast<int>(base_mode_)
       <<",\"px4_main_mode\":"<<static_cast<int>(px4_main)
       <<",\"px4_sub_mode\":"<<static_cast<int>(px4_sub)
       <<",\"manual_mode_selected\":"<<((live&&px4_main==1)?"true":"false")
       <<",\"manual_takeover_latched\":"<<(manual_takeover_latched_?"true":"false")
       <<",\"rc_telemetry_fresh\":"<<(rc_telemetry_fresh?"true":"false")
       <<",\"airframe_configured\":"<<(airframe_valid_?"true":"false")
       <<",\"gps_valid\":"<<(gps_valid_?"true":"false")
       <<",\"local_valid\":"<<(local_valid_?"true":"false")
       <<",\"horizontal_accuracy_m\":"<<acc
       <<",\"rx\":"<<rx_packets_<<",\"tx\":"<<tx_packets_
       <<",\"parse_errors\":"<<parse_errors_<<"}";
      out.data=o.str();status_pub_->publish(out);
    }
    if (avoidance_tx_ && avoidance_fw_confirmed_ &&
        connected() && airframe_valid_ && now-last_avoidance_send_>=100ms) {
      last_avoidance_send_=now;
      pixhawk_interfaces::msg::AvoidanceAdvisory input;
      bool fresh=false;
      {
        std::lock_guard<std::mutex> lk(avoidance_mutex_);
        fresh=avoidance_received_ && (now-avoidance_last_rx_<250ms);
        if(fresh)input=avoidance_last_;
      }
      if(fresh) {
        avoidance::AdvisoryWire payload{};
        payload.flags=avoidance::VALID |
          (input.obstacle?avoidance::OBSTACLE:0) |
          ((input.stop||!input.valid)?avoidance::STOP:0);
        payload.sequence=++avoidance_seq_;
        payload.session=avoidance_session_;
        payload.vx_limit_mps=input.valid?input.vx_limit_mps:0.f;
        // ROS FLU: +yaw turns LEFT; PX4 FRD/NED: +yaw turns RIGHT.
        payload.yaw_correction_radps=input.valid?-input.yaw_correction_radps:0.f;
        payload.front_clearance_m=input.front_clearance_m;
        payload.left_clearance_m=input.left_clearance_m;
        payload.right_clearance_m=input.right_clearance_m;
        avoidance::seal(payload);
        // Generated MAVLink pack() copies all 128 bytes, regardless of payload_length.
        // Never hand it a pointer to the 36-byte wire struct (out-of-bounds read).
        std::array<uint8_t,128> padded_tunnel_payload{};
        std::memcpy(padded_tunnel_payload.data(),&payload,sizeof(payload));
        mavlink_message_t packet{};
        mavlink_msg_tunnel_pack(245,MAV_COMP_ID_ONBOARD_COMPUTER,&packet,
          target_sys_,target_comp_,avoidance::kTunnelPayloadType,sizeof(payload),
          padded_tunnel_payload.data());
        send(packet);
      }
    }
    if(now-last_sp_>=100ms){
      last_sp_=now;Setpoint sp;
      {std::lock_guard<std::mutex> lk(sp_mutex_);
        if(setpoint_.active && now>=setpoint_.expires)setpoint_.active=false;
        sp=setpoint_;
      }
      if(sp.active){
        if(manual_takeover_latched_) {
          std::lock_guard<std::mutex> lk(sp_mutex_);
          setpoint_.active=false;
          return;
        }
        if(sp.velocity) {
          const uint8_t px4_current_main_mode=static_cast<uint8_t>((custom_mode_.load()>>16)&0xff);
          if(!allow_velocity_||!rover_confirmed_||
             mav_type_!=MAV_TYPE_GROUND_ROVER||(px4_current_main_mode!=4&&px4_current_main_mode!=6)){
            std::lock_guard<std::mutex> lk(sp_mutex_);setpoint_.active=false;
            return; // PX4 MANUAL / POSCTL takeover cancels streaming.
          }
        }
        std::string reason;
        if(!motion_ready(sp.global,reason)){
          std::lock_guard<std::mutex> lk(sp_mutex_);setpoint_.active=false;return;
        }
        mavlink_message_t msg{};
        // Position + yaw; ignore speed, acceleration and yaw rate (MAVLink bitmask 0x9F8).
        constexpr uint16_t mask=0x09F8;
        const uint32_t t=static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>(now-start_).count());
        if(sp.velocity) {
          // Stock Rover: use velocity VECTOR, never pretend yaw_rate is obeyed.
          // 0x0DE7 ignores position, vertical velocity, acceleration, yaw/yawrate; keeps only vx/vy.
          constexpr uint16_t velocity_mask=0x0DE7;
          mavlink_msg_set_position_target_local_ned_pack(245,191,&msg,t,target_sys_,target_comp_,
            MAV_FRAME_LOCAL_NED,velocity_mask,0,0,0,sp.x,sp.y,0,0,0,0,0,0);
        } else if(sp.global)
          mavlink_msg_set_position_target_global_int_pack(245,191,&msg,t,target_sys_,target_comp_,
            MAV_FRAME_GLOBAL_INT,mask,sp.lat,sp.lon,sp.z,0,0,0,0,0,0,sp.yaw,0);
        else
          mavlink_msg_set_position_target_local_ned_pack(245,191,&msg,t,target_sys_,target_comp_,
            MAV_FRAME_LOCAL_NED,mask,sp.x,sp.y,sp.z,0,0,0,0,0,0,sp.yaw,0);
        send(msg);
      }
    }
  }

  bool send_command(uint16_t cmd,const std::array<float,7>&par,uint8_t &result,std::string &why){
    if(!connected()){why="No MAVLink heartbeat";return false;}
    std::lock_guard<std::mutex> gate(command_gate_);
    uint64_t seq;{std::lock_guard<std::mutex> lk(cv_mutex_);seq=ack_by_command_[cmd].first;}
    mavlink_message_t msg{};
    mavlink_msg_command_long_pack(245,191,&msg,target_sys_,target_comp_,cmd,0,
      par[0],par[1],par[2],par[3],par[4],par[5],par[6]);
    if(!send(msg)){why="Serial write failed";return false;}
    std::unique_lock<std::mutex> lk(cv_mutex_);
    bool ack=cv_.wait_for(lk,2300ms,[&]{return ack_by_command_[cmd].first>seq;});
    if(!ack){why="COMMAND_ACK timeout (command may still have executed)";result=255;return false;}
    result=ack_by_command_[cmd].second;
    why="COMMAND_ACK result="+std::to_string(result);
    return result==MAV_RESULT_ACCEPTED;
  }
  bool is_read_only(uint16_t cmd)const{
    return cmd==MAV_CMD_REQUEST_MESSAGE||cmd==MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES||
      cmd==MAV_CMD_SET_MESSAGE_INTERVAL||cmd==MAV_CMD_REQUEST_PROTOCOL_VERSION;
  }
  bool motion_ready(bool global,std::string &reason){
    if(!connected()){reason="MAVLink heartbeat stale/unavailable";return false;}
    if(!enable_motion_){reason="enable_motion_commands=false; explicit opt-in required";return false;}
    if(!airframe_valid_||mav_state_==MAV_STATE_UNINIT){
      reason="PX4 unconfigured: SYS_AUTOSTART=0 or MAV_STATE_UNINIT";return false;
    }
    if(global && !gps_valid_){reason="No valid GPS 3D fix";return false;}
    if(!global && !local_valid_){reason="Estimator local position unreliable";return false;}
    const auto now=std::chrono::steady_clock::now();
    {std::lock_guard<std::mutex> lk(state_mutex_);
      if(global && now-last_gps_>3s){reason="GPS fix data stale";return false;}
      if(!global && now-last_local_>2s){reason="Local position data stale";return false;}
    }
    return true;
  }
  void setup_services(){
    auto cmd=create_service<iface::CommandLong>("command_long",
      [this](std::shared_ptr<iface::CommandLong::Request> r,std::shared_ptr<iface::CommandLong::Response> s){
        const bool read=is_read_only(r->command);
        if(manual_takeover_latched_ && !read &&
           !(r->command==MAV_CMD_COMPONENT_ARM_DISARM && r->params[0]<=0)){
          s->message="RC_MANUAL_TAKEOVER_LATCHED";return;
        }
        if(!read){
          if(!enable_motion_){s->message="Motion/configuration commands disabled";return;}
          if(!airframe_valid_ && !(r->command==MAV_CMD_COMPONENT_ARM_DISARM && r->params[0]==0)){
            s->message="PX4 airframe not configured";return;
          }
        }
        if(r->command==MAV_CMD_DO_SET_MODE && !allow_mode_){
          s->message="Changing PX4 mode requires allow_mode_changes=true";return;
        }
        if(r->command==MAV_CMD_DO_SET_MODE && r->params[1]==6){
          std::lock_guard<std::mutex> lk(sp_mutex_);
          if((!gps_valid_&&!local_valid_) || !setpoint_.active ||
              std::chrono::steady_clock::now()-setpoint_.activated<1s ||
              std::chrono::steady_clock::now()>=setpoint_.expires){
            s->message="OFFBOARD mode requires valid estimate and >=1s live setpoint prestream";return;
          }
        }
        if(r->command==MAV_CMD_COMPONENT_ARM_DISARM && r->params[0]>0 &&
           (mav_state_==MAV_STATE_UNINIT || r->params[1]!=0)){
          s->message="Arming disabled while uninitialized; force-arm bypass forbidden";return;
        }
        if(r->command==MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN && armed_){
          s->message="Cannot request flight-controller reboot while armed";return;
        }

        std::array<float,7> p{};std::copy(r->params.begin(),r->params.end(),p.begin());
        s->success=send_command(r->command,p,s->result,s->message);
      });services_.push_back(cmd);
    auto pg=create_service<iface::ParamGet>("param_get",
      [this](std::shared_ptr<iface::ParamGet::Request> r,std::shared_ptr<iface::ParamGet::Response> s){
        auto key=safe_param_name(r->name);
        if(key.empty()){s->message="Parameter name requires 1..16 A-Z 0-9 underscore";return;}
        if(!connected()){s->message="No MAVLink heartbeat";return;}
        std::lock_guard<std::mutex> gate(param_gate_);
        uint64_t seq;{std::lock_guard<std::mutex> lk(cv_mutex_);seq=param_cache_[key].seq;}
        mavlink_message_t msg{};
        auto keybuf=param_id_buf(key);
        mavlink_msg_param_request_read_pack(245,191,&msg,target_sys_,target_comp_,keybuf.data(),-1);
        if(!send(msg)){s->message="Serial write failed";return;}
        std::unique_lock<std::mutex> lk(cv_mutex_);
        bool ok=cv_.wait_for(lk,2300ms,[&]{return param_cache_[key].seq>seq;});
        if(!ok){s->message="Timed out waiting for PARAM_VALUE";return;}
        auto v=param_cache_[key];s->success=true;s->mav_type=v.type;s->value=v.value;s->message="PARAM_VALUE received";
      });services_.push_back(pg);
    auto ps=create_service<iface::ParamSet>("param_set",
      [this](std::shared_ptr<iface::ParamSet::Request> r,std::shared_ptr<iface::ParamSet::Response> s){
        auto key=safe_param_name(r->name);
        if(key.empty()){s->message="Invalid parameter name";return;}
        if(!allow_param_write_){s->message="allow_parameter_writes=false; explicit opt-in required";return;}
        if(!connected()){s->message="MAVLink disconnected";return;}
        if(armed_){s->message="Parameter changes require disarmed vehicle";return;}
        if(!is_finite_value(r->value)||r->value>2147483647.||r->value<-2147483648.){
          s->message="Value is NaN/Inf or exceeds safe 32-bit range";return;}
        if(r->mav_type!=MAV_PARAM_TYPE_REAL32 && r->mav_type!=MAV_PARAM_TYPE_INT32 &&
          r->mav_type!=MAV_PARAM_TYPE_UINT32 && r->mav_type!=MAV_PARAM_TYPE_INT16 &&
          r->mav_type!=MAV_PARAM_TYPE_UINT16 && r->mav_type!=MAV_PARAM_TYPE_INT8 &&
          r->mav_type!=MAV_PARAM_TYPE_UINT8){s->message="Unsupported MAV_PARAM_TYPE";return;}
        if(r->mav_type!=MAV_PARAM_TYPE_REAL32&&std::floor(r->value)!=r->value){
          s->message="Integer MAV_PARAM_TYPE requires integral value";return;}
        std::lock_guard<std::mutex> gate(param_gate_);
        uint64_t seq;{std::lock_guard<std::mutex> lk(cv_mutex_);seq=param_cache_[key].seq;}
        mavlink_message_t msg{};
        auto keybuf=param_id_buf(key);
        mavlink_msg_param_set_pack(245,191,&msg,target_sys_,target_comp_,keybuf.data(),
          encode_param(r->value,r->mav_type),r->mav_type);
        if(!send(msg)){s->message="Serial write failed";return;}
        std::unique_lock<std::mutex> lk(cv_mutex_);
        // PARAM_VALUE contains no transaction id. Old in-flight replies may precede the
        // write echo, so wait for the requested *value*, not merely any new PARAM_VALUE.
        auto deadline=std::chrono::steady_clock::now()+3500ms;
        while(std::chrono::steady_clock::now()<deadline){
          if(!cv_.wait_until(lk,deadline,[&]{return param_cache_[key].seq>seq;}))
            break;
          const auto v=param_cache_[key];
          seq=v.seq;
          s->applied_value=v.value;
          if(v.type==r->mav_type &&
             std::abs(v.value-r->value)<=std::max(1.e-4,std::abs(r->value)*1.e-5)){
            s->success=true;
            s->message="PARAM_VALUE requested-value echo verified";
            return;
          }
        }
        s->message="No matching PARAM_VALUE echo received; read parameter again to verify actual state";
      });services_.push_back(ps);
    auto list=create_service<std_srvs::srv::Trigger>("param_request_list",
      [this](std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> s){
        if(!connected()){s->message="Disconnected";return;}
        mavlink_message_t msg{};
        mavlink_msg_param_request_list_pack(245,191,&msg,target_sys_,target_comp_);
        s->success=send(msg);s->message=s->success?"Requested: subscribe to /pixhawk/parameters":"Write failed";
      });services_.push_back(list);
    auto interval=create_service<iface::MessageInterval>("set_message_interval",
      [this](std::shared_ptr<iface::MessageInterval::Request> r,
             std::shared_ptr<iface::MessageInterval::Response> s){
        if(r->interval_us< -1 || r->interval_us>10000000){
          s->message="interval_us must be -1 (disable), 0 (default) or <=10000000";return;
        }
        uint8_t result=255;
        s->success=send_command(MAV_CMD_SET_MESSAGE_INTERVAL,
          {static_cast<float>(r->message_id),static_cast<float>(r->interval_us),0,0,0,0,0},result,s->message);
      });services_.push_back(interval);

    auto download=create_service<iface::MissionDownload>("mission_download",
      [this](std::shared_ptr<iface::MissionDownload::Request> r,
             std::shared_ptr<iface::MissionDownload::Response> out){
        if(r->mission_type>2){out->message="Unsupported MAV_MISSION_TYPE";return;}
        if(!connected()){out->message="No MAVLink heartbeat";return;}
        std::lock_guard<std::mutex> gate(mission_gate_);
        const auto sys=target_sys_.load(),comp=target_comp_.load();
        uint64_t countseq;
        {std::lock_guard<std::mutex> lk(cv_mutex_);countseq=mission_count_seq_;}
        bool got_count=false;uint16_t count=0;
        for(int retry=0;retry<4&&!got_count;retry++){
          mavlink_message_t req{};
          mavlink_msg_mission_request_list_pack(245,191,&req,sys,comp,r->mission_type);
          if(!send(req)){out->message="MISSION_REQUEST_LIST serial failure";return;}
          std::unique_lock<std::mutex> lk(cv_mutex_);
          got_count=cv_.wait_for(lk,1500ms,[&]{
            return mission_count_seq_>countseq && mission_count_type_==r->mission_type;
          });
          if(got_count)count=mission_count_;
        }
        if(!got_count){out->message="MISSION_COUNT timeout";return;}
        if(count>1000){out->message="Mission item count > 1000 rejected";return;}
        for(uint16_t i=0;i<count;i++){
          uint64_t seq;{std::lock_guard<std::mutex> lk(cv_mutex_);seq=mission_item_seq_;}
          bool found=false; mavlink_mission_item_int_t item{};
          for(int retry=0;retry<4&&!found;retry++){
            if(!connected()){out->message="MAVLink disconnect during mission download";return;}
            mavlink_message_t req{};
            mavlink_msg_mission_request_int_pack(245,191,&req,sys,comp,i,r->mission_type);
            if(!send(req)){out->message="Mission request serial write failed";return;}
            std::unique_lock<std::mutex> lk(cv_mutex_);
            found=cv_.wait_for(lk,1600ms,[&]{
              return mission_item_seq_>seq && mission_item_type_==r->mission_type &&
                     last_mission_item_.seq==i;
            });
            if(found)item=last_mission_item_;
            seq=mission_item_seq_;
          }
          if(!found){out->message="Timeout downloading mission sequence "+std::to_string(i);return;}
          pixhawk_interfaces::msg::MissionItem out_item{};
          out_item.seq=i;out_item.mission_type=item.mission_type;out_item.command=item.command;
          out_item.frame=item.frame;out_item.current=item.current!=0;out_item.autocontinue=item.autocontinue!=0;
          out_item.params={item.param1,item.param2,item.param3,item.param4};
          out_item.x=item.x;out_item.y=item.y;out_item.z=item.z;
          out->items.push_back(out_item);
        }
        mavlink_message_t ack{};
        mavlink_msg_mission_ack_pack(245,191,&ack,sys,comp,MAV_MISSION_ACCEPTED,r->mission_type,0);
        send(ack);
        out->success=true;out->message="Downloaded "+std::to_string(out->items.size())+" mission items";
      });services_.push_back(download);
    auto upload=create_service<iface::MissionUpload>("mission_upload",
      [this](std::shared_ptr<iface::MissionUpload::Request> r,
             std::shared_ptr<iface::MissionUpload::Response> out){
        if(!connected()){out->message="Disconnected";return;}
        if(!allow_mission_ || !enable_motion_){
          out->message="Mission uploads disabled: allow_mission_writes and enable_motion_commands must be true";return;
        }
        if(!airframe_valid_ || armed_){out->message="Only configured, disarmed autopilot can upload missions";return;}
        if(r->mission_type>2){out->message="Unsupported MAV_MISSION_TYPE";return;}
        if(r->items.empty()){out->message="Empty mission would clear FC mission; use explicit clear operation";return;}
        if(r->items.size()>200){out->message="Mission >200 items rejected";return;}
        for(size_t i=0;i<r->items.size();i++){
          const auto &q=r->items[i];
          if(q.seq!=i || q.mission_type!=r->mission_type ||
             !is_finite_value(q.z)||!is_finite_value(q.params[0])||
             !is_finite_value(q.params[1])||!is_finite_value(q.params[2])||!is_finite_value(q.params[3])){
            out->message="Invalid mission item sequence/type/parameter";return;
          }
          if(q.command==0){out->message="Mission item command=0 invalid";return;}
          // MAV_CMD_NAV_WAYPOINT uses WGS84 lat/lon scaled by 1e7 in
          // MISSION_ITEM_INT for global coordinate frames. Never forward a
          // wildly invalid geographic target to a flight controller.
          if(r->mission_type==MAV_MISSION_TYPE_MISSION &&
             q.command==MAV_CMD_NAV_WAYPOINT) {
            const bool geographic_frame= q.frame==MAV_FRAME_GLOBAL ||
              q.frame==MAV_FRAME_GLOBAL_RELATIVE_ALT ||
              q.frame==MAV_FRAME_GLOBAL_INT ||
              q.frame==MAV_FRAME_GLOBAL_RELATIVE_ALT_INT ||
              q.frame==MAV_FRAME_GLOBAL_TERRAIN_ALT ||
              q.frame==MAV_FRAME_GLOBAL_TERRAIN_ALT_INT;
            const auto latitude=static_cast<int64_t>(q.x);
            const auto longitude=static_cast<int64_t>(q.y);
            if(!geographic_frame || latitude<-900000000LL ||
               latitude>900000000LL || longitude<-1800000000LL ||
               longitude>1800000000LL) {
              out->message="Mission waypoint invalid frame or WGS84 coordinate range";
              return;
            }
          }
        }
        std::lock_guard<std::mutex> gate(mission_gate_);
        const auto sys=target_sys_.load(),comp=target_comp_.load();
        uint64_t rqseq,ackseq;
        {std::lock_guard<std::mutex> lk(cv_mutex_);rqseq=mission_request_seq_;ackseq=mission_ack_seq_;}
        mavlink_message_t count{};
        mavlink_msg_mission_count_pack(245,191,&count,sys,comp,
          static_cast<uint16_t>(r->items.size()),r->mission_type,0);
        if(!send(count)){out->message="MISSION_COUNT serial failure";return;}
        auto limit=std::chrono::steady_clock::now()+30s;
        size_t sent=0;int retries=0;
        while(std::chrono::steady_clock::now()<limit){
          if(!connected()){out->message="MAVLink disconnected during mission upload";return;}
          std::unique_lock<std::mutex> lk(cv_mutex_);
          bool event=cv_.wait_for(lk,1800ms,[&]{
            return mission_request_seq_>rqseq || mission_ack_seq_>ackseq;
          });
          if(!event){
            lk.unlock();
            if(++retries>4){out->message="MISSION_REQUEST_INT timeout";return;}
            // Retransmit count only before the first item. Otherwise FC must re-request an item.
            if(sent==0)send(count);
            continue;
          }
          retries=0;
          if(mission_ack_seq_>ackseq){
            ackseq=mission_ack_seq_;
            if(mission_ack_type_==r->mission_type){
              auto result=mission_ack_result_;
              out->success=result==MAV_MISSION_ACCEPTED && sent==r->items.size();
              out->message="MISSION_ACK="+std::to_string(result)+" sent="+std::to_string(sent);
              return;
            }
          }
          if(mission_request_seq_>rqseq){
            rqseq=mission_request_seq_;auto i=mission_request_index_;
            auto type=mission_request_type_;
            lk.unlock();
            if(type!=r->mission_type || i>=r->items.size()){
              out->message="Invalid mission request seq/type";return;
            }
            const auto &q=r->items[i];
            mavlink_message_t m{};
            mavlink_msg_mission_item_int_pack(245,191,&m,sys,comp,i,q.frame,q.command,
              q.current?1:0,q.autocontinue?1:0,q.params[0],q.params[1],q.params[2],q.params[3],
              q.x,q.y,q.z,q.mission_type);
            if(!send(m)){out->message="MISSION_ITEM_INT write failure";return;}
            sent=std::max(sent,static_cast<size_t>(i+1));
          }
        }
        out->message="Mission upload exceeded 30 second transaction deadline";
      });services_.push_back(upload);
    auto local=create_service<iface::GotoLocal>("goto_local",
      [this](std::shared_ptr<iface::GotoLocal::Request> r,std::shared_ptr<iface::GotoLocal::Response> s){
        if(manual_takeover_latched_){s->message="RC_MANUAL_TAKEOVER_LATCHED";return;}
        if(!motion_ready(false,s->message))return;
        if(!is_finite_value(r->x)||!is_finite_value(r->y)||!is_finite_value(r->z)||!is_finite_value(r->yaw) ||
           r->duration_sec<1 || r->duration_sec>120){
          s->message="Expected finite local NED position, yaw in radians, duration 1..120 s";return;
        }
        double x,y,z;
        {std::lock_guard<std::mutex> lk(state_mutex_);x=local_x_;y=local_y_;z=local_z_;}
        if(std::hypot(r->x-x,r->y-y)>20. || std::abs(r->z-z)>10.){
          s->message="Target too far from current position (>20m horizontal or >10m vertical)";return;
        }
        std::lock_guard<std::mutex> lk(sp_mutex_);
        setpoint_.active=true;setpoint_.global=false;setpoint_.velocity=false;
        setpoint_.activated=std::chrono::steady_clock::now();
        setpoint_.x=r->x;setpoint_.y=r->y;setpoint_.z=r->z;setpoint_.yaw=r->yaw;
        setpoint_.expires=std::chrono::steady_clock::now()+
          std::chrono::milliseconds(static_cast<int>(r->duration_sec*1000));
        s->success=true;s->message="Local NED offboard setpoint scheduled at 10 Hz; must explicitly select PX4 OFFBOARD mode";
      });services_.push_back(local);
    auto global=create_service<iface::GotoGlobal>("goto_global",
      [this](std::shared_ptr<iface::GotoGlobal::Request> r,std::shared_ptr<iface::GotoGlobal::Response> s){
        if(manual_takeover_latched_){s->message="RC_MANUAL_TAKEOVER_LATCHED";return;}
        if(!motion_ready(true,s->message))return;
        if(!is_finite_value(r->latitude)||!is_finite_value(r->longitude)||!is_finite_value(r->altitude)||!is_finite_value(r->yaw)||
          std::abs(r->latitude)>90||std::abs(r->longitude)>180||
          r->duration_sec<1||r->duration_sec>120){
          s->message="Invalid global WGS84 lat/lon/AMSL altitude/yaw/duration 1..120s";return;
        }
        double lat,lon,alt;
        {std::lock_guard<std::mutex> lk(state_mutex_);lat=gps_lat_;lon=gps_lon_;alt=gps_alt_;}
        constexpr double rad=0.017453292519943295;
        double dlat=(r->latitude-lat)*rad,dlon=(r->longitude-lon)*rad;
        double a=std::pow(std::sin(dlat/2),2)+std::cos(lat*rad)*std::cos(r->latitude*rad)*std::pow(std::sin(dlon/2),2);
        double distance=12742000.*std::asin(std::sqrt(std::clamp(a,0.,1.)));
        if(distance>20. || std::abs(r->altitude-alt)>10.){
          s->message="Global target outside 20m horizontal / 10m altitude safety envelope";return;
        }
        std::lock_guard<std::mutex> lk(sp_mutex_);
        setpoint_.active=true;setpoint_.global=true;setpoint_.velocity=false;
        setpoint_.activated=std::chrono::steady_clock::now();
        setpoint_.lat=static_cast<int32_t>(std::llround(r->latitude*1.e7));
        setpoint_.lon=static_cast<int32_t>(std::llround(r->longitude*1.e7));
        setpoint_.z=r->altitude;setpoint_.yaw=r->yaw;
        setpoint_.expires=std::chrono::steady_clock::now()+
          std::chrono::milliseconds(static_cast<int>(r->duration_sec*1000));
        s->success=true;s->message="Global AMSL MAVLink setpoint scheduled at 10Hz; PX4 OFFBOARD mode required";
      });services_.push_back(global);
    auto vector=create_service<iface::VelocityVector>("offboard_velocity_vector",
      [this](std::shared_ptr<iface::VelocityVector::Request> r,
             std::shared_ptr<iface::VelocityVector::Response> s){
        if(manual_takeover_latched_){s->message="RC_MANUAL_TAKEOVER_LATCHED";return;}
        if(!enable_motion_||!allow_mode_||!allow_velocity_||!rover_confirmed_){
          s->message="Stock Rover velocity is disabled / not verified";return;
        }
        if(mav_type_!=MAV_TYPE_GROUND_ROVER){s->message="NOT_PX4_ROVER";return;}
        if(!motion_ready(false,s->message))return;
        const uint8_t px4_current_main_mode=static_cast<uint8_t>((custom_mode_.load()>>16)&0xff);
        if(px4_current_main_mode!=4 && px4_current_main_mode!=6){s->message="RC/MANUAL/OTHER_MODE_IN_CONTROL";return;}
        if(!std::isfinite(r->vx_north_mps)||!std::isfinite(r->vy_east_mps)||
           !std::isfinite(r->duration_sec)||r->duration_sec<.1f||
           r->duration_sec>.35f||
           std::hypot(r->vx_north_mps,r->vy_east_mps)>1.5f){
          s->message="Invalid vector; max speed 1.5m/s, lease 0.1..0.35s";return;
        }
        auto now=std::chrono::steady_clock::now();
        std::lock_guard<std::mutex> lk(sp_mutex_);
        // Updating a valid velocity lease should not reset OFFBOARD prestream start.
        const bool continuing=setpoint_.active && setpoint_.velocity &&
           now<setpoint_.expires;
        if(!continuing)setpoint_.activated=now;
        setpoint_.active=true;setpoint_.velocity=true;setpoint_.global=false;
        setpoint_.x=r->vx_north_mps;setpoint_.y=r->vy_east_mps;
        setpoint_.z=0;setpoint_.yaw=0;
        setpoint_.expires=now+
           std::chrono::milliseconds(static_cast<int>(r->duration_sec*1000));
        s->success=true;s->message="Vector lease accepted, local NED velocity-only at 10Hz, OFFBOARD must be explicitly selected";
      });services_.push_back(vector);
    auto clear=create_service<std_srvs::srv::Trigger>("cancel_target",
      [this](std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> s){
        std::lock_guard<std::mutex> lk(sp_mutex_);setpoint_.active=false;
        s->success=true;s->message="Setpoint streaming stopped; PX4 offboard-loss failsafe governs actual vehicle behavior";
      });services_.push_back(clear);
    auto mode=create_service<iface::SetMode>("set_mode",
      [this](std::shared_ptr<iface::SetMode::Request> r,std::shared_ptr<iface::SetMode::Response> s){
        if(manual_takeover_latched_){s->message="RC_MANUAL_TAKEOVER_LATCHED";return;}
        if(!allow_mode_||!enable_motion_){s->message="Mode changes require both allow_mode_changes and enable_motion_commands";return;}
        if(!airframe_valid_){s->message="PX4 airframe unconfigured";return;}
        if(r->main_mode==6){
          if(!local_valid_&&!gps_valid_){s->message="OFFBOARD requires valid navigation state";return;}
          std::lock_guard<std::mutex> lk(sp_mutex_);
          if(!setpoint_.active || std::chrono::steady_clock::now()-setpoint_.activated<1s ||
             std::chrono::steady_clock::now()>=setpoint_.expires){
            s->message="OFFBOARD requires active prestreamed setpoint >=1 second";return;
          }
        }
        uint8_t result=255;
        s->success=send_command(MAV_CMD_DO_SET_MODE,
          {static_cast<float>(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
           static_cast<float>(r->main_mode),static_cast<float>(r->sub_mode),0,0,0,0},result,s->message);
      });services_.push_back(mode);
  }
};
int main(int argc,char **argv){
  rclcpp::init(argc,argv);
  {
    auto node=std::make_shared<PixhawkBridge>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),4);
    executor.add_node(node);executor.spin();
  }
  rclcpp::shutdown();
  return 0;
}
