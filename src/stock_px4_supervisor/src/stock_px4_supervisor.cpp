// ROS 2 integration is strictly advisory. No serial port, MAVLink TX or actuator API.
#include "stock_px4_supervisor/safety.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <pixhawk_interfaces/msg/avoidance_advisory.hpp>
#include <nlohmann/json.hpp>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <cstdlib>
#include <filesystem>
#include <system_error>
#include <cstdio>
#include <memory>
#include <string>
#include <algorithm>
using Clock=std::chrono::steady_clock;
using namespace std::chrono_literals;
using stock_px4::Input;using stock_px4::Config;
using json=nlohmann::json;
class Supervisor:public rclcpp::Node {
public:
 Supervisor():rclcpp::Node("stock_px4_supervisor"),origin_(Clock::now()){
  const bool sim=declare_parameter<bool>("simulation_only",false);
  sim_=sim;
  calib_=declare_parameter<bool>("calibration_verified",false);
  rc_verified_=declare_parameter<bool>("rc_priority_verified",false);
  // Steering servo PWM is not a measured steering angle; physical feedback remains unverified.
  // No stock actuator interface proven. There is intentionally no actuator output.
  Config c;
  c.stop_m=declare_parameter<double>("stop_distance_m",1.15);
  c.slow_m=declare_parameter<double>("slow_distance_m",3.0);
  c.cruise_mps=declare_parameter<double>("cruise_limit_mps",0.75);
  c.avoid_mps=declare_parameter<double>("avoid_limit_mps",0.25);
  c.decel_mps2=declare_parameter<double>("deceleration_mps2",0.8);
  c.reaction_s=declare_parameter<double>("reaction_time_s",0.45);
  c.safety_margin_m=declare_parameter<double>("safety_margin_m",0.30);
  c.kp=declare_parameter<double>("wall_kp",0.6);
  c.ki=declare_parameter<double>("wall_ki",0.0);
  c.kd=declare_parameter<double>("wall_kd",0.0);
  c.wall_target_m=declare_parameter<double>("wall_target_m",0.9);
  c.max_yaw_rps=declare_parameter<double>("yaw_limit_radps",0.25);
  c.differential_wheel_track_m=declare_parameter<double>("differential_wheel_track_m",0.0);
  c.differential_geometry_verified=declare_parameter<bool>("differential_geometry_verified",false);
  c.max_wheel_speed_mps=declare_parameter<double>("max_wheel_speed_mps",0.35);
  c.clear_confirm_frames=declare_parameter<int>("clear_confirm_frames",3);
  diff_geometry_verified_=c.differential_geometry_verified;
  safety_=std::make_unique<stock_px4::Safety>(c);
  const auto side=declare_parameter<std::string>("preferred_wall_side","AUTO");
  safety_->set_preferred_side(side=="LEFT"?1:side=="RIGHT"?-1:0);
  if(!c.valid())throw std::runtime_error("Invalid safety configuration");
  report_=create_publisher<std_msgs::msg::String>("/pixhawk/stock/status",10);
  suggestions_=create_publisher<pixhawk_interfaces::msg::AvoidanceAdvisory>("/pixhawk/stock/advisory",10);
  obstacle_=create_subscription<pixhawk_interfaces::msg::AvoidanceAdvisory>(
    "/pixhawk/avoidance/advisory",10,[this](pixhawk_interfaces::msg::AvoidanceAdvisory::ConstSharedPtr m){
      last_lidar_=Clock::now();scan_seq_=m->source_scan_seq;
      i_.lidar_valid=m->valid && m->source_scan_seq>0;
      i_.perception_stop=m->stop;
      i_.front_m=m->front_clearance_m;i_.left_m=m->left_clearance_m;
      i_.right_m=m->right_clearance_m;
      // TF is validated in the upstream LiDAR perception adapter.
      i_.tf_valid=m->valid;
  });
  status_=create_subscription<std_msgs::msg::String>("/pixhawk/status",10,
    [this](std_msgs::msg::String::ConstSharedPtr m){
      auto j=json::parse(m->data,nullptr,false);
      if(j.is_discarded()||!j.is_object())return;
      last_mav_=Clock::now();i_.mav_connected=j.value("connected",false);
      i_.estimator_valid=j.value("local_valid",false)&&j.value("airframe_configured",false);
      // PX4 main AUTO=4, AUTO_MISSION submode=4 (not AUTO_LOITER=3).
      // These are monitor signals, not permission to actuate hardware.
      const int main=j.value("px4_main_mode",-1);
      const int sub=j.value("px4_sub_mode",-1);
      i_.mission_auto=(main==4 && sub==4);
      rc_telemetry_fresh_=j.value("rc_telemetry_fresh",false);
      manual_mode_selected_=j.value("manual_mode_selected",false);
      px4_main_mode_=main;px4_sub_mode_=sub;
    });
  feedback_=create_subscription<std_msgs::msg::String>("/pixhawk/rover/feedback",10,
    [this](std_msgs::msg::String::ConstSharedPtr m){
      auto j=json::parse(m->data,nullptr,false);
      if(j.is_discarded()||!j.is_object())return;
      if(j.value("type",std::string{})=="VFR_HUD"){
       double speed=j.value("ground_speed_mps",-1.0);
       if(std::isfinite(speed)&&speed>=0)i_.speed_mps=speed;
      }
      // PWM and RC_CHANNELS do not prove steering angle or manual takeover.
    });
  imu_=create_subscription<sensor_msgs::msg::Imu>("/pixhawk/imu",rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::Imu::ConstSharedPtr m){(void)m;last_imu_=Clock::now();i_.imu_valid=true;});
  if(sim_){
   sim_input_=create_subscription<std_msgs::msg::String>("/pixhawk/stock/sim_input",10,
     [this](std_msgs::msg::String::ConstSharedPtr m){
       auto j=json::parse(m->data,nullptr,false);
       if(j.is_discarded()||!j.is_object())return;
       // SIMULATION_ONLY: never controls serial or motor.
       i_.rc_known=j.value("rc_known",true);
       i_.manual_rc=j.value("manual_rc",false);
       i_.estop=j.value("estop",false);
       i_.mission_auto=j.value("mission_auto",true);
       i_.mav_connected=j.value("mav_connected",true);
       i_.estimator_valid=j.value("estimator_valid",true);
       i_.imu_valid=j.value("imu_valid",true);
       i_.steering_feedback_valid=j.value("steering_feedback_valid",true);
       const bool synthetic_scan=j.value("simulate_synthetic_scan",true);
       if(synthetic_scan){
         // Unit fixtures synthesize distinct LiDAR samples without a perception node.
         i_.tf_valid=j.value("tf_valid",true);
         i_.lidar_valid=j.value("lidar_valid",true);
         i_.front_m=j.value("front_m",4.0);
         i_.left_m=j.value("left_m",2.0);i_.right_m=j.value("right_m",2.0);
         last_lidar_=Clock::now();++scan_seq_;
       }
       i_.calibration_verified=j.value("calibration_verified",true);
       i_.speed_mps=j.value("speed_mps",0.0);
       last_mav_=last_imu_=Clock::now();
       // Simulated operator reset must be a rising edge, not a held heartbeat.
       const bool ack=j.value("operator_ack",false);
       if(ack&&!last_sim_ack_)safety_->acknowledge_operator();
       last_sim_ack_=ack;
     });
  }
  tick_=create_wall_timer(100ms,[this]{tick();});
  RCLCPP_WARN(get_logger(),
   "PX4 STOCK ADVISORY ONLY: hardware TX=DISABLED, rc priority unverified; simulation=%s",
   sim_?"true":"false");
 }
private:
 static uint64_t age_ms(Clock::time_point t){
   if(t==Clock::time_point{})return UINT64_MAX;
   return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
     Clock::now()-t).count());
 }
 void tick(){
  Input current=i_;
  current.scan_seq=scan_seq_;
  current.lidar_age_ms=age_ms(last_lidar_);
  current.mav_age_ms=age_ms(last_mav_);
  current.imu_age_ms=age_ms(last_imu_);
  if(!sim_){
    current.calibration_verified=calib_;
    // RC is verified only after real receiver and mode-switch acceptance tests.
    current.rc_known=rc_verified_ && rc_telemetry_fresh_ && current.mav_connected;
    current.manual_rc=manual_mode_selected_;
    // Independent encoder or calibrated steering feedback is not installed here.
    current.steering_feedback_valid=false;
  }
  const auto ms=static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
       Clock::now()-origin_).count());
  const auto d=safety_->step(current,ms);
  pixhawk_interfaces::msg::AvoidanceAdvisory adv;
  adv.valid=!d.stop&&!d.hazard_latched;
  adv.obstacle=d.state!=stock_px4::State::CLEAR;
  adv.stop=d.stop;
  adv.vx_limit_mps=static_cast<float>(d.vx_limit_mps);
  adv.yaw_correction_radps=static_cast<float>(d.yaw_correction_rps);
  adv.front_clearance_m=static_cast<float>(std::max(0.,current.front_m));
  adv.left_clearance_m=static_cast<float>(std::max(0.,current.left_m));
  adv.right_clearance_m=static_cast<float>(std::max(0.,current.right_m));
  adv.source_scan_seq=current.scan_seq;
  suggestions_->publish(adv);
  json j={{"state",stock_px4::state_name(d.state)},{"reason",d.reason},
    {"hardware_command_enabled",false},{"stock_firmware_interface","UNPROVEN"},
    {"simulation_only",sim_},{"stop",d.stop},{"hazard_latched",d.hazard_latched},
    {"vx_limit_mps",d.vx_limit_mps},{"yaw_correction_radps",d.yaw_correction_rps},
    {"dynamic_stop_m",d.stop_distance_m},{"clear_frames",d.clear_frames},
    {"side",d.side},{"front_m",current.front_m},{"left_m",current.left_m},
    {"right_m",current.right_m},{"speed_mps",current.speed_mps},
    {"lidar_age_ms",current.lidar_age_ms},{"mav_age_ms",current.mav_age_ms},
    {"imu_age_ms",current.imu_age_ms},{"rc_takeover_verified",current.rc_known},
    {"rc_telemetry_fresh",rc_telemetry_fresh_},
    {"px4_manual_mode_selected",manual_mode_selected_},
    {"px4_main_mode",px4_main_mode_},{"px4_sub_mode",px4_sub_mode_},
    {"physical_steering_feedback_verified",false},
    {"drive_model","DIFFERENTIAL_SKID_STEER"},
    {"differential_geometry_verified",diff_geometry_verified_}};
  std_msgs::msg::String out;out.data=j.dump();report_->publish(out);
  // Read-only local snapshot for the separate loopback Foxglove tuning panel.
  const auto wall_ms=std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
  j["observed_at_unix_ms"]=wall_ms;
  // Resolve snapshot storage per user; no machine-specific absolute path.
  static const std::filesystem::path snapshot_dir = [] {
    const char* xdg = std::getenv("XDG_CACHE_HOME");
    const char* home = std::getenv("HOME");
    std::filesystem::path base = (xdg && *xdg) ? std::filesystem::path(xdg) :
      ((home && *home) ? std::filesystem::path(home) / ".cache" : std::filesystem::path("/tmp"));
    const auto dir = base / "mid360_pixhawk_ros";
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    return ec ? std::filesystem::path{} : dir;
  }();
  if (!snapshot_dir.empty()) {
    const auto snapshot = (snapshot_dir / "stock_px4_latest.json").string();
    const auto temp = (snapshot_dir / "stock_px4_latest.json.tmp").string();
    { std::ofstream file(temp, std::ios::trunc); if (file) file << j.dump(); }
    std::rename(temp.c_str(), snapshot.c_str());
  }
  if(d.state!=last_){RCLCPP_WARN(get_logger(),"state=%s reason=%s",stock_px4::state_name(d.state),d.reason.c_str());last_=d.state;}
 }
 bool sim_=false,calib_=false,rc_verified_=false,diff_geometry_verified_=false;
 bool rc_telemetry_fresh_=false,manual_mode_selected_=false;
 int px4_main_mode_=-1,px4_sub_mode_=-1;
 bool last_sim_ack_=false;
 Input i_{};uint64_t scan_seq_=0;Clock::time_point origin_,last_lidar_{},last_mav_{},last_imu_{};
 stock_px4::State last_=stock_px4::State::UNKNOWN;
 std::unique_ptr<stock_px4::Safety> safety_;
 rclcpp::Publisher<std_msgs::msg::String>::SharedPtr report_;
 rclcpp::Publisher<pixhawk_interfaces::msg::AvoidanceAdvisory>::SharedPtr suggestions_;
 rclcpp::Subscription<pixhawk_interfaces::msg::AvoidanceAdvisory>::SharedPtr obstacle_;
 rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_,feedback_,sim_input_;
 rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_;
 rclcpp::TimerBase::SharedPtr tick_;
};
int main(int argc,char**argv){
 rclcpp::init(argc,argv);
 try{rclcpp::spin(std::make_shared<Supervisor>());}
 catch(const std::exception& e){RCLCPP_FATAL(rclcpp::get_logger("stock_px4_supervisor"),"%s",e.what());rclcpp::shutdown();return 1;}
 rclcpp::shutdown();return 0;
}
