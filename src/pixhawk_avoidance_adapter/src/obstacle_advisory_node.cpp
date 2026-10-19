// MID-360S fail-closed perception. Never transmits MAVLink or controls actuators.
// A sector observation is not a verified collision-free trajectory.
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/string.hpp>
#include <pixhawk_interfaces/msg/avoidance_advisory.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <nlohmann/json.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <cstdint>
#include <memory>
#include <string>
#include <stdexcept>

using Clock=std::chrono::steady_clock;
using Advisory=pixhawk_interfaces::msg::AvoidanceAdvisory;

class ObstacleAdvisory final: public rclcpp::Node {
public:
  ObstacleAdvisory():Node("obstacle_advisory_node") {
    const auto input=declare_parameter<std::string>("cloud_topic","/livox/lidar");
    const auto output=declare_parameter<std::string>("output_topic","/pixhawk/avoidance/advisory");
    base_=declare_parameter<std::string>("base_frame","base_link");
    calibrated_=declare_parameter<bool>("calibration_verified",false);
    sim_=declare_parameter<bool>("simulation_only_override",false);
    min_z_=declare_parameter<double>("obstacle_min_z_m",.15);
    max_z_=declare_parameter<double>("obstacle_max_z_m",1.75);
    max_range_=declare_parameter<double>("max_scan_range_m",8.);
    half_width_=declare_parameter<double>("robot_width_m",.65)/2.;
    margin_=declare_parameter<double>("extra_clearance_m",.35);
    slow_=declare_parameter<double>("front_slow_distance_m",3.);
    stop_=declare_parameter<double>("front_stop_distance_m",1.15);
    side_open_=declare_parameter<double>("side_open_distance_m",1.2);
    min_front_=declare_parameter<int>("front_min_points",4);
    min_front_observed_=declare_parameter<int>("min_front_observed_points",12);
    min_side_observed_=declare_parameter<int>("min_side_observed_points",6);
    min_scan_=declare_parameter<int>("min_valid_scan_points",150);
    max_points_=declare_parameter<int>("max_processed_points",12000);
    max_yaw_=declare_parameter<double>("max_yaw_correction_rps",.25);
    wall_=declare_parameter<std::string>("wall_mode","AUTO");
    wall_target_=declare_parameter<double>("wall_target_distance_m",.9);
    kp_=declare_parameter<double>("wall_kp",.6);
    hysteresis_=declare_parameter<double>("side_hysteresis_m",.35);
    timeout_=declare_parameter<double>("cloud_timeout_s",.30);
    cruise_=declare_parameter<double>("cruise_limit_mps",.75);
    const auto hz=declare_parameter<double>("advisory_rate_hz",10.);
    if(!(std::isfinite(min_z_)&&std::isfinite(max_z_)&&std::isfinite(max_range_)&&
         std::isfinite(half_width_)&&std::isfinite(margin_)&&std::isfinite(slow_)&&
         std::isfinite(stop_)&&std::isfinite(side_open_)&&std::isfinite(max_yaw_)&&
         std::isfinite(wall_target_)&&std::isfinite(kp_)&&std::isfinite(hysteresis_)&&
         std::isfinite(timeout_)&&std::isfinite(cruise_)&&std::isfinite(hz) &&
         min_z_<max_z_&&min_z_>=0&&max_range_>slow_&&slow_>stop_&&stop_>0&&
         half_width_>0&&margin_>0&&side_open_>0&&max_yaw_>0&&wall_target_>0&&
         kp_>=0&&hysteresis_>=0&&timeout_>=.1&&timeout_<=5.&&cruise_>0&&
         hz>=2&&hz<=50&&min_front_>=1&&min_front_observed_>=min_front_&&
         min_side_observed_>=1&&min_scan_>=10&&max_points_>=100&&
         base_.size()>0&&(wall_=="AUTO"||wall_=="LEFT"||wall_=="RIGHT")))
      throw std::runtime_error("Invalid obstacle perception configuration");
    tf_buffer_=std::make_shared<tf2_ros::Buffer>(get_clock());
    listener_=std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    pub_=create_publisher<Advisory>(output,10);
    status_=create_publisher<std_msgs::msg::String>("/pixhawk/avoidance/status",10);
    diagnostics_=create_publisher<std_msgs::msg::String>("/pixhawk/avoidance/diagnostics",10);
    sub_=create_subscription<sensor_msgs::msg::PointCloud2>(
      input,rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m){scan(*m);});
    timer_=create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(1./hz)),[this]{publish();});
    reject("STARTUP_NO_CLOUD");
    RCLCPP_WARN(get_logger(),"Perception is advisory-only; hardware command TX disabled. Calibrated=%s sim=%s",
      calibrated_?"true":"false",sim_?"true":"false");
  }
private:
  struct Sector {double distance=100.;uint32_t count=0;};
  void reject(const char* reason){
    status_reason_=reason;
    latest_=Advisory{};
    latest_.valid=false;latest_.stop=true;latest_.obstacle=true;
    latest_.source_scan_seq=0;
    last_cloud_=Clock::time_point{};
    last_reject_=Clock::now();
    ++rejected_;
    good_points_=front_count_=left_count_=right_count_=0;
  }
  void scan(const sensor_msgs::msg::PointCloud2& m){
    const auto find=[&](const char* name)->int{
      const auto it=std::find_if(m.fields.begin(),m.fields.end(),[name](const auto& f){
        return f.name==name && f.datatype==sensor_msgs::msg::PointField::FLOAT32 && f.count==1;
      });
      return it==m.fields.end()?-1:static_cast<int>(it->offset);
    };
    const int ix=find("x"),iy=find("y"),iz=find("z");
    if(ix<0||iy<0||iz<0||m.height==0||m.width==0||m.point_step<12||
       m.is_bigendian||m.header.frame_id.empty()||
       ix+4>m.point_step||iy+4>m.point_step||iz+4>m.point_step||
       m.row_step<static_cast<uint64_t>(m.width)*m.point_step||
       static_cast<uint64_t>(m.row_step)*m.height>m.data.size()){
       reject("MALFORMED_CLOUD");return;
    }
    const auto stamp=rclcpp::Time(m.header.stamp);
    const int64_t stamp_ns=stamp.nanoseconds();
    if(stamp_ns<=0){reject("CLOUD_NO_STAMP");return;}
    if(stamp_ns<=last_stamp_ns_){reject("CLOUD_NONMONOTONIC");return;}
    const auto scan_age=(get_clock()->now()-stamp).seconds();
    if(!sim_&&(!std::isfinite(scan_age)||scan_age>timeout_||scan_age<-.1)){
      reject("CLOUD_STALE_TIMESTAMP");return;
    }
    // No one can set simulation_only_override=true on production launches.
    if(!calibrated_&&!sim_){reject("EXTRINSICS_UNCALIBRATED");return;}
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf=tf_buffer_->lookupTransform(base_,m.header.frame_id,stamp);
    }catch(const std::exception& e) {
      RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),5000,
        "TF unavailable; advisory STOP: %s",e.what());
      reject("TF_UNAVAILABLE");return;
    }
    const double tf_age=(get_clock()->now()-rclcpp::Time(tf.header.stamp)).seconds();
    if(!sim_&&tf.header.stamp.sec!=0&&(!std::isfinite(tf_age)||tf_age>.25||tf_age<-.1)){
      reject("TF_STALE");return;
    }
    const auto& q=tf.transform.rotation;
    const double w=q.w,x=q.x,y=q.y,z=q.z;
    const double norm=w*w+x*x+y*y+z*z;
    const auto& tr=tf.transform.translation;
    if(!std::isfinite(norm)||std::abs(norm-1.)>.05||
       !std::isfinite(tr.x)||!std::isfinite(tr.y)||!std::isfinite(tr.z)){
      reject("TF_INVALID_GEOMETRY");return;
    }
    const double r00=1-2*(y*y+z*z),r01=2*(x*y-z*w),r02=2*(x*z+y*w);
    const double r10=2*(x*y+z*w),r11=1-2*(x*x+z*z),r12=2*(y*z-x*w);
    const double r20=2*(x*z-y*w),r21=2*(y*z+x*w),r22=1-2*(x*x+y*y);
    Sector front,left,right;
    uint32_t points=0;
    const size_t total=static_cast<size_t>(m.width)*m.height;
    const size_t stride=std::max<size_t>(1,(total+max_points_-1)/max_points_);
    for(size_t k=0;k<total;k+=stride){
      const size_t j=(k/m.width)*static_cast<size_t>(m.row_step)+
                     (k%m.width)*static_cast<size_t>(m.point_step);
      if(j+static_cast<size_t>(m.point_step)>m.data.size()){reject("CLOUD_DATA_OVERRUN");return;}
      const auto val=[&](int off)->float{
        float v;std::memcpy(&v,m.data.data()+j+off,sizeof(v));return v;
      };
      const float X=val(ix),Y=val(iy),Z=val(iz);
      if(!std::isfinite(X)||!std::isfinite(Y)||!std::isfinite(Z))continue;
      const double px=r00*X+r01*Y+r02*Z+tr.x;
      const double py=r10*X+r11*Y+r12*Z+tr.y;
      const double pz=r20*X+r21*Y+r22*Z+tr.z;
      // Height above calibrated base_link ground reference is a coarse ground filter.
      if(!std::isfinite(px)||!std::isfinite(py)||!std::isfinite(pz)||
         pz<min_z_||pz>max_z_||px<=0||px>max_range_)continue;
      ++points;
      if(std::abs(py)<=half_width_+margin_){
        ++front.count;front.distance=std::min(front.distance,px);
      } else if(py>half_width_+margin_&&py<half_width_+margin_+2.5&&px<slow_){
        ++left.count;left.distance=std::min(left.distance,py-half_width_);
      } else if(py<-(half_width_+margin_)&&py>-(half_width_+margin_+2.5)&&px<slow_){
        ++right.count;right.distance=std::min(right.distance,-py-half_width_);
      }
    }
    good_points_=points;front_count_=front.count;left_count_=left.count;right_count_=right.count;
    Advisory out{};
    out.front_clearance_m=front.distance;
    out.left_clearance_m=left.distance;
    out.right_clearance_m=right.distance;
    out.source_scan_seq=++scan_seq_;
    // Do not treat missing front rays / sparse cloud as a clear corridor.
    if(points<static_cast<uint32_t>(min_scan_)||
       front.count<static_cast<uint32_t>(min_front_observed_)){
      out.valid=false;out.obstacle=true;out.stop=true;
      status_reason_=points<static_cast<uint32_t>(min_scan_)
        ?"INSUFFICIENT_POINTS":"FRONT_COVERAGE_UNKNOWN";
    }else{
      out.valid=true;
      out.obstacle=(front.count>=static_cast<uint32_t>(min_front_)&&front.distance<slow_);
      // No trajectory/curvature proof exists; perception MUST NOT authorize movement around obstacles.
      out.stop=out.obstacle;
      out.vx_limit_mps=out.obstacle?0.f:static_cast<float>(cruise_);
      if(!out.obstacle)status_reason_="CLEAR_ADVISORY";
      else if(front.distance<=stop_)status_reason_="FRONT_STOP";
      else{
        const double l=left.count>=static_cast<uint32_t>(min_side_observed_)?left.distance:-1.;
        const double r=right.count>=static_cast<uint32_t>(min_side_observed_)?right.distance:-1.;
        int wanted=wall_=="LEFT"?1:wall_=="RIGHT"?-1:l>=r?1:-1;
        if(side_&&wanted!=side_&&l>=0&&r>=0&&std::abs(l-r)<hysteresis_)wanted=side_;
        side_=wanted;
        const double clearance=wanted>0?l:r;
        if(clearance<side_open_)status_reason_="BYPASS_CLEARANCE_UNKNOWN";
        else{
          const double error=std::clamp(clearance-wall_target_,-.4,.4);
          out.yaw_correction_radps=static_cast<float>(
            std::clamp(wanted*(kp_*error+.12),-max_yaw_,max_yaw_));
          status_reason_="BYPASS_SUGGESTION_ONLY";
        }
      }
    }
    latest_=out;
    last_stamp_ns_=stamp_ns;
    last_cloud_=Clock::now();
  }
  void publish(){
    Advisory msg=latest_;
    const bool stale=last_cloud_==Clock::time_point{} ||
      std::chrono::duration<double>(Clock::now()-last_cloud_).count()>timeout_;
    const bool fresh_reject=last_reject_!=Clock::time_point{} &&
      std::chrono::duration<double>(Clock::now()-last_reject_).count()<timeout_;
    const std::string reason=stale?(fresh_reject?status_reason_:"CLOUD_STALE"):status_reason_;
    if(stale){
      msg.valid=false;msg.obstacle=true;msg.stop=true;
      msg.vx_limit_mps=0;msg.yaw_correction_radps=0;msg.source_scan_seq=0;
    }
    pub_->publish(msg);
    std_msgs::msg::String status;status.data=reason;status_->publish(status);
    const nlohmann::json data={
      {"state",reason},{"valid",msg.valid},{"stop",msg.stop},
      {"advisory_only",true},{"hardware_command_enabled",false},
      {"calibration_verified",calibrated_},{"simulation_only",sim_},
      {"front_clearance_m",msg.front_clearance_m},
      {"left_clearance_m",msg.left_clearance_m},{"right_clearance_m",msg.right_clearance_m},
      {"vx_limit_mps",msg.vx_limit_mps},{"yaw_suggestion_radps",msg.yaw_correction_radps},
      {"source_scan_seq",msg.source_scan_seq},
      {"front_observed_points",front_count_},{"left_observed_points",left_count_},
      {"right_observed_points",right_count_},{"valid_points",good_points_},
      {"invalid_scan_count",rejected_}};
    std_msgs::msg::String diag;diag.data=data.dump();diagnostics_->publish(diag);
  }
  std::string base_,wall_,status_reason_="STARTUP_NO_CLOUD";
  bool calibrated_=false,sim_=false;
  double min_z_{},max_z_{},max_range_{},half_width_{},margin_{},slow_{},stop_{},
    side_open_{},max_yaw_{},wall_target_{},kp_{},hysteresis_{},timeout_{},cruise_{};
  int min_front_{},min_front_observed_{},min_side_observed_{},min_scan_{},max_points_{},side_=0;
  uint32_t good_points_=0,front_count_=0,left_count_=0,right_count_=0;
  uint64_t rejected_=0,scan_seq_=0;int64_t last_stamp_ns_=0;
  Clock::time_point last_cloud_{},last_reject_{};
  Advisory latest_;
  rclcpp::Publisher<Advisory>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_,diagnostics_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
};
int main(int argc,char**argv){
  rclcpp::init(argc,argv);
  try{rclcpp::spin(std::make_shared<ObstacleAdvisory>());}
  catch(const std::exception& e){
    RCLCPP_FATAL(rclcpp::get_logger("obstacle_advisory_node"),"%s",e.what());
    rclcpp::shutdown();return 1;
  }
  rclcpp::shutdown();return 0;
}
