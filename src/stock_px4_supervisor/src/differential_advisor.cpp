// OBSERVATION-ONLY ROS2 differential kinematics; never sends motor, MAVLink or PWM commands.
#include "stock_px4_supervisor/differential.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <pixhawk_interfaces/msg/avoidance_advisory.hpp>
#include <nlohmann/json.hpp>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

using Clock=std::chrono::steady_clock;
using namespace std::chrono_literals;

class DifferentialAdvisor final:public rclcpp::Node{
public:
 DifferentialAdvisor():Node("differential_advisor"){
  sim_=declare_parameter<bool>("simulation_only",false);
  geometry_verified_=declare_parameter<bool>("wheel_track_verified",false);
  track_=declare_parameter<double>("wheel_track_m",0.0);
  wheel_limit_=declare_parameter<double>("wheel_speed_limit_mps",0.35);
  const double max_age=declare_parameter<double>("advisory_timeout_s",0.30);
  if (!std::isfinite(max_age)||max_age<.1||max_age>2||
      !std::isfinite(wheel_limit_)||wheel_limit_<=0||wheel_limit_>5||
      !std::isfinite(track_)||track_<0||track_>5||
      (geometry_verified_ && track_<=0))
   throw std::runtime_error("Invalid differential advisory configuration");
  max_age_=std::chrono::duration<double>(max_age);
  publisher_=create_publisher<std_msgs::msg::String>("/pixhawk/differential/diagnostics",10);
  advisory_=create_subscription<pixhawk_interfaces::msg::AvoidanceAdvisory>(
   "/pixhawk/stock/advisory",10,
   [this](pixhawk_interfaces::msg::AvoidanceAdvisory::ConstSharedPtr msg){
    last_advisory_=*msg;last_advisory_at_=Clock::now();
   });
  status_=create_subscription<std_msgs::msg::String>(
   "/pixhawk/stock/status",10,[this](std_msgs::msg::String::ConstSharedPtr msg){
    auto parsed=nlohmann::json::parse(msg->data,nullptr,false);
    if(parsed.is_discarded()||!parsed.is_object())return;
    // Only safety-clear states are allowed even for non-actuating preview.
    clear_=(parsed.value("state",std::string{})=="CLEAR" &&
            !parsed.value("stop",true) && !parsed.value("hazard_latched",true));
    last_status_at_=Clock::now();
   });
  timer_=create_wall_timer(100ms,[this]{tick();});
  RCLCPP_WARN(get_logger(),
    "DIFFERENTIAL PREVIEW ONLY: actual Cube output locked, sim=%s, track verified=%s",
    sim_?"true":"false",geometry_verified_?"true":"false");
 }
private:
 void tick(){
  const auto now=Clock::now();
  const bool current=last_status_at_!=Clock::time_point{} &&
    last_advisory_at_!=Clock::time_point{} &&
    (now-last_status_at_)<max_age_&&(now-last_advisory_at_)<max_age_;
  const bool allowed=current && clear_ && last_advisory_.valid &&
    !last_advisory_.stop && last_advisory_.source_scan_seq>0;
  stock_px4::DifferentialInput in;
  in.motion_advisory_valid=allowed;
  in.geometry_verified=geometry_verified_ && sim_; // never preview motor movement in production
  in.speed_mps=last_advisory_.vx_limit_mps;
  in.yaw_radps=last_advisory_.yaw_correction_radps;
  in.wheel_track_m=track_;
  in.wheel_speed_limit_mps=wheel_limit_;
  const auto d=stock_px4::preview_wheels(in);
  const nlohmann::json j={
   {"kind","DIFFERENTIAL_VELOCITY_PREVIEW"},{"controller_type","DIFFERENTIAL_SKID_STEER"},
   {"wheel_track_m",track_},{"wheel_track_verified",geometry_verified_},
   {"left_wheel_mps",d.left_mps},{"right_wheel_mps",d.right_mps},
   {"preview_valid",d.valid},{"reason",current?d.reason:"SOURCE_TIMEOUT"},
   {"stock_px4_interface_supported",false},{"hardware_command_enabled",false},
   {"ros_motion_publisher_enabled",false},{"simulation_only",sim_},
   {"requires_px4_rover_firmware_proof",true}};
  std_msgs::msg::String msg;msg.data=j.dump();publisher_->publish(msg);
 }
 bool sim_=false,geometry_verified_=false,clear_=false;
 double track_=0,wheel_limit_=0.35;
 std::chrono::duration<double> max_age_{.30};
 Clock::time_point last_status_at_{},last_advisory_at_{};
 pixhawk_interfaces::msg::AvoidanceAdvisory last_advisory_;
 rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
 rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_;
 rclcpp::Subscription<pixhawk_interfaces::msg::AvoidanceAdvisory>::SharedPtr advisory_;
 rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char**argv){
 rclcpp::init(argc,argv);
 try {rclcpp::spin(std::make_shared<DifferentialAdvisor>());}
 catch(const std::exception& e){
  RCLCPP_FATAL(rclcpp::get_logger("differential_advisor"),"%s",e.what());
  rclcpp::shutdown();return 1;
 }
 rclcpp::shutdown();return 0;
}
