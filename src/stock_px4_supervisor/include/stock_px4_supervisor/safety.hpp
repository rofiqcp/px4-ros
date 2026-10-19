#pragma once
// Stock PX4 safety advisory only. No MAVLink transmit and no actuator authority.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>

namespace stock_px4 {
enum class State {ESTOP, MANUAL_RC, UNKNOWN, LATCHED, CLEAR, SLOW, STOP, AVOID_PROPOSED};
inline const char* state_name(State s) {
 switch(s){case State::ESTOP:return "ESTOP";case State::MANUAL_RC:return "MANUAL_RC";
 case State::UNKNOWN:return "UNKNOWN";case State::LATCHED:return "LATCHED";
 case State::CLEAR:return "CLEAR";case State::SLOW:return "SLOW";
 case State::STOP:return "STOP";case State::AVOID_PROPOSED:return "AVOID_PROPOSED";}
 return "UNKNOWN";
}
struct Config {
 double stop_m=1.15, slow_m=3.0, cruise_mps=0.75, avoid_mps=0.25;
 double reaction_s=0.45, decel_mps2=0.8, safety_margin_m=0.3;
 double width_m=0.65, side_clearance_m=1.2, hysteresis_m=0.35;
 double wall_target_m=0.9, kp=0.6, ki=0, kd=0, max_yaw_rps=0.25;
 double differential_wheel_track_m=0.0, max_wheel_speed_mps=0.35;
 double max_plausible_speed_mps=8.0;
 bool differential_geometry_verified=false;
 unsigned clear_confirm_frames=3;
 uint64_t advisory_timeout_ms=300, mav_timeout_ms=1500, imu_timeout_ms=400;
 bool valid()const{
  return std::isfinite(stop_m)&&stop_m>0&&std::isfinite(slow_m)&&slow_m>stop_m
   &&std::isfinite(cruise_mps)&&cruise_mps>0&&std::isfinite(avoid_mps)&&avoid_mps>0
   &&std::isfinite(reaction_s)&&reaction_s>=0&&std::isfinite(decel_mps2)&&decel_mps2>0
   &&std::isfinite(safety_margin_m)&&safety_margin_m>=0
   &&std::isfinite(width_m)&&width_m>0&&std::isfinite(side_clearance_m)&&side_clearance_m>0
   &&std::isfinite(hysteresis_m)&&hysteresis_m>=0&&std::isfinite(wall_target_m)&&wall_target_m>0
   &&std::isfinite(kp)&&std::isfinite(ki)&&std::isfinite(kd)
   &&std::isfinite(max_yaw_rps)&&max_yaw_rps>0
   &&std::isfinite(differential_wheel_track_m)&&differential_wheel_track_m>=0
   &&(!differential_geometry_verified || differential_wheel_track_m>0)
   &&std::isfinite(max_wheel_speed_mps)&&max_wheel_speed_mps>0&&max_wheel_speed_mps<=5
   &&std::isfinite(max_plausible_speed_mps)&&max_plausible_speed_mps>=max_wheel_speed_mps&&max_plausible_speed_mps<=30
   &&clear_confirm_frames>=3&&clear_confirm_frames<=100
   &&advisory_timeout_ms>=100&&mav_timeout_ms>=300&&imu_timeout_ms>=100;
 }
};
struct Input {
 bool estop=false, manual_rc=false, rc_known=false;
 bool lidar_valid=false, tf_valid=false, calibration_verified=false;
 bool perception_stop=false;
 bool mav_connected=false, estimator_valid=false, imu_valid=false, steering_feedback_valid=false;
 bool mission_auto=false;
 double front_m=0,left_m=0,right_m=0, speed_mps=0;
 uint64_t lidar_age_ms=UINT64_MAX,mav_age_ms=UINT64_MAX,imu_age_ms=UINT64_MAX;
 uint64_t scan_seq=0; // Nonzero in ROS runtime; distinct scans are required for clearance.
};
struct Decision {
 State state=State::UNKNOWN;
 double vx_limit_mps=0, yaw_correction_rps=0, stop_distance_m=0;
 bool stop=true, hazard_latched=true, control_supported=false;
 int side=0, clear_frames=0;
 std::string reason="INIT";
};
class Safety {
public:
 explicit Safety(const Config& c=Config{}): cfg_(c) {}
 void acknowledge_operator(){operator_approved_=true;}
 void set_preferred_side(int s){ preferred_= s>0?1:s<0?-1:0; }
 Decision step(const Input& i, uint64_t now_ms){
  Decision o;
  o.stop_distance_m=cfg_.stop_m;
  if(!cfg_.valid()){latched_=true;return stop(o,State::UNKNOWN,"BAD_CONFIG");}
  if(i.estop){latched_=true;clear_frames_=0;operator_approved_=false;
    return stop(o,State::ESTOP,"E_STOP_ASSERTED");}
  // Manual RC can be recognized; this is informational, never a companion actuation.
  if(i.rc_known && i.manual_rc){
    latched_=true;operator_approved_=false;clear_frames_=0;
    return stop(o,State::MANUAL_RC,"RC_HAS_PRIORITY");
  }
  if(!i.rc_known){latched_=true;clear_frames_=0;operator_approved_=false;
    return stop(o,State::UNKNOWN,"OPERATOR_AUTHORITY_UNVERIFIED");}
  const bool finite=std::isfinite(i.front_m)&&std::isfinite(i.left_m)
    &&std::isfinite(i.right_m)&&std::isfinite(i.speed_mps)
    &&i.front_m>=0&&i.left_m>=0&&i.right_m>=0&&i.speed_mps>=0;
  if(finite && i.speed_mps>cfg_.max_plausible_speed_mps){
    latched_=true;clear_frames_=0;operator_approved_=false;
    return stop(o,State::UNKNOWN,"IMPLAUSIBLE_SPEED_TELEMETRY");
  }
  const bool fresh=i.lidar_valid&&i.tf_valid&&i.calibration_verified&&i.mav_connected
    &&i.estimator_valid&&i.imu_valid&&i.steering_feedback_valid&&i.mission_auto
    &&i.lidar_age_ms<=cfg_.advisory_timeout_ms
    &&i.mav_age_ms<=cfg_.mav_timeout_ms&&i.imu_age_ms<=cfg_.imu_timeout_ms;
  if(!finite||!fresh){latched_=true;clear_frames_=0;operator_approved_=false;
    return stop(o,State::UNKNOWN,!finite?"INVALID_NUMERIC_OR_DISTANCE":"STALE_OR_UNVERIFIED_INPUT");}
  if(i.perception_stop){
    latched_=true;clear_frames_=0;operator_approved_=false;
    return stop(o,State::STOP,"PERCEPTION_STOP_ADVISORY");
  }
  const double v=i.speed_mps;
  const double braking=v*v/(2*cfg_.decel_mps2)+v*cfg_.reaction_s+cfg_.safety_margin_m;
  o.stop_distance_m=std::max(cfg_.stop_m,braking);
  if(i.front_m<=o.stop_distance_m){
    latched_=true;clear_frames_=0;operator_approved_=false;
    return stop(o,State::STOP,"STOPPING_DISTANCE");
  }
  if(i.front_m<cfg_.slow_m){
    latched_=true;clear_frames_=0;operator_approved_=false;
    const double l=i.left_m,r=i.right_m;
    int wanted=preferred_?preferred_:(l>=r?1:-1);
    if(side_ && wanted!=side_ && std::abs(l-r)<cfg_.hysteresis_m)wanted=side_;
    side_=wanted;
    if((wanted>0?l:r)<cfg_.side_clearance_m)
      return stop(o,State::STOP,"NO_VERIFIED_BYPASS_CLEARANCE");
    const double err=std::clamp((wanted>0?l:r)-cfg_.wall_target_m,-1.,1.);
    if(!cfg_.differential_geometry_verified){
      o.side=wanted;
      return stop(o,State::AVOID_PROPOSED,"DIFFERENTIAL_GEOMETRY_UNVERIFIED");
    }
    // Differential rover can turn in place (no Ackermann minimum-turn radius).
    // This yaw suggestion is capped by track width and a per-wheel speed ceiling.
    const double max_diff_yaw=2.*cfg_.max_wheel_speed_mps/cfg_.differential_wheel_track_m;
    const double max_geo_yaw=std::min(cfg_.max_yaw_rps,max_diff_yaw);
    o.yaw_correction_rps=std::clamp(wanted*cfg_.kp*err,-max_geo_yaw,max_geo_yaw);
    o.side=wanted;
    // AVOID_PROPOSED cannot move until trajectory collision and stock firmware
    // support are proven. A yaw suggestion is NOT a steering command.
    return stop(o,State::AVOID_PROPOSED,"PATH_AND_PX4_INTERFACE_UNVERIFIED");
  }
  if(i.front_m<cfg_.slow_m+cfg_.hysteresis_m){
    clear_frames_=0;
  }else if((i.scan_seq!=0 && i.scan_seq!=last_clear_seq_ && i.scan_seq>last_clear_seq_) ||
            (i.scan_seq==0 && last_clear_ms_!=now_ms)) {
    ++clear_frames_;last_clear_ms_=now_ms;last_clear_seq_=i.scan_seq;
  }
  if(latched_){
    if(clear_frames_>=static_cast<int>(cfg_.clear_confirm_frames) && operator_approved_){
       latched_=false;operator_approved_=false;
    } else return stop(o,State::LATCHED, "WAIT_CLEAR_SCANS_AND_OPERATOR_ACK");
  }
  o.state=State::CLEAR;o.reason="CLEAR_ADVISORY_ONLY";
  o.vx_limit_mps=cfg_.cruise_mps;o.stop=false;o.hazard_latched=false;
  o.clear_frames=clear_frames_;
  return o;
 }
 bool latched() const{return latched_;}
 const Config& config()const{return cfg_;}
private:
 Decision stop(Decision o,State s,const char*why){
  o.state=s;o.reason=why;o.stop=true;o.vx_limit_mps=0;
  // Preserve yaw ONLY as planning suggestion (still zero when safety critical).
  if(s!=State::AVOID_PROPOSED)o.yaw_correction_rps=0;
  o.hazard_latched=latched_;o.clear_frames=clear_frames_;return o;
 }
 Config cfg_;bool latched_=true,operator_approved_=false;
 int preferred_=0,side_=0;int clear_frames_=0;uint64_t last_clear_ms_=UINT64_MAX;
 uint64_t last_clear_seq_=0;
};
} // namespace stock_px4
