#pragma once
#include "AvoidanceWire.hpp"
#include <cmath>
#include <cstdint>
namespace avoidance {
enum class State : uint8_t {
 MISSION=0, AVOIDING=1, STOPPING=2, STALE_CLEAR=3,
 STALE_HAZARD_HOLD=4, MANUAL_RC=5, LINK_NOT_SEEN=6
};
struct Output {float vx, yaw;State state;bool advisory_fresh;};
class Mixer {
 public:
  explicit Mixer(uint32_t timeout_ms=350) : timeout_ms_(timeout_ms) {}
  void reset() {seen_=false;hazard_latch_=false;clear_streak_=0;seq_=0;session_=0;}
  bool accept(const uint8_t *buf,size_t length,uint64_t now_ms) {
    AdvisoryWire next{};
    if(!decode(buf,length,next))return false;
    // Same session requires monotonic modulo-16bit sequence; unrelated
    // restart accepted only after timeout to prevent stale session reset.
    if(seen_ && next.session==session_) {
      uint16_t delta=static_cast<uint16_t>(next.sequence-seq_);
      if(delta==0 || delta>32767)return false;
    } else if(seen_ && (now_ms<=last_rx_ ||
             now_ms-last_rx_<=timeout_ms_))return false;
    session_=next.session;seq_=next.sequence;last_rx_=now_ms;
    latest_=next;seen_=true;
    if((next.flags&(OBSTACLE|STOP))!=0) {
      hazard_latch_=true;clear_streak_=0;
    } else if(hazard_latch_) {
      if(++clear_streak_>=3) {hazard_latch_=false;clear_streak_=0;}
    }
    return true;
  }
  Output step(uint64_t now_ms,float mission_vx,float mission_yaw,
              bool auto_mission,bool manual_mode) const {
    if(!std::isfinite(mission_vx)||!std::isfinite(mission_yaw))
      return {0,0,State::STOPPING,false};
    // PX4 RC / mode selection overrides ROS advisory. Physical E-STOP is
    // out-of-band and must not be bypassed by any software mixer.
    if(manual_mode||!auto_mission)
      return {mission_vx,mission_yaw,State::MANUAL_RC,false};
    if(!seen_)return {mission_vx,mission_yaw,State::LINK_NOT_SEEN,false};
    bool fresh=now_ms>=last_rx_ && (now_ms-last_rx_<=timeout_ms_);
    if(!fresh) {
      if(hazard_latch_) return {0,0,State::STALE_HAZARD_HOLD,false};
      return {mission_vx,mission_yaw,State::STALE_CLEAR,false};
    }
    if(hazard_latch_ && !(latest_.flags&(OBSTACLE|STOP))) {
      // Require multiple independent clear scans; never clear after only
      // one fresh packet following a hazardous episode or reconnection.
      return {0,0,State::STOPPING,true};
    }
    if(latest_.flags&STOP)return {0,0,State::STOPPING,true};
    const float limit=latest_.vx_limit_mps;
    const float speed=(mission_vx<0.f)?0.f:(mission_vx>limit?limit:mission_vx);
    const float combined_yaw=mission_yaw+latest_.yaw_correction_radps;
    const float yaw=combined_yaw<-1.f?-1.f:(combined_yaw>1.f?1.f:combined_yaw);
    return {speed,yaw,(latest_.flags&OBSTACLE)?State::AVOIDING:State::MISSION,true};
  }
  bool ever_connected()const{return seen_;}
  bool hazard_latched()const{return hazard_latch_;}
 private:
  uint32_t timeout_ms_{350},session_{0};
  uint16_t seq_{0};
  uint64_t last_rx_{0};
  AdvisoryWire latest_{};
  bool seen_{false},hazard_latch_{false};
  uint8_t clear_streak_{0};
};
} // namespace avoidance
