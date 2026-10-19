#include "stock_px4_supervisor/safety.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>
using namespace stock_px4;
Input good(){
 Input i; i.rc_known=true;i.lidar_valid=true;i.tf_valid=true;
 i.calibration_verified=true;i.mav_connected=true;i.estimator_valid=true;
 i.imu_valid=true;i.steering_feedback_valid=true;i.mission_auto=true;
 i.lidar_age_ms=0;i.mav_age_ms=0;i.imu_age_ms=0;
 i.front_m=7;i.left_m=2;i.right_m=2;i.speed_mps=0.2;
 return i;
}
void clears(Safety &s,Input i,uint64_t t){
 s.acknowledge_operator();
 for(int k=0;k<5;k++)s.step(i,t+100*k);
}
int main(){
 int count=0;
 auto test=[&](bool b,const char* name){if(!b){std::cerr<<"FAIL "<<name<<"\n";std::exit(1);}++count;};
 Config c;test(c.valid(),"config-valid");
 c.decel_mps2=0;test(!c.valid(),"reject-zero-decel");c.decel_mps2=.8;
 c.clear_confirm_frames=2;test(!c.valid(),"require-three-scan-confirm");c.clear_confirm_frames=3;
 auto i=good(); Safety s(c);
 test(s.step(i,0).state==State::LATCHED,"boot-never-move");
 clears(s,i,200);
 test(s.step(i,800).state==State::CLEAR,"operator-clear-after-scan-proof");
 test(!s.step(i,900).hazard_latched,"clear-after-operator");
 i.front_m=0.5;
 auto d=s.step(i,1000);test(d.state==State::STOP && d.stop,"obstacle-stop");
 i.front_m=7;
 test(s.step(i,1100).state==State::LATCHED,"stop-remains-latched");
 for(int k=0;k<10;k++)s.step(i,1200+k*100);
 test(s.latched(),"scans-without-ack-cannot-reset");
 s.acknowledge_operator();test(s.step(i,2300).state==State::CLEAR,"ack-and-three-scans");
 i.lidar_age_ms=301;test(s.step(i,2400).state==State::UNKNOWN,"lidar-timeout");
 i.lidar_age_ms=0;test(s.step(i,2500).state==State::LATCHED,"timeout-recovery-ack-required");
 clears(s,i,2600);
 test(s.step(i,3200).state==State::CLEAR,"latched-recovered");
 i.mav_age_ms=1600;test(s.step(i,3300).state==State::UNKNOWN,"mavlink-loss");
 i.mav_age_ms=0;clears(s,i,3400);
 i.imu_age_ms=500;test(s.step(i,4000).state==State::UNKNOWN,"imu-loss");
 i.imu_age_ms=0;clears(s,i,4100);
 i.rc_known=false;test(s.step(i,4700).state==State::UNKNOWN,"rc-unknown-blocks");
 i.rc_known=true;clears(s,i,4800);
 i.manual_rc=true;test(s.step(i,5400).state==State::MANUAL_RC,"rc-manual-priority");
 i.manual_rc=false;clears(s,i,5500);
 i.estop=true;test(s.step(i,6200).state==State::ESTOP,"estop-over-everything");
 i.manual_rc=true;test(s.step(i,6300).state==State::ESTOP,"estop-over-rc");
 i.estop=false;i.manual_rc=false;clears(s,i,6400);
 i.front_m=2.0;i.left_m=2.2;i.right_m=0.4;
 d=s.step(i,7100);test(d.state==State::AVOID_PROPOSED,"suggest-left");
 test(d.side==1 && d.stop && d.vx_limit_mps==0,"suggest-never-actuates");
 test(d.yaw_correction_rps==0,"unverified-differential-geometry-no-turn-preview");
 {
   Config diff=c;diff.differential_geometry_verified=true;
   diff.differential_wheel_track_m=.60;diff.max_wheel_speed_mps=.35;
   test(diff.valid(),"measured-differential-config-valid");
   Safety sim_diff(diff); auto segment=good();
   clears(sim_diff,segment,17000);
   segment.front_m=2.;segment.left_m=2.2;segment.right_m=.4;
   auto y=sim_diff.step(segment,17500);
   test(y.state==State::AVOID_PROPOSED && y.yaw_correction_rps>0,
        "differential-left-turn-suggestion-only");
   test(y.stop && y.vx_limit_mps==0,"differential-does-not-actuate");
 }

 i.front_m=7;clears(s,i,7200);
 i.front_m=2.0;i.left_m=0.2;i.right_m=2.3;
 d=s.step(i,7900);test(d.state==State::AVOID_PROPOSED&&d.side==-1,"suggest-right");
 i.front_m=7;clears(s,i,8000);
 i.speed_mps=2.0;i.front_m=3;
 d=s.step(i,8600);
 test(d.state==State::STOP&&d.stop_distance_m>3.0,"dynamic-braking");
 i.speed_mps=129.7;i.front_m=7;
 d=s.step(i,8650);
 test(d.state==State::UNKNOWN&&d.stop&&
      d.reason=="IMPLAUSIBLE_SPEED_TELEMETRY","reject-px4-uninitialized-speed-glitch");
 i.speed_mps=.2;i.front_m=7;clears(s,i,8700);
 i.front_m=std::numeric_limits<double>::quiet_NaN();
 test(s.step(i,9300).state==State::UNKNOWN,"reject-nan");
 i.front_m=7;clears(s,i,9400);
 i.steering_feedback_valid=false;test(s.step(i,10000).state==State::UNKNOWN,"missing-steering-feedback");
 i.steering_feedback_valid=true;clears(s,i,10100);
 i.estimator_valid=false;test(s.step(i,10700).state==State::UNKNOWN,"estimator-invalid");
 i.estimator_valid=true;clears(s,i,10800);
 i.calibration_verified=false;test(s.step(i,11400).state==State::UNKNOWN,"uncalibrated-sensor");
 i.calibration_verified=true;clears(s,i,11500);
 i.mission_auto=false;test(s.step(i,12100).state==State::UNKNOWN,"mission-mode-unverified");
 i.mission_auto=true;clears(s,i,12200);
 i.tf_valid=false;test(s.step(i,12800).state==State::UNKNOWN,"tf-timeout");
 i.tf_valid=true;clears(s,i,12900);
 // Four tested link-loss durations (no manual-reset bypass).
 for(int ms:{300,1000,5000,30000}){
   i.mav_connected=false;
   test(s.step(i,14000+ms).stop,"loss-safestop");
   i.mav_connected=true;
   test(s.step(i,14001+ms).hazard_latched,"hotplug-rejoin-latch");
   clears(s,i,15000+ms);
 }
 for(int k=0;k<100;k++){
  i.mav_connected=false;test(s.step(i,50000+k*1000).stop,"reconnect-loss");
  i.mav_connected=true;test(s.step(i,50100+k*1000).hazard_latched,"reconnect-no-autoresume");
  clears(s,i,50200+k*1000);
 }
 // Upstream ROS perception STOP must override an apparently clear geometric sector.
 {
   Safety guard;auto v=good();
   clears(guard,v,170000);
   test(guard.step(v,171000).state==State::CLEAR,"pre-upstream-clear");
   v.perception_stop=true;
   test(guard.step(v,171100).state==State::STOP,"perception-stop-veto");
   v.perception_stop=false;
   test(guard.step(v,171200).state==State::LATCHED,"perception-stop-latched");
 }
 // Live ROS clearance proof counts distinct scan_seq, never multiple timer ticks.
 {
   Safety s2;auto f=good();f.scan_seq=1;s2.acknowledge_operator();
   test(s2.step(f,200000).state==State::LATCHED,"first-fresh-scan");
   test(s2.step(f,200100).state==State::LATCHED,"replayed-same-scan-1");
   test(s2.step(f,200200).state==State::LATCHED,"replayed-same-scan-2");
   f.scan_seq=2;test(s2.step(f,200300).state==State::LATCHED,"second-fresh-scan");
   f.scan_seq=3;test(s2.step(f,200400).state==State::CLEAR,"third-distinct-scan");
 }
 std::cout<<"PASS stock PX4 advisory safety assertions="<<count
          <<" (hardware actuation untouched, simulation unit test)\n";
}
