#include "stock_px4_supervisor/differential.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
int main(){
 using namespace stock_px4;
 DifferentialInput v;
 assert(!preview_wheels(v).valid);
 v.motion_advisory_valid=true;
 assert(!preview_wheels(v).valid);
 v.geometry_verified=true;v.wheel_track_m=0.60;v.wheel_speed_limit_mps=1.;
 v.speed_mps=0.25;v.yaw_radps=0;
 auto a=preview_wheels(v);
 assert(a.valid&&std::abs(a.left_mps-.25)<1e-10&&std::abs(a.right_mps-.25)<1e-10);
 v.speed_mps=0;v.yaw_radps=1.;
 a=preview_wheels(v);
 assert(a.valid&&std::abs(a.left_mps+.30)<1e-10&&std::abs(a.right_mps-.30)<1e-10);
 v.yaw_radps=-1.;a=preview_wheels(v);
 assert(a.valid&&std::abs(a.left_mps-.30)<1e-10&&std::abs(a.right_mps+.30)<1e-10);
 v.speed_mps=2.;v.yaw_radps=1.;a=preview_wheels(v);
 assert(a.valid&&a.right_mps<=1.&&a.left_mps<=1.&&a.left_mps>0);
 v.speed_mps=0;v.yaw_radps=0;v.wheel_track_m=0;
 assert(!preview_wheels(v).valid);
 v.wheel_track_m=.60;v.speed_mps=std::numeric_limits<double>::quiet_NaN();
 assert(!preview_wheels(v).valid);
 v.speed_mps=0;v.wheel_speed_limit_mps=0;
 assert(!preview_wheels(v).valid);
 v.wheel_speed_limit_mps=1.;v.motion_advisory_valid=false;
 a=preview_wheels(v);assert(!a.valid&&a.left_mps==0&&a.right_mps==0);
 std::cout<<"DIFFERENTIAL_KINEMATICS_TEST_PASS\n";
}
