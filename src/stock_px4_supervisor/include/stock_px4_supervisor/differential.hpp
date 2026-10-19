#pragma once
#include <algorithm>
#include <cmath>

namespace stock_px4 {
struct DifferentialInput {
 double speed_mps=0, yaw_radps=0;
 double wheel_track_m=0, wheel_speed_limit_mps=0;
 bool motion_advisory_valid=false, geometry_verified=false;
};
struct DifferentialOutput {
 double left_mps=0,right_mps=0;
 bool valid=false;
 const char* reason="INVALID_OR_NOT_VERIFIED";
};
inline DifferentialOutput preview_wheels(const DifferentialInput& in) {
 DifferentialOutput out;
 if (!in.motion_advisory_valid) {out.reason="SAFETY_STOP_OR_UNKNOWN";return out;}
 if (!in.geometry_verified) {out.reason="GEOMETRY_UNVERIFIED";return out;}
 if (!std::isfinite(in.speed_mps)||!std::isfinite(in.yaw_radps)||
     !std::isfinite(in.wheel_track_m)||!std::isfinite(in.wheel_speed_limit_mps)||
     in.wheel_track_m<=0 || in.wheel_track_m>5 ||
     in.wheel_speed_limit_mps<=0 || in.wheel_speed_limit_mps>5) {
  out.reason="INVALID_GEOMETRY_OR_NUMERIC";return out;
 }
 const double left=in.speed_mps-in.yaw_radps*in.wheel_track_m*0.5;
 const double right=in.speed_mps+in.yaw_radps*in.wheel_track_m*0.5;
 const double peak=std::max(std::abs(left),std::abs(right));
 // Preserve curvature by uniformly scaling both wheels.
 const double ratio=peak>in.wheel_speed_limit_mps?in.wheel_speed_limit_mps/peak:1.0;
 out.left_mps=left*ratio;
 out.right_mps=right*ratio;
 out.valid=true;
 out.reason="SIMULATION_PREVIEW_ONLY";
 return out;
}
} // namespace stock_px4
