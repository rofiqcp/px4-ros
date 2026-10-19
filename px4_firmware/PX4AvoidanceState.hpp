#pragma once
// Compiled into PX4 rover_ackermann; no ROS2 runtime dependency.
// The PX4 AUTO mission and RC/manual steering remain firmware-owned.
#include "AvoidanceWire.hpp"
#include "AvoidanceMixer.hpp"

namespace px4_avoid {
struct State {
  avoidance::Mixer mixer{350};
  bool mission_auto{false};
};
inline State& shared() {
  static State v;
  return v;
}
} // namespace px4_avoid
