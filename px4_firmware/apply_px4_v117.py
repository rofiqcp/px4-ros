#!/usr/bin/env python3
"""Apply source-only PX4 v1.17 Ackermann mixer patch. NEVER flashes hardware."""
from pathlib import Path
import shutil,sys,datetime
root=Path("/media/sirobo/HDD/autopilot/PX4-Autopilot-v1.17.0")
mod=root/"src/modules/rover_ackermann"
here=Path("/media/sirobo/HDD/pixhawk")
backup=here/"px4_firmware"/"backups"/datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
backup.mkdir(parents=True,exist_ok=True)
def patch(path,items):
 p=Path(path)
 s=p.read_text()
 if "MID360 PX4 mixer v1" in s:
  print("SKIP (already patched)",p);return
 orig=s
 for old,new in items:
  if old not in s:raise RuntimeError("Anchor missing "+str(p)+": "+old[:100])
  s=s.replace(old,new,1)
 if s==orig:raise RuntimeError("No changes "+str(p))
 dest=backup/p.relative_to(root)
 dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,dest)
 p.write_text(s)
 print("PATCHED",p)
for name in ["AvoidanceWire.hpp","AvoidanceMixer.hpp"]:
 shutil.copy2(here/"avoidance_core"/name,mod/name)
shutil.copy2(here/"px4_firmware"/"PX4AvoidanceState.hpp",mod/"PX4AvoidanceState.hpp")
patch(mod/"RoverAckermann.hpp",[
 ('#include <uORB/topics/vehicle_status.h>',
  '#include <uORB/topics/vehicle_status.h>\n#include <uORB/topics/mavlink_tunnel.h> // MID360 PX4 mixer v1'),
 ('\tuORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};',
  '\tuORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};\n\tuORB::Subscription _avoidance_tunnel_sub{ORB_ID(mavlink_tunnel)};')
])
patch(mod/"RoverAckermann.cpp",[
 ('#include "RoverAckermann.hpp"',
  '#include "RoverAckermann.hpp"\n#include "PX4AvoidanceState.hpp" // MID360 PX4 mixer v1'),
 ('void RoverAckermann::Run()\n{',
 '''void RoverAckermann::Run()
{
    // MAVLink TUNNEL goes to uORB; advice has per-frame CRC, version,
    // session and sequence. PX4 owns the 350ms watchdog and hazard latch.
    mavlink_tunnel_s tunnel{};
    if (_avoidance_tunnel_sub.update(&tunnel) &&
        tunnel.payload_type == avoidance::kTunnelPayloadType &&
        tunnel.payload_length == sizeof(avoidance::AdvisoryWire)) {
        px4_avoid::shared().mixer.accept(
            tunnel.payload, tunnel.payload_length, hrt_absolute_time()/1000);
    }
    vehicle_status_s avoidance_status{};
    if (_vehicle_status_sub.copy(&avoidance_status)) {
        px4_avoid::shared().mission_auto =
            (avoidance_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION);
    }
''')
])
patch(mod/"AckermannSpeedControl/AckermannSpeedControl.cpp",[
 ('#include "AckermannSpeedControl.hpp"',
  '#include "AckermannSpeedControl.hpp"\n#include "../PX4AvoidanceState.hpp" // MID360 PX4 mixer v1'),
 ('\tif (PX4_ISFINITE(_speed_setpoint)) {\n\t\tconst float speed_setpoint = math::constrain(_speed_setpoint,',
 '''\tif (PX4_ISFINITE(_speed_setpoint)) {
        const auto mixed=px4_avoid::shared().mixer.step(
            _timestamp/1000,_speed_setpoint,0.0f,
            px4_avoid::shared().mission_auto,!px4_avoid::shared().mission_auto);
        const float speed_setpoint = math::constrain(mixed.vx,''')
])
patch(mod/"AckermannRateControl/AckermannRateControl.cpp",[
 ('#include "AckermannRateControl.hpp"',
  '#include "AckermannRateControl.hpp"\n#include "../PX4AvoidanceState.hpp" // MID360 PX4 mixer v1'),
 ('\tif (PX4_ISFINITE(_yaw_rate_setpoint)) {',
 '''\tif (PX4_ISFINITE(_yaw_rate_setpoint)) {
        const auto mixed=px4_avoid::shared().mixer.step(
            _timestamp/1000,0.0f,_yaw_rate_setpoint,
            px4_avoid::shared().mission_auto,!px4_avoid::shared().mission_auto);'''),
 ('float constrained_yaw_rate = math::constrain(_yaw_rate_setpoint,',
  'float constrained_yaw_rate = math::constrain(mixed.yaw,')
])
# Dedicated PX4 MAVLink source filter; CRC is not cryptographic, use secure serial.
receiver=root/"src/modules/mavlink/mavlink_receiver.cpp"
patch(receiver,[
 ('\tmavlink_msg_tunnel_decode(msg, &mavlink_tunnel);',
  '''\tmavlink_msg_tunnel_decode(msg, &mavlink_tunnel);
    // MID360 PX4 mixer v1: only dedicated companion sysid/compid, addressed
    // to this autopilot. No control injection via generic GCS TUNNEL.
    if (mavlink_tunnel.payload_type == 42001 &&
        (msg->sysid != 245 || msg->compid != MAV_COMP_ID_ONBOARD_COMPUTER ||
         mavlink_tunnel.target_system != _mavlink.get_system_id() ||
         mavlink_tunnel.target_component != MAV_COMP_ID_AUTOPILOT1)) return;''')
])
print("BACKUP",backup)
print("SOURCE_PATCH_ONLY: No PX4 binary flashed, no Cube params changed.")
