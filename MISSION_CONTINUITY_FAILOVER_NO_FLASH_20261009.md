# ROS2 ↔ PX4 Stock — Mission Continuity and Failover Contract
Status: 2026-10-09, software-only. **No permission for autonomous motion or FCU write.**

## What can and cannot happen
- PX4 AUTO Mission and PX4 OFFBOARD are **mutually exclusive operating modes**. The Cube keeps its mission data when entering OFFBOARD, but mission execution is not simultaneously AUTO.
- In PX4 OFFBOARD, ROS2 can feed a supported 2D velocity VECTOR through MAVLink `SET_POSITION_TARGET_LOCAL_NED` when (and only when) the firmware/airframe supports it. Yaw and yaw_rate in this MAVLink message do **not** control Ackermann steering directly: rover derives course from vx/vy.
- PX4 OFFBOARD's liveness timer and failsafe `COM_OF_LOSS_T` and `COM_OBL_RC_ACT` choose an action after the stream disappears. No assumption that that action is AUTO Mission. Current Cube backup: `COM_OF_LOSS_T=1`, `COM_OBL_RC_ACT=0`; actual installed firmware version and exact action semantics not proven.
- Mission preserved, Mission running, and Vehicle moving safely are distinct. PX4 cannot infer LiDAR hazards once its companion is gone unless a suitable independent safety system is installed.
- RC should have highest operator control authority below hard-wired E-STOP. RC stick values alone do not prove a valid takeover; check PX4 mode state and receiver status and simulate RC loss/recovery.
- Firmware stock has not demonstrated onboard true AUTO mixing, so no claim that PX4 continues AUTO with per-frame ROS2 steering corrections.

## Desired finite-state design
| Mode | Entry conditions | FCU mode | Actual behavior |
|---|---|---|---|
| MISSION | Valid localization, enabled airframe, valid mission, RC failsafe and hardware E-STOP healthy | AUTO Mission | PX4 owns waypoint and motor/steering; ROS2 only observes sensors and calculates advice |
| AVOID_REQUESTED | Valid obstacle evidence and clearance/trajectory verification | Remain AUTO until accepted transition | Verify plan and MAVLink state; if no safe bypass, request/maintain STOP through independent safety hardware |
| AVOID_ACTIVE | Confirmed actual ACK + heartbeat mode OFFBOARD, valid estimator and velocity-vector controller, steering constraints, RC not overriding | OFFBOARD | ROS2 provides short-leased planar velocity vectors; NOT AUTO mixing. Mission remains saved on Cube |
| REJOIN | Obstacle-free evidence, path-to-mission cleared, fresh location, validated mission item index & continuity, explicit safety permit | Transition OFFBOARD → AUTO | Resume mission **only if** mode ACK and mission progress/position checks succeed |
| RC_MANUAL | Confirmed manual mode/receiver takeover | MANUAL / RC-selected mode | Stop ROS2 setpoints immediately; keep mission stored; do not forcibly return to AUTO |
| COMPANION_LOSS | ROS2 command stream absent beyond timeout | Actual PX4 configured offboard-loss failsafe | **NO GUARANTEED AUTO RESUME**; current failsafe semantics must be verified on installed firmware. Independent hardware should inhibit motion when obstacle remains/unknown |
| UNKNOWN/HAZARD_LATCH | LiDAR, TF, IMU, position, steering feedback, watchdog, E-STOP uncertain | Independent hold/stop where available | Fail closed; do not auto-reset when mini-PC reconnects |

## Software implemented (not hardware enabled)
- New `pixhawk_interfaces/srv/VelocityVector.srv`.
- `pixhawk_bridge` has an optional `offboard_velocity_vector` service and read-only `/pixhawk/firmware/version` publisher parsing MAVLink `AUTOPILOT_VERSION` (requires a controlled live bridge deployment; existing bridge remains running from old cache).
- Rover velocity service is gated OFF by default: `enable_motion_commands=false`, `allow_mode_changes=false`, `allow_stock_rover_velocity=false`, `stock_rover_interface_confirmed=false`; also checks MAV_TYPE_GROUND_ROVER, PX4 mode, local estimator, finite 2D speed and bounded lease 0.1..0.35 s.
- MAVLink message uses velocity x/y only, local NED frame, yaw/yaw-rate ignored, masked per MAVLink type mask; stream is time-limited and aborted on non-AUTO/non-OFFBOARD heartbeat mode. Not intended to take over RC and not deployed to hardware.
- Pseudo-serial MAVLink simulator test: `sim_bench.py` verifies the new velocity service **DENIED** in default configuration while preexisting mission upload/download and other simulated commands continue to work. This does not establish acceptance by installed Cube.
- PX4 exact version still UNKNOWN, airframe still unconfigured (`SYS_AUTOSTART=0`, `MAV_TYPE=0`), stored mission backup has 0 waypoints and position invalid. There is no physical rover AUTO route to resume at this time.

## Critical answer to requirement 'ROS crash then PX4 automatically continues the waypoint'
*Achievable conditionally, but not proven with the installed stock firmware, and NOT safe to promise on present hardware.*
1. PX4 must first have a configured and calibrated rover airframe, valid horizontal localization, intact waypoint mission, receiver and E-STOP failsafes.
2. Version-specific Rover AUTO → OFFBOARD → AUTO switching and mission index must pass matched SITL and fault-injection testing, not just MAVLink ACK.
3. Autonomous recovery on total ROS/mini-PC loss requires an **independent watchdog/safety path** with an independent hazard-clear decision; an action programmed exclusively on the mini-PC cannot execute after that mini-PC fails.
4. If obstacle evidence was present or becomes unknown at loss, independent safety must hold/stop until new safe clearance is established. 'Always resume automatically regardless of hazards' is a dangerous requirement and is explicitly rejected.
5. If the vehicle was demonstrably in an obstacle-free, independently verified corridor and PX4 firmware has an explicitly supported autonomous resume mechanism, an independent supervisor may permit reentry to AUTO; otherwise stop/hold and require manual intervention.
6. Physical actuator wiring, FCU parameter changes, arming, AUTO change, and mission editing are all out of scope until separately approved.

## Source references
- PX4 current: https://docs.px4.io/main/en/flight_modes/offboard (Rover: 2D vx/vy accepted, Ackermann yaw/yaw_rate ignored; offboard-loss action version-dependent).
- PX4 current: https://docs.px4.io/main/en/config/safety (COM_OF_LOSS_T and COM_OBL_RC_ACT).
- PX4 rover drive modes: https://docs.px4.io/main/en/flight_modes_rover/ (Mission and Manual are distinct modes).
