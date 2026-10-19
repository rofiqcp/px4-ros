# Audit Implementasi RC Manual Takeover → Stock PX4 (No Flash)
Host: sirobo | 9 Oktober 2026 | Status: **SIMULATOR VERIFIED; CUBE RC NOT YET VERIFIED**

## Requirement and correct hardware hierarchy

```text
Radio transmitter --RF--> RC receiver --RCIN/SBUS/CRSF/etc--> PX4 flight controller
                                                             |       |
                                                             |       +--> RC manual mode → steering + throttle
                                                             |
Mini-PC sirobo → ROS2 obstacle planner → stock PX4 MAVLink ------> OFFBOARD mode only
                    |                                        |
                    +--> telemetry, status, safety advisory   +--> PX4 mission saved locally
Independent hardware E-STOP -------------------------------------> motor enable cutout
```

**Key distinction**: RC receiver and PX4 must keep working without the mini-PC. When the RC mode switch selects MANUAL, PX4 must leave OFFBOARD and accept the operator input. ROS2 does not emulate RC input, inject PWM, change RC channels, or manipulate PX4 odometry. PX4 AUTO Mission and OFFBOARD are different modes. An OFFBOARD-loss failsafe is NOT proof of automatic return to AUTO; the transmitter operator must be able to select MANUAL physically. No guarantee that a rover can move safely after power/firmware/RC failures.

## What was found on the real Cube through the previously running, read-only ROS2 bridge

- USB MAVLink connected, PX4 HEARTBEAT decoded but `vehicle_type=0` (generic), `mav_state=0` (uninitialized), `airframe_configured=false`, `local_valid=false`, `armed=false`.
- Real Cube backup has 732 parameters, `SYS_AUTOSTART=0`, `MAV_TYPE=0`, `COM_RC_IN_MODE=3`, `COM_RC_OVERRIDE=1`, `COM_OF_LOSS_T=1`, `COM_OBL_RC_ACT=0`; do not interpret an unknown firmware's parameter semantics without identifying the firmware.
- During seven seconds of live passive observation: `RC_CHANNELS` **0 observed**, `SERVO_OUTPUT_RAW` **0 observed**, `VFR_HUD` **0 observed**. **Cannot conclude** receiver is absent; streams might not be configured. Cannot claim physical RC tested.
- Existing physical bridge binary remains from `/home/sirobo/.cache/pixhawk_ros`; the new source is compiled in the mandatory unified cache. **No restart of the active hardware-facing bridge** was performed; thus new RC status fields and AUTOPILOT_VERSION parser are not deployed on the Cube.
- No firmware flash, no Cube parameter/mission change, no arming, no motor movement, no hardware mode change.

## Software changes implemented

1. `src/pixhawk_bridge/src/pixhawk_node.cpp`: reads PX4 HEARTBEAT and decodes main/submode (PX4 only), publishes `base_mode`, `px4_main_mode`, `px4_sub_mode`, `manual_mode_selected`, `manual_takeover_latched`, and RC_CHANNELS freshness in `/pixhawk/status`. Raw RC samples are *telemetry*, not proof of physical steering. ArduPilot is not decoded with PX4 custom-mode semantics.
2. The direct, **manual-mode takeover latch** is set when PX4 reports main mode MANUAL=1. It cancels active ROS setpoints; blocks new OFFBOARD velocity leases, GOTO setpoint services, ROS mode commands and non-read-only commands (emergency disarm remains allowed). Lock stays engaged even if PX4 later reports OFFBOARD again. Operator RC commands remain handled by PX4 directly and are never generated/blocked by the companion program. A deliberate new software session is needed before external control is reconsidered, avoiding blind auto-resume.
3. New `offboard_velocity_vector` uses bounded-duration `SET_POSITION_TARGET_LOCAL_NED` with **vx/vy only** (planar NED, type mask `0x0DE7`; ignores position, vertical velocity, accel, yaw/yaw rate). Service is fully **disabled by default** (`enable_motion_commands=false`, `allow_mode_changes=false`, `allow_stock_rover_velocity=false`, `stock_rover_interface_confirmed=false`) and requires configured, position-valid PX4 Rover. The read-only physical system cannot execute this.
4. `src/stock_px4_supervisor/src/stock_px4_supervisor.cpp`: removes dead `&& false` logic, includes future-capable `rc_priority_verified` and fresh RC telemetry gate and PX4 AUTO_MISSION decoding; steering angle feedback remains intentionally unproven and forced invalid (no suitable measured steering sensor message in this project).
5. Dashboard `slam3d-livox-mid360s/foxglove/stock_px4_tuning.html` now distinguishes RC verified, receiver telemetry fresh/stale, manual mode selection, and main/submode instead of presenting an unconditional AUTO status.

**The companion must not disable or override real RC mode-switch authority.** The software latch protects the specific added bridge, not every external GCS/ROS process that might separately write to PX4. Exclusive policy, QGC joystick behavior and receiver setup must be validated on the actual Cube.

## Critical RC configuration: `COM_RC_IN_MODE=3`

PX4 documentation at https://docs.px4.io/main/en/config/manual_control states source selection `3` means *keep the first valid manual-control source*, not RC-priority. Other current values include `0`: RC only; `5`: RC-first prioritization. **These are documentation candidates, not instructions to write an unknown-version Cube without physical receiver verification.** If a MAVLink joystick is bound as the first source, simply possessing an RC transmitter does not prove it will override. `COM_RC_OVERRIDE` auto-stick takeover is platform/mode-specific and should NOT be assumed on Ackermann.

PX4 documentation https://docs.px4.io/main/en/flight_modes/offboard and https://docs.px4.io/v1.15/en/flight_modes_mc/offboard warns OFFBOARD setpoint proof-of-life (>2 Hz) is required and fallback action is determined by `COM_OF_LOSS_T` / `COM_OBL_RC_ACT`; not automatically AUTO Mission.

## Independent simulator verification

Test fixture: `scripts/test_rc_takeover_pty.py` running exclusively on dynamically allocated Linux PTY and isolated `ROS_DOMAIN_ID=82`. This is a PX4-like MAVLink message fixture, **not** full PX4 SITL or true physical PX4 runtime.

Checks include:
- Rover heartbeat in OFFBOARD + fresh RC_CHANNELS telemetry.
- ROS2 service command → exact MAVLink velocity `SET_POSITION_TARGET_LOCAL_NED`, 10 Hz nominal bridge tick, type mask `0x0DE7`, finite low speed.
- RC mode switch to MANUAL main=1 cancels old setpoint stream.
- New velocity vector, ROS SetMode and GotoLocal requests are refused during manual takeover; mission download still works and two simulated waypoints remain intact.
- Switching simulated PX4 heartbeat back to OFFBOARD does not reactivate old commands and does not release RC lock.
- RC telemetry loss is detected after freshness timeout.
- Terminating companion bridge process does not terminate *simulated* receiver messages running in independent FC fixture.
- Test log: `/home/sirobo/.cache/mid360_pixhawk_ros/rc_takeover_pty_test_final.log`.

Older `scripts/sim_bench.py` defaults to denying the new velocity service, preserving the fail-closed deployment default. Stock C++ supervisor's fault tests use CTest in unified cache.

## Operational guidance (only disarmed hardware)

1. **RC receiver must wire to PX4 RC input, NOT to mini-PC.** Pair transmitter and receiver, bind RX, and verify true input and mode switch with QGroundControl Radio/Actuators diagnostics *with drivetrain disabled/disconnected*, keeping physical E-STOP present.
2. Determine Cube AUTOPILOT_VERSION and actual airframe class read-only through the single serial-owner bridge. Existing Cube `SYS_AUTOSTART=0` and `MAV_TYPE=0` are a hard stop for real AUTO/OFFBOARD/motion tests.
3. Confirm manual-control source priority vs joystick and exact `COM_RC_IN_MODE` semantics on actual firmware. Do not silently change the parameter during this audit.
4. Verify manual mode switch causes an actual PX4 HEARTBEAT mode change, channels update and servo/motor feedback can be read without sending any motor command; these are **separate physical tests** requiring explicit approval and safe bench.
5. Verify RC failsafe (transmitter power off), mini-PC cable pull, RC reconnect, manual latch, E-STOP and operator takeover independently, always bench-disarmed first. Mission-preservation is not mission-execution or physical safe-motion proof.
6. If ROS2 fails during obstacle avoidance, **RC pilot takeover is the fallback**, not automatic autonomous resumption. If the pilot cannot see or control safely or no RC is available, safe HOLD/STOP via an independently tested failsafe is required.
7. The source patch, ROS2 user-facing bridge and source tuning must remain disabled for hardware motion until exact version, receiver, airframe, safety and legal test approval are proven.

## Build and rollback

```bash
cd /media/sirobo/HDD/pixhawk
./build.sh
source env.sh
# Simulator only. This test never opens /dev/ttyACM0:
ROS_DOMAIN_ID=82 python3 scripts/test_rc_takeover_pty.py
# This folder is not currently a Git worktree. Restore source only from a verified backup.
# Software snapshot: backups/rc_takeover_verified_software_20261009.tar.gz
# Do not alter the live Pixhawk serial bridge or firmware to roll back a simulator test.
```
The active Cube bridge was **not** replaced/restarted. Its existing hardware command permissions remain disabled. Existing SLAM/mapping/web ports are preserved. This document supersedes unverified assumptions about automatic RC priority in previous plans.

## Still BLOCKED
- Physical RC connection, receiver binding, channel mapping, failsafe, mode-switch observation.
- Exact stock Cube AUTOPILOT_VERSION, firmware-matched PX4 SITL and proven Ackermann Rover OFFBOARD support.
- Calibrated steering feedback, validated localization, loaded mission, obstacle safety guarantee on mini-PC loss.
- Autonomous mission return on ROS failure; it is not synonymous with RC manual takeover.

## Final quality-assurance results
- Unified `./build.sh` for both workspaces: **12/12 ROS2 packages PASS**. Log `/home/sirobo/.cache/mid360_pixhawk_ros/rc_takeover_full_unified_build.log`.
- Simulator-based Rover/RC PTY regression: **PASS**, 9 valid velocity packets at planar `0x0DE7` mask, manual takeover immediately cancels setpoint lease, manual mode disallows ROS velocity, mode and GOTO requests, two simulator mission waypoints preserved, sticky manual latch prevents restart, RC signal timeout detected, receiver fixture unaffected when companion process stops. Log `/home/sirobo/.cache/mid360_pixhawk_ros/rc_takeover_pty_test_final.log`.
- Previous synthetic MAVLink bench regression: **PASS**; hardware-safe default denies `offboard_velocity_vector` while simulator param operations, mission download/upload and GOTO pass. Log `/home/sirobo/.cache/mid360_pixhawk_ros/rc_takeover_legacy_regression.log`.
- Supervisor CTest: **1 test, zero failures**. Log `/home/sirobo/.cache/mid360_pixhawk_ros/rc_takeover_ctest.log`.
- cppcheck (warning/style/performance): **no code errors or unused/dead-code findings**. Two advisory performance suggestions remain for constructor-body member initialization that follows runtime ROS parameter declarations: `device_` and `safety_`; they are not unused code or motion-safety warnings. Log `/home/sirobo/.cache/mid360_pixhawk_ros/rc_takeover_cppcheck.log`.
- Browser Playwright: **PASS**, 19 inputs, no JS errors, no 390px mobile overflow. It explicitly shows `RC not verified | receiver data: unknown/stale | PX4 mode: UNKNOWN` because real hardware RC proof is unavailable. Screenshots: `/media/sirobo/HDD/slam3d-livox-mid360s/results/stock_px4_browser/rc_takeover_desktop.png` and `rc_takeover_mobile.png`. Local mapping and tuning servers both HTTP 200.
- Live Cube bridge guard readback: `enable_motion_commands=false`, `allow_parameter_writes=false`, `allow_mode_changes=false`, `allow_mission_writes=false`. The active hardware-facing bridge process still runs the former read-only binary. The changed source is only in unified cache and is intentionally **not deployed to real Cube**.
- **Physical RC receiver check: BLOCKED**, no RC_CHANNELS observed in previous 7 s sampling; RC/steering/motor command tests not performed. Model/firmware version unknown.

## Clean-code and repeatability audit addendum
- Python `ruff check --select F401,F841 scripts/sim_bench.py scripts/test_rc_takeover_pty.py`: **All checks passed** after removing unused `math`, `CommandLong`, `MissionItem` imports and obsolete `ROS_SETUP` constant from the older fixture. Source regression after cleanup: RC PTY PASS and original PTY bench PASS (both exit 0).
- This workspace **is not a Git worktree**. For future source rollback a verified-current-state backup was created at `backups/rc_takeover_verified_software_20261009.tar.gz` (7 software files) with hash manifest `backups/rc_takeover_verified_software_20261009.sha256`. This snapshot is **not** a copy of pre-project source and does not modify Cube firmware; restoring it would restore this tested state.
- No old runtime PX4 bridge process was restarted or killed. The work remains a validated source+software-simulator increment rather than an enabled hardware motion system.
