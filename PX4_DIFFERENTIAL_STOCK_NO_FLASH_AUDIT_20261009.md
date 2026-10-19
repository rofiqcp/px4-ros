# PX4 Differential Drive / ROS2 Integration — Stock Firmware, No Flash
**2026-10-09, host: sirobo. Implementation: ROS2 differential monitoring and staging; hardware PX4 differential activation: BLOCKED.**

## Hard boundaries
- **NO Cube firmware flashing, firmware update/replacement, bootloader access or parameter writes**; motor arming remains forbidden.
- Original physical Pixhawk bridge retains read-only configuration: enable_motion_commands=false, allow_parameter_writes=false, allow_mode_changes=false, allow_mission_writes=false. Existing serial bridge /dev/ttyACM0 was not restarted.
- Differential preview topic does **not** publish motor topics, actuator topics, PX4 offboard setpoints or MAVLink commands. No PWM injection, RC spoofing or odometry manipulation.
- The original request was to use *two independently driven wheels, left and right* (Differential/Skid-Steer). This is **not Ackermann**.

## Cube firmware and parameter audit
Original 732 unique parameters backed up in `backups/cube_px4_20261008_213146_complete.json` (SHA256 copied to staged profile).
| Evidence | Observed | Decision |
|---|---|---|
| SYS_AUTOSTART | 0 (live read-only PARAM_VALUE success) | No Rover frame configured |
| MAV_TYPE | 0 (live read-only PARAM_VALUE success) | Generic, not verified Differential Rover |
| RD_* params | **0 of 732** | Differential Rover module support **unproven** |
| CA_* params | **0 of 732** | New-style control allocation and L/R output function mapping absent |
| RO_* params | only RO_SPEED_LIM=-1 | No complete rover-specific tuning profile |
| RC_CHAN_CNT | 0 in backup | Physical receiver, channels and mode-switch unverified |
| RC_MAP_* | all 0 in backup | No valid manual RC calibration/mode switch mapping |
| PWM_MAIN_FUNC1 | 404 | Meaning/motor mapping unknown; do NOT assign output functions by guessing |
| PWM_MAIN_FUNC2..8, PWM_AUX_FUNC1..6 | all 0 | No validated dual-motor left/right mapping |
| COM_RC_IN_MODE | 3 | User-requested direct RC priority not verified by firmware + receiver tests |
| RD_WHEEL_TRACK read-only PARAM_GET | **timed out** awaiting PARAM_VALUE | Supports inability to confirm parameter; not proof of module identity |
| AUTOPILOT_VERSION | unknown (old physical bridge does not publish it) | Do not assume older/newer firmware support |
| Cube armed | false in live heartbeat | no physical motion testing |

**PX4 modern documentation** says a special rover firmware variant with the differential module is needed. Only on a proven compatible rover build is Generic Rover Differential frame associated with **SYS_AUTOSTART=50000**. The current stock firmware does not advertise RD_WHEEL_TRACK; therefore we **did not send SYS_AUTOSTART=50000** or overwrite parameters.
References:
- https://docs.px4.io/main/en/config_rover/basic_setup
- https://docs.px4.io/main/en/config_rover/differential.html
- https://docs.px4.io/main/en/config_rover/
- https://docs.px4.io/main/en/flight_modes_rover/differential.html
- https://docs.px4.io/main/en/flight_modes/offboard

**Consequence of the mandatory no-flash constraint:** If the installed Cube genuinely lacks rover modules, neither setting MAV_TYPE nor selecting SYS_AUTOSTART alone can give a functioning PX4 differential Rover. Keep firmware stock; an independently safety-gated external differential motor controller is a separate architectural alternative requiring approval.

## Implemented ROS2 differential staging and monitoring
1. C++ differential math: `src/stock_px4_supervisor/include/stock_px4_supervisor/differential.hpp`.
2. Differential advisory ROS node: `src/stock_px4_supervisor/src/differential_advisor.cpp` subscribes `/pixhawk/stock/{status,advisory}` and publishes JSON `/pixhawk/differential/diagnostics` at 10 Hz, containing wheel velocity previews, geometry status, and reason for zero command. **NEVER** connects to the Cube.
3. Differential preview formulas in FLU coordinate convention:
   - `v_left = v_forward - yaw_radps * wheel_track_m/2`
   - `v_right = v_forward + yaw_radps * wheel_track_m/2`
   - Uniform wheel speed saturation preserves curvature; zero/NaN/invalid width/invalid upstream advisory returns zero.
4. Pure C++ supervisor safety uses differential geometry, **not Ackermann minimum turning radius or steering angle**. It allows in-place-turn suggestion *only* when measured wheel track is verified, and still returns **AVOID_PROPOSED/STOP** (not a motor command).
5. `src/stock_px4_supervisor/config/differential.yaml`: `simulation_only=false`, `wheel_track_verified=false`, `wheel_track_m=0.0` (intentionally UNKNOWN), `wheel_speed_limit_mps=.35` (conservative simulation ceiling, not calibrated vehicle limit).
6. `src/stock_px4_supervisor/config/supervisor.yaml`: `differential_geometry_verified=false`, `differential_wheel_track_m=0.0`, `max_wheel_speed_mps=.35`. No fake wheel dimension.
7. `src/pixhawk_avoidance_adapter/launch/stock_ros_stack.launch.py` launches obstacle perception, stock safety supervisor and differential advisor. Existing LiDAR mapping, FAST-LIO, Foxglove and separate Cube serial bridge are NOT duplicated.
8. Local-only staged Cube plan: `config/px4_differential_staged.yaml`, generated by `tools/differential_px4_profile.py`. It carries candidate `SYS_AUTOSTART=50000` **only as information contingent on firmware verification**, with `writes_to_cube:false`, and lists every BLOCKER. Preflight JSON in `results/px4_differential_preflight.json`.
9. Staged web tuning is updated to `wheel_track_m` and `max_wheel_speed_mps`, replacing the obsolete Ackermann wheelbase/turn-radius/steering fields. The web server accepts `wheel_track_m=0` as UNKNOWN, and permits Save YAML, Apply to Simulation profile, and Rollback without applying anything to Cube. The live simulator does not hot-reload staged YAML.

## Software verification (separate from physical hardware)
| Test | Result | Evidence |
|---|---|---|
| Unified colcon (both workspaces) | **12/12 PASS** | `/home/sirobo/.cache/mid360_pixhawk_ros/differential_supervisor_final_build.log` |
| CTest stock safety + differential | **2/2 PASS** | `/home/sirobo/.cache/mid360_pixhawk_ros/differential_supervisor_final_ctest.log` |
| Isolated ROS2 topic-level differential fixture | **PASS** | `/home/sirobo/.cache/mid360_pixhawk_ros/differential_ros_sim_test.log` |
| Differential fixture checks | startup zero, left/right kinematics, obstacle STOP, RC manual, invalid advisory, absent scan ID, timeout zero | Test source `scripts/test_differential_ros.py` |
| Python Ruff/compile | **PASS** | Differential files checked |
| Live ROS2 graph | **Three nodes active** | obstacle_advisory_node, stock_px4_supervisor, differential_advisor |
| Live differential status | **0.0 / 0.0 m/s, preview_valid=false, wheel_track_verified=false, hardware_command_enabled=false** | Read-only `/pixhawk/differential/diagnostics` observation |
| Web Playwright desktop/mobile | **PASS** | `slam3d-livox-mid360s/results/stock_px4_browser/differential_px4_desktop.png` and `differential_px4_mobile.png` |
| Web staging Save / Apply to Simulation / Rollback | **PASS** | Browser assertions; no PX4 writes |
| Physical motor response/RC switch/mission AUTO | **NOT TESTED** | Cube still unconfigured and unarmed |
| Version-matched Rover SITL | **NOT TESTED** | Installed AUTOPILOT_VERSION unknown |

The current MID-360S/FAST-LIO streams and localization were previously observed missing/invalid. Therefore `ROS motion preview valid=false` in production is **correct fail-closed behavior**; do not interpret the live node as permission to drive.

## Required hardware configuration before any physical parameter change
1. Identify exact firmware with `AUTOPILOT_VERSION` through the existing single-owner serial bridge and confirm rover-differential module exists; no flash.
2. Physically measure **wheel track centre-to-centre in metres**, wheel radius and actual motor rotation directions. Identify ESC model and command input protocols for each wheel; never infer from one PWM_MAIN_FUNC1=404.
3. Physically verify independent receiver-to-PX4 connection, RC channel calibration, RC manual mode-switch and fail-safe. Emergency motor-enable hard cutout must be proven in safe bench.
4. Map left/right outputs to actual pins only after exact installed-firmware output-function documentation confirms them. If no relevant actuator functions/rover modules exist, **BLOCK** and choose an external safety controller, not parameter hacks.
5. If compatible firmware is eventually proven, make complete parameter backup, calculate diff against desired values, apply only an explicitly approved disarmed change set, read back and compare, then reboot/confirm airframe with drivetrain physically disconnected. **No part of that workflow was executed in this audit.**
6. Establish valid localization and real LiDAR extrinsic calibration before mission/offboard; continue to separate mission-preserved vs mission-running vs safe-motion.

## Rollback
- To disable added ROS differential monitor, stop the `stock_ros_stack.launch.py` launch; old Pixhawk physical serial bridge remains unaffected.
- Source-only snapshot from before differential code: `backups/ros_advisory_release_20261009.tar.gz`. Rebuild all affected ROS packages together if restoring that source. Do not restore pre-change files without matching interface versions.
- Web backups: `slam3d-livox-mid360s/foxglove/stock_px4_tuning.ackermann_backup_20261009.html` and `tools/stock_px4_tuning_server.ackermann_backup_20261009.py` are for source rollback only; original UI must not be treated as current differential tuning.
- Cube parameter/mission rollback: **NOT REQUIRED**, because this task wrote **zero** parameters or missions and left installed firmware unchanged.
