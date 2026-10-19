# STOCK PX4 NO-FLASH IMPLEMENTATION AUDIT
Tanggal: 2026-10-09 WIB | Host: sirobo | ROS 2 Humble
Status: **PARTIAL SOFTWARE IMPLEMENTATION / HARDWARE AUTONOMY BLOCKED**.
This is a safety engineering record, NOT authorization to operate the rover.

## 0. Non-negotiable hardware boundary
- No firmware flashing, upgrade, downgrade, upload, bootloader changes or PX4 source patch has been applied to the Cube.
- No arm/disarm, actuator, motor, mission-write, parameter-write, or flight-mode-change command was sent to the real Cube by this task.
- Port /dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v2.x_0-if00 was already owned by the ROS2 pixhawk bridge. We did not open a competing serial reader or restart that bridge.
- The ROS2 stock_px4_supervisor never imports MAVLink TX and has no mechanism to actuate the controller. The current bridge's custom TUNNEL TX is disabled by its default launch configuration.
- E-STOP must be a proven physical independent circuit. Software advisory and RC are not a substitute.

## 1. Real firmware and platform evidence
| Field | Current evidence | Assessment |
|---|---|---|
| USB ID | 26ac:0011 / 3D Robotics PX4 FMU v2.x; /dev/ttyACM0 | OBSERVED |
| MAVLink | HEARTBEAT: autopilot 12 PX4, sysid 1; live telemetry | OBSERVED |
| AUTOPILOT_VERSION.flight_sw_version | **NOT OBTAINED**: running bridge does not decode/publish 148; existing port is exclusively in use | UNKNOWN — exact version MUST be obtained before version-specific SITL/feature claims |
| Actual git SHA / board hardware version / capabilities bitmap | Not captured | UNKNOWN |
| Parameter snapshot | backups/cube_px4_20261008_213146_complete.json, 732/732 unique, checked by previous audit; no new hardware write | OBSERVED BACKUP |
| COM_OBS_AVOID | Absent from 732 parameter snapshot | NOT ADVERTISED; native interface unproven |
| CP_DIST | Absent | NOT ADVERTISED |
| MAV_FWDEXTSP | 1 | CONFIG VALUE; not proof of waypoint/yaw control |
| COM_OF_LOSS_T | 1 | CONFIG VALUE; failsafe not physically validated |
| COM_OBL_RC_ACT | 0 | CONFIG VALUE; semantics MUST be verified against exact installed firmware |
| COM_RC_IN_MODE | 3 | CONFIG VALUE; RC semantics unverified |
| COM_RC_OVERRIDE | 1 | CONFIG VALUE; actual manual takeover not tested |
| SYS_AUTOSTART | 0 | NOT CONFIGURED |
| MAV_TYPE | 0 | GENERIC, NOT VALIDATED ACKERMANN |
| Vehicle position | snapshot EKF flags 37; local_valid=false, GPS invalid; horizontal accuracy ~1377 m | INVALID FOR NAVIGATION |
| Servo / RC | bridge decodes RC_CHANNELS / SERVO_OUTPUT_RAW and VFR_HUD; servo PWM is **not** a measured steering angle | TELEMETRY ONLY |
| Mission | read-only mission backup had 0 mission and fence items; rally timed out | MISSION STORAGE NOT POPULATED IN BACKUP |

No hardware motion, AUTO Rover, safe steering, obstacle latch after companion loss, or firmware-aware command-support claim can be made using these data.

## 2. PX4 stock alternatives / evidence decision table
| Path | Maintain AUTO? | v/yaw adjustment? | Mission preserved? | Hotplug safety | Evidence verdict |
|---|---|---|---|---|---|
| A: native PX4 mission avoidance | Possible in **some older PX4 versions**, only through proper path planning support; not universal / not proven for rover | Not shown for this Cube | Yes in principle | Requires PX4 failsafe proof | **BLOCKED/UNKNOWN:** COM_OBS_AVOID absent; PX4 v1.13-1.14 docs do not establish installed firmware support |
| B: mission-aware MAVLink route / change-speed | High-level route amendments, not 50 Hz yaw mixing | Speed and waypoint APIs version-specific; no active-mission test | Can be preserved with snapshots and ACK rollback | Hazard latch requires independent mechanism | **DESIGN ONLY**; mission backup has 0 items; no permission for writes to real Cube |
| C: AUTO -> stock supported control mode -> AUTO | **No**, not continuous AUTO mixing | Setpoint semantics and RC/offboard loss behavior unverified on this rover/firmware | Mission may remain stored; index/rejoin not proven | Mode loss must HOLD, not blind AUTO resume | **SIMULATOR VALIDATION NEEDED**, no real mode changes |
| D: independent motor enable / E-STOP watchdog / certified range safety | PX4 can keep mission; physical motion inhibited externally when unsafe | Independent STOP, not a PX4 yaw mixer | Yes | Independent fail-closed hazard circuit necessary | **ARCHITECTURE PROPOSAL ONLY**, hardware modifications require separate approval |

**Selected safe current implementation**: stock-firmware-agnostic advisory-only ROS2 safety supervisor + existing PX4 read-only telemetry bridge + separate local tuning panel. **No enabled stock PX4 motion adapter exists yet**, because actual firmware capabilities and rover readiness have not been demonstrated. The current solution does NOT meet the vehicle-level obstacle-avoidance acceptance condition.
Reference PX4 older docs: https://docs.px4.io/v1.14/en/computer_vision/path_planning_interface ; https://docs.px4.io/v1.14/en/computer_vision/obstacle_avoidance ; latest MAVLink mission spec: https://mavlink.io/en/services/mission.html . Verify version before treating those docs as applicable.

## 3. Implemented source
- pixhawk/src/stock_px4_supervisor/include/stock_px4_supervisor/safety.hpp: deterministic fail-closed state engine, latched STOP, stale-data watchdog, E-STOP > RC, no automatic fault clear, 3 distinct ROS advisory frames for clear confirmation (plus explicit operator ack), dynamic stop distance (v²/(2a) + reaction*v + safety margin), side hysteresis, Ackermann yaw rate bound. AVOID_PROPOSED does **not** provide motion authority.
- pixhawk/src/stock_px4_supervisor/src/stock_px4_supervisor.cpp: subscribe existing /pixhawk/status, /pixhawk/rover/feedback, /pixhawk/imu, /pixhawk/avoidance/advisory; publish /pixhawk/stock/status and /pixhawk/stock/advisory. RC takeover, steering sensor and PX4 AUTO mode **cannot** be validated from current message schema and remain locked out. Simulation inputs explicitly gated to simulation_only=true; hardware default false. Never opens serial.
- pixhawk/src/stock_px4_supervisor/test/test_safety.cpp: C++ fault injection state regression.
- pixhawk/src/pixhawk_avoidance_adapter: existing cloud detector; calibration_verified=false by default. No trajectory collision proof; NO stock firmware command support.
- pixhawk/src/pixhawk_bridge: pre-existing MAVLink telemetry/mission code; custom 42001 TUNNEL patch is not a stock PX4 decoder, TX must stay false.
- slam3d-livox-mid360s/foxglove/stock_px4_tuning.html, tools/stock_px4_tuning_server.py: independent loopback-only 5002 web staging UI. Does not send command to PX4. Save YAML, simulator staging profile, rollback staged config; MCAP through web not activated.
- Existing mapping dashboard 5001 and Foxglove bridge 8765 remain separate and untouched.

## 4. Build and test evidence
- Unified colcon build: `cd /media/sirobo/HDD/pixhawk && ./build.sh && source env.sh`.
- Shared cache: /home/sirobo/.cache/mid360_pixhawk_ros; 12 ROS2 packages (previous 11 plus stock_px4_supervisor).
- Build log: /home/sirobo/.cache/mid360_pixhawk_ros/stock_full_build.log. **PASS 12/12** (2026-10-09).
- Standalone C++ test reported **242 passing assertions** including RC manual, physical E-STOP precedence in simulated logic, timeout, dynamic braking, left/right wall choice, obstacle latch, 100 synthetic reconnects. Log from remote tool invocation; `colcon test` CTest reported **1 test / 0 failures**.
- PX4 bridge remained active, telemetry visible. Separate stock supervisor node started in hardware read-only mode and publishes UNKNOWN/LATCHED when RC/mode/steering or calibration cannot be proven. This is intentional.
- Playwright visited local tuning at localhost:5002, 19 numeric inputs, screenshot desktop 1440 x 960 and mobile 390 x 844, no mobile horizontal overflow. Screenshots: slam3d-livox-mid360s/results/stock_px4_browser/{desktop,mobile}.png.
- Python web server opened successfully on local port 5002. Existing mapping localhost:5001 remained HTTP 200.
- Simulation-only YAML file save was tested using Playwright. Apply to Simulation **writes a staging profile** and does NOT hot-apply a running simulator. Rollback needs a previous saved version. No PX4 updates.
- ROS2 synthetic LiDAR cloud test **PASS** (CLEAR, AVOID, STOP, CLOUD_STALE), via isolated ROS_DOMAIN_ID=78 with no FCU access. Log: /home/sirobo/.cache/mid360_pixhawk_ros/stock_cloud_fixture_test.log.
- MAVLink PTY bridge simulator **PASS**, includes simulated mission download/upload of two items and ACK=0, param read/set, mode ACK, 10 Hz local/global setpoints, cancellation. Log: /home/sirobo/.cache/mid360_pixhawk_ros/stock_pty_sim.log. PTY simulator does not represent actual installed hardware firmware.
- MCAP CLI recording **PASS**: 111 messages over 5.2 sec (53 stock status, 53 stock advisory, 5 original PX4 status), metadata at slam3d-livox-mid360s/results/stock_px4_mcap/read_only_20261009/metadata.yaml. Replayed in isolated ROS_DOMAIN_ID=77, ros2 bag play exit 0. The browser Start MCAP action remains disabled.
- Version-matched PX4 SITL, physical servo response, active-mission pause/rejoin: **NOT TESTED**. Earlier experimental v1.17 SITL does not prove compatibility with the unknown installed firmware.
- CPU/RAM 6-second *idle/available-load sample*, not full motion load: supervisor process CPU p95 6.6%, RAM p95 26.15 MiB; FAST-LIO CPU p95 13.2%, RAM 47.87 MiB; pixhawk bridge CPU p95 6.6%, RAM 21.29 MiB; tuning web ~19.62 MiB RAM. Sample: /home/sirobo/.cache/mid360_pixhawk_ros/stock_resource_profile.json. Packet loss, end-to-end perception latency p95, dropped frames under moving rover: **NOT MEASURED**.

## 5. Hotplug/fault table
| Scenario | Tested evidence | Safety consequence |
|---|---|---|
| Companion absent at boot | Unit STOP latch; physical PX4 mission read-only storage | Mission can remain stored; safe motion NOT proven |
| Companion joins | Unit model; no physical motion | No auto arming or resume |
| Front obstacle | Unit STOP / dynamic stopping calculation | Advisory only, not actual motor brake |
| Disconnect while CLEAR | Simulated disconnect | No blanket guarantee of safe autonomous driving |
| Disconnect while STOP/AVOID | Unit hazard latch; read-only node cannot make PX4 latch | **BLOCKED: independent hazard watchdog needed** |
| Reconnect while hazard unknown | Requires clear observations + operator acknowledge in software | Hardware must still have independent protection |
| RC manual takeover | Unit precedence proof only | Physical RC takeover NOT TESTED |
| 0.3/1/5/30 sec MAVLink loss | C++ injected faults | Software STOP, physical STOP not verified |
| Invalid EKF, IMU, TF, steering sensor | Unit block | Correct conservative UNKNOWN state |
| Mission index consistency / resume | No mission stored in backup, no active physical mission | **NOT TESTED** |

## 6. Deployment and rollback (software only)
1. Do not flash Cube, alter PX4 parameters/modes/missions, arm or move hardware for this workflow.
2. Confirm Cube disarmed, custom TUNNEL TX false, bridge permissions false, external E-STOP functional.
3. `cd /media/sirobo/HDD/pixhawk && ./build.sh && source env.sh`.
4. Run only read-only diagnostics: `ros2 run stock_px4_supervisor stock_px4_supervisor --ros-args --params-file src/stock_px4_supervisor/config/supervisor.yaml`.
5. Foxglove diagnostics: subscribe /pixhawk/stock/status, /pixhawk/stock/advisory, /pixhawk/status, /pixhawk/rover/feedback. Tuning staging: `python3 /media/sirobo/HDD/slam3d-livox-mid360s/tools/stock_px4_tuning_server.py` then http://127.0.0.1:5002.
6. STOP supervisor or tuning server for rollback; pre-existing pixhawk bridge + MID-360S mapping launch remain unchanged. Staged YAML can be rolled back using prior saved profile.
7. Do not enable stock motion until (a) precise AUTOPILOT_VERSION captured safely through single-reader bridge, (b) matched Rover SITL and interface ACK+actuator behavior proven, (c) airframe and horizontal estimator healthy, (d) RC and steering feedback calibrated, (e) independent hazard latch/physical E-STOP validated, and (f) separate hardware test authorization.

## 7. Still BLOCKED / next verifiable work
- Exact installed firmware AUTOPILOT_VERSION/version-git-sha and capabilities read-only instrumentation of existing single-owner serial bridge (no competing /dev/ttyACM reader). Service 512 can request message 148, but current parser does not expose its payload; changing live bridge needs a controlled restart.
- No COM_OBS_AVOID/CP_DIST in real backup and no proof Rover AUTO accepts avoidance/yaw mixing.
- No vehicle configuration, mission items, valid horizontal position, RC mode evidence, steering angle sensor, E-STOP/hazard-latch hardware.
- Temporary mission routing protocol, mission checksum/ACK/rollback and mode switch must be proven in exactly matching SITL before any hardware writes.
- No permission to modify physical actuator wiring, no actual brake/steer test, and no guarantee PX4 will stop on mini-PC loss. **Do not operate autonomously based on this prototype.**

## 8. PX4 official documentation cross-check
Official PX4 *current* documentation explicitly marks the Path Planning Interface and Mission Obstacle Avoidance **removed from PX4 v1.15 onward / unsupported and unmaintained**: https://docs.px4.io/main/en/computer_vision/path_planning_interface .
The legacy v1.14 documentation required `COM_OBS_AVOID=1`: https://docs.px4.io/v1.14/en/computer_vision/path_planning_interface . This parameter is absent in the actual 732-item Cube backup. Collision Prevention `CP_DIST` in the v1.15 documentation targets **multicopter Position mode**, not automatically Ackermann rover AUTO: https://docs.px4.io/v1.15/en/computer_vision/collision_prevention .
Therefore: neither sending `OBSTACLE_DISTANCE` nor `DISTANCE_SENSOR`, nor an unrecognized custom `TUNNEL` payload establishes rover AUTO yaw/speed mixing. Do not enable any motor command based on these messages. This audit deliberately does not invent firmware version, supported flight modes, or proof of hardware avoided obstacles.

## 9. Isolated ROS2 supervisor integration verification
- Fixture: `scripts/test_stock_supervisor_sim.py`, runs with `ROS_DOMAIN_ID=79` and `stock_px4_supervisor --ros-args -p simulation_only:=true` (no FCU serial access).
- Result: **STOCK_ROS2_SIM_INTEGRATION_PASS**, exit code 0; checked BOOT_LATCH, operator ACK + unique clearance observations → CLEAR, obstacle STOP, persistence until ACK, RC override, E-STOP over RC, MAVLink-loss UNKNOWN, and reconnect-without-autoresume. All received diagnostics reported `hardware_command_enabled=false`.
- Log: `/home/sirobo/.cache/mid360_pixhawk_ros/stock_ros2_sim_test.log` and companion node log. Isolated test companion process shut down after the fixture. It is not a PX4 version-matched SITL or physical RC test.

## 10. Live read-only MAVLink traffic observation
Existing `/pixhawk/status` cumulative counters were sampled over 6.4 seconds without writing FCU parameters: 2,177 RX packets (340.2 packets/s), 6 TX packets across the sampled interval, and 0 increment in bridge MAVLink parser errors. This is **traffic rate only**, not packet loss or ACK RTT proof. The bridge currently reports `enable_motion_commands=false`, `allow_parameter_writes=false`, `allow_mode_changes=false`, `allow_mission_writes=false`; two optional avoidance TX-related names were not set as live ROS2 parameters (source defaults false and no custom firmware capability confirmed). The currently active serial bridge binary originates from the earlier cache `/home/sirobo/.cache/pixhawk_ros`, while newly built components use the mandated unified cache. Replacing/restarting the bridge was deliberately deferred to avoid disrupting the single-owner serial link.

## 11. Follow-up: ROS takeover, mission preservation and offboard-loss (2026-10-09)
- User requirement clarified: PX4 owns mission waypoints, RC remains operator-priority, ROS temporarily contributes obstacle avoidance, and on ROS loss the mission should resume *when independently proven safe*.
- **Important constraint:** mission AUTO and external OFFBOARD control do not execute simultaneously. A transition OFFBOARD→AUTO is needed for mission execution to resume; PX4 stock offboard-loss failsafe actions cannot be assumed to include AUTO Mission.
- Cube backup settings: `COM_OF_LOSS_T=1`, `COM_OBL_RC_ACT=0`. Version-specific interpretation and actual behavior remain unverified; no Cube parameter was changed.
- Added dormant `pixhawk_bridge/offboard_velocity_vector` service (`pixhawk_interfaces/srv/VelocityVector.srv`) that emits generic MAVLink local-NED vx/vy vector with yaw/yaw_rate ignored for Ackermann. It is locked behind four default-false / constrained gates and was **not run on Cube**.
- Added a read-only MAVLink `AUTOPILOT_VERSION` parser and publisher in newly built `pixhawk_bridge`. The existing active physical-serial bridge was deliberately not restarted; real firmware version remains unknown. Do not infer version from experiments on PX4 1.17 source.
- New build log `/home/sirobo/.cache/mid360_pixhawk_ros/stock_mission_continuity_build.log`: **12/12 packages PASS**. After adding new message interface, isolated pseudo-serial regression `stock_vector_guard_pty.log` PASS, including default rejection of Rover velocity, simulated mission upload/download, ACK and setpoint streams. No actual rover velocity-mixing acceptance, automatic AUTO rejoin, or RC takeover physics has been demonstrated.
- Safety supervisor CTest rerun PASS, 1 test / 0 failures. Both existing Web mapping 5001 and local tuning 5002 returned HTTP 200.
- Detailed failover and mission continuity contract: `/media/sirobo/HDD/pixhawk/MISSION_CONTINUITY_FAILOVER_NO_FLASH_20261009.md`. Key blocker remains **independent safety watchdog and hazard clearance on companion loss**.

## 12. RC manual direct-control architecture and clean-code regression (2026-10-09)
Real RC control must be `Radio TX → paired RX → PX4 RCIN → MANUAL flight mode → actuators`, **not ROS → RC spoofing**. The bridge now has a sticky RC-manual-takeover latch in source, preventing new velocity / GOTO / mode commands after a PX4 MANUAL heartbeat while permitting disarm, passive telemetry and mission download. This implements ROS command suppression; PX4 remains solely responsible for physical RC. PX4 `COM_RC_IN_MODE=3` means first valid manual input is retained on versions matching current documentation, **not guaranteed RC priority**; do not claim physical precedence without testing actual receiver. Full audit: `RC_MANUAL_TAKEOVER_NO_FLASH_AUDIT_20261009.md`.
Regression: unified build 12/12 PASS, RC PTY simulator PASS (mission still 2 items, mode/velocity lock, RC persists after companion crash), previous MAVLink sim PASS, CTest 1 PASS, cppcheck no dead-code issues (2 constructor performance suggestions), Playwright desktop/mobile PASS. Hardware firmware unchanged; no FCU parameters/missions or motors were changed.
