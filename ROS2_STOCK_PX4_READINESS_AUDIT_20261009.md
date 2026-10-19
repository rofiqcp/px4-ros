# ROS2 MID-360S → PX4 Stock: Implementation and Runtime Readiness Audit
Date: 2026-10-09 (Asia/Jakarta) | Host: sirobo | ROS 2 Humble

## Executive status

**ROS2 advisory software: built, unit/integration tested and running in production ROS_DOMAIN_ID=0, with hardware motion locked.**

**Full autonomous rover/motion readiness: BLOCKED.** Cube firmware remains unmodified; physical RC authority, PX4 Rover configuration, LiDAR extrinsic calibration, live LiDAR data and localization are not verified. It is not safe to arm or drive based on this ROS2 stack.

## Workspaces, build and topology
- LiDAR/Mapping: `/media/sirobo/HDD/slam3d-livox-mid360s`
- Bridge/advisory: `/media/sirobo/HDD/pixhawk`
- Shared colcon build/install/log: `/home/sirobo/.cache/mid360_pixhawk_ros`
- Unified: `cd /media/sirobo/HDD/pixhawk && ./build.sh && source env.sh`
- Read-only stack launch: `ros2 launch pixhawk_avoidance_adapter stock_ros_stack.launch.py`. This deliberately does **not** launch another serial Pixhawk bridge, does not arm, does not send motor PWM or PX4 MAVLink control.
- Pipeline: MID-360S LiDAR → `/livox/lidar` → `obstacle_advisory_node` → `/pixhawk/avoidance/advisory` → `stock_px4_supervisor` → `/pixhawk/stock/{status,advisory}`; original `/pixhawk/status`, `/pixhawk/imu` and `/pixhawk/rover/feedback` remain read-only inputs. FAST-LIO runs separately with `/lio/odometry` for mapping; obstacle perception performs conservative base-frame sector sampling (not full trajectory prediction), no automatic navigation authority.
- Monitoring: `/pixhawk/avoidance/status` textual reason and `/pixhawk/avoidance/diagnostics` JSON diagnostics, 10 Hz. Supervisor status/advisory also 10 Hz.
- Mapping UI `localhost:5001`, local web tuning `localhost:5002`, Foxglove bridge on `ws://127.0.0.1:8765`.
- C++ source: `src/pixhawk_avoidance_adapter/src/obstacle_advisory_node.cpp`, `src/stock_px4_supervisor/include/stock_px4_supervisor/safety.hpp`, `src/stock_px4_supervisor/src/stock_px4_supervisor.cpp`. MAVLink stock bridge source kept separate.
- Production-safe default configs: `src/pixhawk_avoidance_adapter/config/avoidance.yaml` and `src/stock_px4_supervisor/config/supervisor.yaml`; `simulation_only=false`, `calibration_verified=false`, `rc_priority_verified=false`, steering feedback not verified. No hardware command path in these ROS nodes.

## Engineering hardening
1. LiDAR parser validates x/y/z float32 layout, count, buffer bounds, height/width, row stride, endian, finite numbers, range and height above base reference.
2. Validates unique monotonic cloud timestamps, not merely periodic publishing, plus cloud freshness and TF calibration/age/quaternion/translation checks.
3. Ground band and 3D point sampling ignore points outside calibrated min/max heights; this is a **coarse height filter**, not a complete ground-segmentation or slope-reconstruction algorithm.
4. Unknown front coverage or insufficient finite obstacle points is **STOP/INVALID**, never OPEN; unseen side is unknown, not a bypass.
5. A front obstacle always produces STOP advisory even when a left/right yaw suggestion is computed; there is **no verified collision-free Ackermann trajectory** to authorize movement.
6. New ROS2 message field `source_scan_seq` identifies a validated **new unique cloud**. Supervisor uses it to prevent repeated timer publications from satisfying a multi-frame clearance requirement; zero source sequence is invalid/fail-closed.
7. Safety supervisor honors upstream perception `stop`, preserves hazard latch, models dynamic braking, E-STOP > RC priority, requires distinct clear scans + one-shot operator approval before simulated CLEAR. Simulation operator approval is rising-edge triggered, not continuous from stale heartbeat.
8. Supervisor remains `hardware_command_enabled=false`, `stock_firmware_interface=UNPROVEN`; no physical PX4 command output. Existing Cube bridge remains running from its previous read-only binary.
9. Added testable isolated launch and read-only ROS doctor at `tools/ros_readiness.py`. Simulator tests use unique ROS_DOMAIN_ID and pseudo-TTY only.

## Software test evidence (simulation only)
- Unified colcon: 12 packages build PASS, verify latest `/home/sirobo/.cache/mid360_pixhawk_ros/ros_readiness_final_build.log`.
- `scripts/test_ros_perception_faults.py`: **11/11 PASS**, startup STOP, valid clear with distinct scan IDs, duplicate-stamp block, malformed buffer block, ground rejection, unknown-front guard, unknown-side guard, suggested left/right while STOP, front STOP, stale timeout. Log `/home/sirobo/.cache/mid360_pixhawk_ros/ros_fault_fixture.log`.
- `scripts/test_ros_chain_sim.py`: **PASS**, perception-to-supervisor real ROS topic test in isolated domain 94, clear ACK, upstream STOP veto, no auto resume after obstacle, operator ack + distinct scans, manual RC priority, MAVLink disconnection. Log `/home/sirobo/.cache/mid360_pixhawk_ros/ros_chain_integration.log`.
- `stock_px4_supervisor` CTest and unit assertions: PASS, read `/home/sirobo/.cache/mid360_pixhawk_ros/ros_readiness_ctest.log`.
- `cppcheck` and Ruff: no code errors/dead-import problems after cleanup; C++ constructor-body initialization suggestions are related to runtime ROS parameter declaration, not unused/unsafe control code.
- Previous stock PX4 pseudo-TTY bench and RC takeover fixture already PASS; **not** proof of real installed Cube firmware, steering, actuators or collision-free motion.

## Live runtime, as measured on 2026-10-09

Doctor report: `/media/sirobo/HDD/pixhawk/results/ros_readiness_20261009.json`

During an 8-second read-only ROS graph test:
- `/pixhawk/avoidance/diagnostics`: **80 messages, 10.0 Hz**.
- `/pixhawk/stock/status`: **81 messages, 10.0 Hz**.
- `/pixhawk/status`: **7 messages (~0.91 Hz)**, hardware MAVLink link connected.
- `/livox/lidar`, `/livox/imu`, `/lio/odometry`: **0 messages received in observation window**. The ROS nodes/drivers may be running, but data were NOT streaming during the sample. Do not call this LiDAR ready.
- Extrinsics currently have `status: uncalibrated` and zero baseline. Do not mark them verified without measured calibration.
- PX4 status: generic/unconfigured airframe, localization invalid; RC authority unverified. Supervisor reports `UNKNOWN / RC_PRIORITY_UNVERIFIED` and perception `CLOUD_STALE`. These are appropriate safety states.
- Live Cube bridge permission readbacks: motion=false, parameter_write=false, mode_change=false, mission_write=false. Firmware remains unchanged.

## Operation / verification
```bash
cd /media/sirobo/HDD/pixhawk
./build.sh
source env.sh
# Only if stock stack is not already running (avoid duplicate publishers)
ros2 launch pixhawk_avoidance_adapter stock_ros_stack.launch.py

# Another terminal; exclusively read-only diagnostics
source /media/sirobo/HDD/pixhawk/env.sh
ros2 topic echo --once /pixhawk/avoidance/diagnostics
ros2 topic echo --once /pixhawk/stock/status
python3 /media/sirobo/HDD/pixhawk/tools/ros_readiness.py --duration 8
```
- In a healthy, separately authorized sensor-only bench test: confirm LiDAR/IMU/LIO stream frequency, uniquely stamped clouds and TF coverage; calibrate LiDAR extrinsics and verify actual base_frame / livox_frame transform. Do not toggle `calibration_verified` to true on assumptions.
- Operator RC wiring must be RC receiver → PX4 independently; do not put receiver behind ROS2. Do not change PX4 mission, firmware or RC priority settings without version-specific and physical-safe verification.
- If ROS2 or sensor input fails, advice becomes STOP or UNKNOWN; **this cannot itself actuate a disconnected vehicle**, therefore independent physical E-STOP and watchdog are required to guarantee physical stopping.
- No AUTO mixing or OFFBOARD authority has been deployed on Cube. ROS2 publisher `/pixhawk/stock/advisory` is diagnostics, not actuator commands.

## Rollback
- Backups of pre-change ROS sources: `backups/ros_runtime_20261009/`.
- Stop only the added `ros2 launch pixhawk_avoidance_adapter stock_ros_stack.launch.py` process and its two child nodes. Do not restart/kill the existing Pixhawk bridge, Livox driver, FAST-LIO, Foxglove or mapping dashboard to roll back the new advisory stack.
- The updated `AvoidanceAdvisory.msg` adds a field. Rebuild all affected packages under the shared colcon cache as a set; do not mix binaries built against old and new interface schema. Old source backup is for investigation; restoring it without rebuilding the interface and dependent packages would be incorrect.

## Remaining blockers and validation requirements
- **NO LIVE LIDAR CLOUD/IMU/ODOM DATA** observed. Confirm MID-360S power and IP network topology without forcing firmware or writes.
- **UNVERIFIED EXTRINSICS** (zero baseline) and no certified ground segmentation / full trajectory clearance.
- **UNVERIFIED PX4 AIRFRAME/RC/MISSION / REAL AUTOPILOT_VERSION** and invalid horizontal estimator.
- **NO VERIFIED PHYSICAL STOP ON MINI-PC LOSS**. Independent hazard latch/watchdog and E-STOP must be implemented and approved before commanding hardware.
- CPU/RAM/load under actual real-time point-cloud rates, dropped frames, full 3D mapping, obstacle decision latency and PX4 SITL matched to Cube firmware are still to be measured under appropriate conditions.
- Do not mark software fixture PASS as hardware PASS or label this rover autonomous-ready.

## Final post-build evidence and replay
- Latest unified build after code cleanup: `/home/sirobo/.cache/mid360_pixhawk_ros/ros_readiness_final_build.log`, **12/12 PASS**.
- CTest `stock_px4_supervisor` **1/1 PASS**, standalone 245 C++ safety assertions PASS; log `/home/sirobo/.cache/mid360_pixhawk_ros/ros_final_ctest.log`.
- Perception 11 fault scenarios PASS and topic-level perception→supervisor chain PASS; `ros_fault_fixture.log` and `ros_chain_integration.log`, respectively.
- Clean-source Ruff F401/F841 PASS for the new fixtures and the older synthetic cloud script; C++ cppcheck no code errors, only constructor runtime-parameter initializer performance suggestions.
- New live 5.31-second MCAP bag recorded **221 messages** (54 perception diagnostics, 54 perception advisory, 54 stock status, 54 stock advisory, 5 PX4 status) at `/media/sirobo/HDD/slam3d-livox-mid360s/results/stock_px4_mcap/ros_final_diagnostics_20261009`. Metadata validation and playback in isolated ROS_DOMAIN_ID=96 **PASS**, never replay into live hardware domain. Logs: `ros_final_mcap_info.log` and `ros_final_mcap_replay.log`.
- Browser Playwright on tuning localhost:5002 PASS: ROS supervisor online, safety UNKNOWN / RC_PRIORITY_UNVERIFIED, no JS errors, no mobile overflow at 390px. Screenshots `results/stock_px4_browser/ros_final_desktop.png` and `ros_final_mobile.png`.
- Live stack running (start log /home/sirobo/.ros/log/2026-10-09-02-23-48-737079-sirobo-439032) with the separate previous Cube serial bridge untouched. This is software monitoring, NOT Rover hardware-validated autonomy.
- Launch and telemetry tested while MID-360S topic present but no actual LiDAR point cloud or IMU streaming. Result is appropriately fail-closed. Do not bypass the uncalibrated extrinsics gate simply to make dashboards green.
