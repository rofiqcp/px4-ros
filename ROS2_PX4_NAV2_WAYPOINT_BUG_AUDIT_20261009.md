# ROS2 + PX4 Differential v1.17 + Nav2 — Multi-Waypoint and Failure-Injection Audit
Date: 2026-10-09 | Machine: sirobo | ROS2 Humble
Scope: **SITL and sensor fixtures only. Physical Cube disarmed, USB bridge read-only.**

## Executive results

- **PX4 firmware v1.17.0 Differential SITL / Gazebo: 4 of 4 mission routes COMPLETED**, 2/3/4/5 waypoints. Every route had real PX4 `MISSION_ACK=0`, successful mission download coordinate readback, valid mission id, sequential `MISSION_ITEM_REACHED` messages, valid PX4 `mission_result` with `finished=True` and `failure=False`, and simulator-only ARM/DISARM.
- **ROS waypoint planner validation: 16 of 16 unit checks PASS**, 4 planned shapes, 12 rejection cases.
- **Nav2 ComputePathToPose: 4/4 action requests produced paths, 3/4 passed strict postplanning costmap safety checks**. One planned path crossed one currently-unknown map cell; **test rejected it**. Never connected these paths to motor control. A separate real obstacle detour using NavFn produced 74 poses, 5.556m versus 5.179m straight, 0 lethal/unknown cells, and off-map goal was rejected.
- **ROS LiDAR fault test: 11/11 PASS**; actual topic-level perception → safety supervisor chain PASS (RC takeover/STOP/ACK/connection-loss). **CTest stock safety + differential: 2/2 PASS**.
- **Unified ROS2 workspace build: 12/12 packages PASS** after changes.
- **Live Cube** is still **disarmed**; installed PX4 firmware is 1.17.0 but Rover module, RC channel, motor outputs, localization and extrinsic calibrations unverified. No real Cube mission, motor command, parameter writes, flashing or arming occurred.
- Full **autonomous avoidance or Nav2-to-PX4 online control is NOT verified**. Separate planning simulations must not be confused with closed-loop obstacle avoidance on physical hardware.

## 1. Verified PX4 Differential mission shapes

| Route | Nominal waypoint plan (N/E from simulation home) | Point count | Planned length | PX4 reached sequence | Actual odometry traveled | Net displacement | Result |
|---|---|---:|---:|---|---:|---:|---|
| line2 | straight | 2 | 3.60m | [0,1] | 3.177m | 3.136m | PASS |
| corner3 | 90-degree corner | 3 | 5.40m | [0,1,2] | 4.590m | 3.495m | PASS |
| square4 | closed rectangular loop | 4 | 7.20m | [0,1,2,3] | 5.717m | 0.484m | PASS |
| zigzag5 | five alternated lateral goals | 5 | 9.24m | [0,1,2,3,4] | 8.304m | 6.704m | PASS |

The plan lengths are nominal geometry; odometry traveled is an actual sample sum (not identical due to PX4 waypoint acceptance radii and turning). Each mission was uploaded through the real compiled ROS2 bridge process connected to a **temporary PTY** ↔ loopback UDP ↔ **actual PX4 v1.17 Differential Rover SITL/Gazebo**. Every completion was checked against the actual PX4 uORB `mission_result` and ROS mission events. Tests run solely in `ROS_DOMAIN_ID=105`, and do not open `/dev/ttyACM0`.

Logs: `results/planner_*_hardened.log`, JSON in `results/planner_waypoints_20261009/{line2,corner3,square4,zigzag5}.json`.
Tools: `tools/waypoint_planner.py`, `scripts/test_waypoint_planner_unit.py`, `scripts/test_px4_117_multiroute_sitl.py`.

## 2. Nav2 PCD-based fixture planner tests

- Tested real installed `nav2_planner/NavfnPlanner` using `/compute_path_to_pose` action and explicit `map` poses. No `NavigateToPose`, `FollowPath`, `cmd_vel`, PX4 mission or motors were invoked from Nav2.
- PCD map from Stage 4 fixture: Nav2 211x209 cells, resolution ~0.15m.
- 4 candidate point goals: NavFn returned valid path message (18/26/28/42 path poses after sensor-frame fix). Strict postplanning map validation found **3 safe paths** and 1 path with 1 unknown cell; the latter was invalidated.
- **Detour around a fixed obstacle: PASS**. Straight route 5.179m, crossing 5 lethal samples; NavFn 74-pose path 5.556m, 0 lethal/unknown samples.
- **Outside-map point: rejected** (ROS action status 6, no path).
- Detailed JSON `results/planner_waypoints_20261009/nav2_fixture_paths.json`; script `scripts/test_nav2_fixture_multigoal.py`; logs `results/nav2_planner_detour_test.log` and `results/nav2_planner_stability_repeat.log`.
- These are **global map planners**, not actual dynamic obstacle-avoidance execution. The 1-unknown-cell candidate is an open P1: enforce safety validation against the latest costmap and trigger replan/STOP before any Nav2 path can be translated into PX4 commands. No path-to-PX4 actuation exists today.

## 3. Bugs found, fix, and regression status

### P0: Simulator Nav2 sensor raytrace originates from fixed (0,0) — FIXED IN FIXTURE
**Symptom:** Old Nav2 costmap rolling window emitted **115** `Sensor origin at (0,0,0) is out of map bounds` warnings. Sensor cloud `/lio/cloud_registered` was already transformed into `lio_odom` and its origin stayed at global (0,0) as the vehicle moved. The local rolling costmap could not raytrace.
**Fix:** `slam3d_relocalizer/src/map_fixture.cpp` now emits a second, **sensor-relative** cloud `/lio/cloud_sensor` in `livox_frame`, computed from each pose via inverse SE(3). The pre-existing registered point cloud remains available for relocalization. Both Nav2 **fixture** obstacle layers now subscribe to `/lio/cloud_sensor`. **After rebuild and restart warning count = 0**. This is in simulator/fixture only, not a change to live LiDAR wiring. Real Nav2 config already uses sensor-relative `/livox/lidar`.

### P1: Costmap clocks inconsistent with launch defaults — FIXED
Fixture `nav2_mid360s_fixture.yaml` and live-default `nav2_mid360s.yaml` both originally had `use_sim_time:true` in local/global costmaps while standard launch mode was false. Changed to `false` in both files; YAML consistency check for **11 Nav2 nodes per config** PASS. The launch can still explicitly override sim time for other environments, with verification required.

### P1: Invalid WGS84 mission item forwarded without local safety validation — FIXED IN SOURCE, SITL TEST PASS
Bridge previously validated mission item numeric finiteness/sequence but did not reject invalid `MAV_CMD_NAV_WAYPOINT` latitude, longitude, or unsupported global frame.
Added guards in `pixhawk_bridge/src/pixhawk_node.cpp` checking WGS84 lat ±90° and lon ±180° (int32 e7 units) and whitelisted standard global MAVLink mission frames **before** `MISSION_COUNT`.
SITL injected (a) latitude 95° and (b) frame=255. **Both ROS services refused with `success=False`; PX4 mission downloaded afterward remained unchanged.** Follow-up valid 2-point mission finished. Build 12/12 PASS.
**The active physical USB bridge process was NOT restarted** after this change and remains read-only with all movement, mode and mission write guards disabled; new validation is in the built binary for future controlled upgrades.

### P1: Waypoint regression used net start–finish displacement — FIXED
A square route can return to its start. Earlier test saw net 0.484m and could misjudge actual travel. It now integrates incremental odometry (5.717m recorded), checks for teleport jumps, verifies reached waypoint sequences and distinct mission IDs.

### P1: Replayed/stale/repeated MAVLink waypoint events could fool test — FIXED
Actual PX4 mission event stream repeats `MISSION_ITEM_REACHED` and can deliver prior mission events. The test clears event history upon a confirmed mission ID change, collapses repeats and asserts that `0,1,...,N-1` appears **in order**. 4/4 routes pass after fix.

### P1 still open: one Nav2 route contained an unknown costmap cell — BLOCKED, NOT EXECUTED
One of four computed candidate paths included a cell currently marked unknown by the global costmap sample; postplanning validator correctly returned invalid. Do not treat action `SUCCEEDED` as collision-free approval. For operational systems, implement a continuously updated path gate / collision recheck and STOP-or-replan on unknown/lethal/inflated obstacles. This was not integrated with actuators.

### Physical blockers not equivalent to ROS software bug fixes
- Real Cube PX4 1.17 is still `SYS_AUTOSTART=0`, `MAV_TYPE=0`, with no verified `RD_*` parameters; installed physical firmware Rover module unproven. Matching SITL Rover mode 50000 does **not** transfer to the device automatically.
- Receiver RC direct priority, ESC L/R pins, independent emergency stop and physical differential wheel geometry are not verified.
- Previously no real MID-360S LiDAR/IMU/LIO messages during sampling, and extrinsics remain uncalibrated. Real ROS bridge is monitoring-only.
- No manual override, sudden mini-PC power-loss failover, or Nav2/PX4 mixed motion tested on real hardware. Safety gating must remain enabled.

## 4. Regression commands

```bash
cd /media/sirobo/HDD/pixhawk
source env.sh
python3 scripts/test_waypoint_planner_unit.py
# Isolated simulator only. PX4 v1.17 differential SITL must run separately.
ROS_DOMAIN_ID=105 python3 scripts/test_px4_117_multiroute_sitl.py --route line2
ROS_DOMAIN_ID=105 python3 scripts/test_px4_117_multiroute_sitl.py --route corner3
ROS_DOMAIN_ID=105 python3 scripts/test_px4_117_multiroute_sitl.py --route square4
ROS_DOMAIN_ID=105 python3 scripts/test_px4_117_multiroute_sitl.py --route zigzag5
# Nav2 Stage6 fixture must be started on ROS_DOMAIN_ID 106 (not real LiDAR)
ROS_DOMAIN_ID=106 python3 scripts/test_nav2_fixture_multigoal.py
# Real Cube ONLY READS parameters/missions and tests denials:
python3 scripts/probe_cube_usb_safe.py
```

## 5. Safety and operational decisions

Production ROS2/PX4 remains **ADVISORY ONLY**. No command to physically arm, move, change mode, write parameters, flash, or rewrite mission on Cube was made during this audit. Do not connect synthetic Nav2 paths to wheel actuator outputs; ROS/MAVLink commands require physical E-STOP and independent RC override verification.
