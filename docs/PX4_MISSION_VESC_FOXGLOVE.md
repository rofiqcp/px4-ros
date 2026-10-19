# MiniPC1: PX4 AUTO.MISSION, wall follower, VESC and Foxglove

## Controller ownership (critical)

**PX4 stock AUTO.MISSION and OFFBOARD setpoints cannot own steering simultaneously.**
The current integration uses:
1. PX4 remains the source of mission waypoints and active sequence over its existing USB MAVLink connection.
2. `/pixhawk/mission_download` reads mission items from PX4 without uploads.
3. `px4_mission_foxglove` previews the current waypoint, projected path and wall follower steering. Its `/px4_mission/preview_twist` is **not** fed to PX4 motor outputs.
4. `px4_vesc_interface` converts the preview into *virtual* Ackermann curvature and left/right motor eRPM diagnostic values using wheelbase, track width, wheel radius and pole pairs.
5. Foxglove shows topics, 3D path, obstacle cloud, tuning diagnostics and marker positions. It never sends arming, parameter writes or motor commands through WebSocket.

**Active PX4 mission steering intervention needs explicit supported firmware capability, mission revision or a supervised mode handover to OFFBOARD.** That is not achieved by simply streaming yaw setpoints while staying in AUTO.MISSION. No hardware control was enabled by this change.

## ROS workflow on MiniPC1

```bash
cd /home/otomasi/Project/px4-ros
./build.sh
source ./env.sh
ros2 launch px4_mission_foxglove mission_foxglove.launch.py start_usb_bridge:=true
```

Do not start another bridge, QGroundControl or MAVProxy on the same USB port. The launcher starts the USB bridge with motion commands, mission writes and mode changes **disabled**. If a USB owner already exists, omit the `start_usb_bridge:=true` argument.

To use LiDAR advisory input (after confirming the calibrated Livox pipeline is already running):

```bash
ros2 launch px4_mission_foxglove mission_foxglove.launch.py start_usb_bridge:=true start_lidar:=true
```

`obstacle_advisory_node` stays fail-closed with `calibration_verified=false` until the sensor frame and extrinsics are verified.

## Live Foxglove

On MiniPC1, the official ROS Humble `foxglove_bridge` package (v3.6.0) was installed as an **unprivileged user-local overlay** under `~/.local/ros-foxglove/opt/ros/humble`. After `source ./env.sh`, ROS finds the package; the tested server listens at `127.0.0.1:8765` and accepts the `foxglove.sdk.v1` WebSocket subprotocol. The ROS topics are available for Foxglove, but actual import/rendering of each GUI panel has not been verified. On a fresh computer, the official Foxglove ROS 2 bridge might need installation. Check via:

```bash
source /opt/ros/humble/setup.bash
ros2 pkg prefix foxglove_bridge
```

Install from official ROS package channels when administrator privileges are available:
```bash
sudo apt update
sudo apt install ros-humble-foxglove-bridge
```

Foxglove WebSocket is bound to **127.0.0.1:8765** (localhost) and forbids any Foxglove-side topic publication, parameter writes and service calls. On another workstation, use an SSH tunnel rather than publishing an unauthenticated WebSocket interface to the LAN. In Foxglove, add a Foxglove WebSocket connection to `ws://localhost:8765` and import `foxglove/px4_mission.layout.json`. Panels include mission 3D preview, status, diagnostics and speed/wall steering plots. Mission preview coordinates use the separate `px4_mission_preview` frame anchored at the *first geographic mission waypoint* and are not a Nav2 `map` frame.

Tuning from ROS terminal (safe observer-only fields):

```bash
ros2 param set /mission_observer heading_kp 0.5
ros2 param set /mission_observer cruise_speed_mps 0.25
ros2 param set /mission_observer max_yaw_rate_radps 0.20
```

For the supervisor and obstacle node, inspect their package-local YAML, RViz-independent diagnostics and LiDAR range thresholds. Persistent parameters should be changed in the package config and reviewed under version control. Foxglove can visualize parameters, plots, Path, markers, TF and PointCloud2, but not every RViz-specific plugin has an equivalent.

## VESC hardware boundary

Firmware source: https://github.com/rofiqcp/hoverboard-vesc (checked out separately at `/home/otomasi/Project/hoverboard-vesc`). Its USART3 control protocol is VESC 6-compatible, with left local VESC ID 1 and right virtual CAN ID 2; the ROS adapter calls firmware-provided `VescDual.set_rpm` only behind independent interlocks.

**Hardware available now: two drive motors, not a confirmed physical steering actuator.** Ackermann angle is a preview only, and the two wheel setpoints are a differential approximation. Do not present this as real Ackermann servo feedback or steering control. Before moving a physical vehicle, verify wheelbase, track width, wheel diameter, gearing, motor pole pairs, wheel polarity, mechanical steering servo, encoder feedback, ROS operator takeover, emergency stop and which controller actually owns each actuator.

The bridge defaults to `enable_output=false`, `hardware_verified=false`, `external_drive_authorized=false` and blank `vesc_port`. Even if set, it rejects PX4 AUTO.MISSION and requires OFFBOARD, an armed/healthy PX4, fresh ROS operator heartbeat, a physical E-stop and a separate process-level guard. This is a commissioning safeguard, **not a certified functional safety system**. It is intentionally not armed or tested on a live motor in this delivery.

## Published topics

| Topic | Type | Description |
| --- | --- | --- |
| `/px4_mission/path` | `nav_msgs/Path` | Path from downloaded PX4 geographic waypoints |
| `/px4_mission/markers` | `visualization_msgs/MarkerArray` | Waypoint spheres with active item coloring |
| `/px4_mission/status` | `std_msgs/String` | JSON mission progress and guarded preview state |
| `/px4_mission/preview_twist` | `geometry_msgs/TwistStamped` | **Non-actuating** heading/wall-follow recommendation |
| `/vesc_ackermann/diagnostics` | `std_msgs/String` | Steering preview, left/right eRPM, interlock state |
| `/pixhawk/status` | `std_msgs/String` | USB/MAVLink PX4 heartbeat, mode, RC and sensor checks |
| `/pixhawk/mission/events` | `std_msgs/String` | MAVLink mission progress messages |
| `/pixhawk/stock/advisory` | `pixhawk_interfaces/AvoidanceAdvisory` | Obstacle and wall-following safety advisory |

## Why mode handover is still required

MAVLink mission download is allowed while PX4 owns AUTO.MISSION. OFFBOARD has distinct streaming and timeout/arming semantics. Directly mixing internal mission setpoints with external motor control is prohibited; either use a verified PX4-native mission replanning mechanism, or explicitly pause/hand over to OFFBOARD with tested failsafe and resume logic.

## Observed MiniPC1 hardware state (2026-10-19)

Real PX4 USB heartbeat and AUTOPILOT_VERSION were observed. A direct read-only
`/pixhawk/mission_download` call returned `success=True`, **zero** mission
items: no waypoint route is available to visualize yet. PX4 reported AUTO
submode 3 (not AUTO.MISSION submode 4), disarmed, airframe unconfigured,
invalid GPS/local position, and no fresh RC telemetry. Therefore the
mission observer correctly emits zero movement preview and the VESC bridge
reports `MONITOR_ONLY`. Live wall-follow execution and Foxglove mission
route rendering cannot be accepted as hardware-verified with those inputs.

The optional localhost-only PX4 calibration and pre-arm web panel is documented at [PX4_CALIBRATION_WEB.md](PX4_CALIBRATION_WEB.md).
