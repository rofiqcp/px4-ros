# Cube/Pixhawk MAVLink ↔ ROS 2 Humble (C++17)

Workspace ROS 2: `/media/sirobo/HDD/pixhawk`  
Packages: `pixhawk_interfaces` (9 typed services and MissionItem message), `pixhawk_bridge` (MAVLink serial driver).  
Dependency headers taken from the existing local PX4 reference under `autopilot/PX4-Autopilot/build/px4_sitl_default/mavlink`; a local copy is included so this workspace builds on its own. No firmware modifications.

## Verified target

- Host: Ubuntu 22.04 / ROS 2 Humble.
- Device USB: `26ac:0011` / 3D Robotics PX4 FMU v2.x.
- Stable serial: `/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v2.x_0-if00`.
- MAVLink heartbeat autopilot PX4; telemetry `ATTITUDE`, `ATTITUDE_QUATERNION`, `HIGHRES_IMU`, `LOCAL_POSITION_NED`, `ODOMETRY`, `SYS_STATUS`.
- Current firmware reports `SYS_AUTOSTART=0`, `MAV_STATE_UNINIT`, GPS missing and an unusable local estimator. The system MUST NOT be armed or sent target navigation commands until configured and verified.

## Build / run

Build products are stored on the native ext4 filesystem `/home/sirobo/.cache/pixhawk_ros` because the HDD uses NTFS3, which is unsuitable for ROSIDL symlink-heavy builds. **All source files remain in the requested HDD project folder.**

```bash
cd /media/sirobo/HDD/pixhawk
bash build.sh
bash run.sh
```

The port is exclusively opened using Linux `flock` to reduce accidental competition with another serial owner. Do not run QGroundControl/MAVProxy/pymavlink against the same USB serial port simultaneously. Autoreconnect handles USB disconnect/reconnect.

## Topics (/pixhawk prefix is obtained using namespace)

**Important:** the default executable node has *relative* topic/service names without `/pixhawk`; the launch file sets the node namespace to `pixhawk`, yielding:

| Topic | ROS type | Data |
| --- | --- | --- |
| /pixhawk/status | std_msgs/String | JSON state, heartbeat, armed, airframe, GPS/local validity, counters |
| /pixhawk/events | std_msgs/String | STATUSTEXT firmware events |
| /pixhawk/imu | sensor_msgs/Imu | acceleration and gyro in FLU, latest orientation |
| /pixhawk/attitude | geometry_msgs/QuaternionStamped | converted ENU/FLU orientation |
| /pixhawk/mag | sensor_msgs/MagneticField | Tesla, FLU |
| /pixhawk/barometer | sensor_msgs/FluidPressure | Pascal |
| /pixhawk/gps/fix | sensor_msgs/NavSatFix | GPS WGS84, only when firmware emits GPS_RAW_INT |
| /pixhawk/odometry | nav_msgs/Odometry | converted NED→ENU local position; **covariance inflated** if estimator is invalid |
| /pixhawk/battery | sensor_msgs/BatteryState | missing voltage/current as NaN |
| /pixhawk/parameters | std_msgs/String | MAVLink PARAM_VALUE JSON |
| /pixhawk/raw_mavlink | std_msgs/UInt8MultiArray | optional, disabled by default to avoid load |

Quaternion transformation: MAVLink NED/FRD → ROS ENU/FLU. Position: ROS X=East, Y=North, Z=Up; body Y and Z gyro/accel signs inverted from FRD. Local odometry is NOT automatically fused into AGV/robot Nav2 due to possible estimator drift.

## Typed service interface

| Service | Type | Meaning |
| --- | --- | --- |
| /pixhawk/command_long | pixhawk_interfaces/srv/CommandLong | Send general MAV_CMD (await COMMAND_ACK); read-only queries allowed by default |
| /pixhawk/param_get | pixhawk_interfaces/srv/ParamGet | Read any MAVLink parameter, interpret integer type via bytewise encoding |
| /pixhawk/param_set | pixhawk_interfaces/srv/ParamSet | Write a parameter (requires opt-in + DISARMED), verify echoed value |
| /pixhawk/param_request_list | std_srvs/srv/Trigger | Request full list; observe /pixhawk/parameters |
| /pixhawk/set_message_interval | pixhawk_interfaces/srv/MessageInterval | Change telemetry interval using MAV_CMD_SET_MESSAGE_INTERVAL |
| /pixhawk/goto_local | pixhawk_interfaces/srv/GotoLocal | Target NED X/Y/Z (meters), yaw (rad), lifetime (sec) |
| /pixhawk/goto_global | pixhawk_interfaces/srv/GotoGlobal | WGS84 latitude/longitude + **absolute altitude AMSL** in meters, yaw, lifetime |
| /pixhawk/set_mode | pixhawk_interfaces/srv/SetMode | PX4 main/sub mode via MAV_CMD_DO_SET_MODE |
| /pixhawk/cancel_target | std_srvs/srv/Trigger | Stop streaming targets; does not directly stop motors |

Example safe read-only calls:

```bash
source /opt/ros/humble/setup.bash
source /home/sirobo/.cache/pixhawk_ros/install/setup.bash
ros2 topic echo --once /pixhawk/status
ros2 topic echo --once /pixhawk/imu
ros2 service call /pixhawk/param_get pixhawk_interfaces/srv/ParamGet "{name: SYS_AUTOSTART}"
ros2 service call /pixhawk/command_long pixhawk_interfaces/srv/CommandLong "{command: 512, params: [148.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
ros2 service call /pixhawk/set_message_interval pixhawk_interfaces/srv/MessageInterval "{message_id: 105, interval_us: 100000}"
```

## Motion safety

For automated **real Cube read-only** verification while the bridge is running: `bash verify.sh`.

For automated **simulated PX4** control/API verification (no hardware actuation):

```bash
source /opt/ros/humble/setup.bash
source /home/sirobo/.cache/pixhawk_ros/install/setup.bash
python3 /media/sirobo/HDD/pixhawk/scripts/sim_bench.py
```

See [REFERENCES.md](REFERENCES.md) for source audit and planned extensions.

Launch defaults: `enable_motion_commands=false`, `allow_mode_changes=false`, `allow_parameter_writes=false`.
To opt in **after airframe is properly configured and physically secured**, run the executable explicitly with ROS params rather than modifying source:

```bash
ros2 run pixhawk_bridge pixhawk_node --ros-args -r __ns:=/pixhawk \
  -p enable_motion_commands:=true -p allow_mode_changes:=true
```

This does *not* arm or change mode. Before enabling, confirm RC/manual recovery, hardware kill, failsafe for offboard loss, correct vehicle type/airframe, calibrated IMU and GPS/external localization. PX4 Offboard requires setpoint stream for >1s at >2Hz; the bridge publishes active setpoints at 10Hz and expires them at `duration_sec`. If stream expires/gets rejected, firmware offboard-loss behavior applies: configure failsafe on PX4 first. No guarantee of physical emergency stop is provided by stopping MAVLink stream.

```bash
# Example ONLY for a *fully configured and secured* test setup with reliable local estimator:
ros2 service call /pixhawk/goto_local pixhawk_interfaces/srv/GotoLocal \
"{x: 1.0, y: 0.0, z: -1.0, yaw: 0.0, duration_sec: 10.0}"
# After 1+ second of streaming, OFFBOARD main mode is 6:
ros2 service call /pixhawk/set_mode pixhawk_interfaces/srv/SetMode "{main_mode: 6, sub_mode: 0}"
```

A goto call is a streamed setpoint, **not a mission planner**, **not an arm command**, and **not a promise the target was reached**. Both goto services require valid estimator/GPS, configured airframe, reasonable bounded target and finite inputs. Targets beyond a 20m horizontal or 10m vertical envelope are rejected. Position estimate age and quality are checked continuously. Any command that is not explicitly telemetry-only requires an operator opt-in.

To write MAVLink parameters, start the node with `-p allow_parameter_writes:=true` while disarmed. Parameter types use MAV_PARAM_TYPE, and writes wait for PARAM_VALUE confirmation. Back up parameters before changing calibration or failsafe settings.

## Remaining integration

- Mission upload/download with ACK and retry is implemented and simulator-tested. Action feedback (reached waypoint), complete RC/actuator control, calibration wizard, MAVFTP/ULog and direct PX4 uORB signals still require further work.
- Add Nav2 or ROS 2 ActionServer only after a valid localization source and a vehicle-specific controller design are available.
- QGroundControl is the recommended interface for selecting airframe, vehicle-specific calibration, failsafes, flight-control and GPS setup.


## Pembaruan 8 Oktober 2026 — mission & sensor lengkap untuk backend

Workspace sudah dilengkapi layanan ROS 2 /pixhawk/mission_download dan /pixhawk/mission_upload berbasis MAVLink MISSION_ITEM_INT, MISSION_ACK, retry dan batas waktu; uji unggah/unduh misi pada simulator lulus. Pada Cube nyata hanya unduh misi yang dijalankan; upload misi tetap nonaktif kecuali operator mengaktifkan allow_mission_writes dan enable_motion_commands.

Topic sensor tambahan yang diaktifkan otomatis: /pixhawk/imu/instance_0, /pixhawk/imu/instance_1, /pixhawk/imu/instance_2, /pixhawk/imu/availability, /pixhawk/estimator/status, /pixhawk/estimator/odometry_raw, /pixhawk/sensors/health, /pixhawk/sensors/vibration, /pixhawk/mission/events. Permintaan frekuensi telemetri bersifat sementara. Firmware pada Cube teruji mengirim dua stream IMU (instance 0 dan 1); instance 2 tidak menghasilkan sampel. Nilai ESTIMATOR_STATUS (flags=37) menyatakan horizontal position/velocity tidak valid; estimator horizontal error ~1,4 km.

Lihat TEST_REPORT_20261008.md untuk hasil colcon build, backup parameter 732/732 sebelum/sesudah, 0 parameter berbeda setelah tes, uji tulis param sementara 0→60→0 berhasil pada SDLOG_UTC_OFFSET, unduh misi aktual, dan snapshot IMU/EKF.

Perintah praktis:

    bash /media/sirobo/HDD/pixhawk/build.sh
    bash /media/sirobo/HDD/pixhawk/run.sh
    bash /media/sirobo/HDD/pixhawk/verify.sh

Untuk pengujian yang tidak menyentuh hardware fisik:

    source /opt/ros/humble/setup.bash
    source /home/sirobo/.cache/pixhawk_ros/install/setup.bash
    python3 /media/sirobo/HDD/pixhawk/scripts/sim_bench.py

Catatan: sistem ini merupakan **backend komunikasi dan kontrol ROS2**, bukan tiruan grafis lengkap Mission Planner (belum menyediakan peta GCS, wizard kalibrasi, flashing firmware, unduh ULog/MAVFTP maupun seluruh fitur ground station).
