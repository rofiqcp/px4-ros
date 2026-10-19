# Referensi lokal dan dependensi

Telah dicek pada RDC sirobo, 8 Oktober 2026.

## Kode sumber yang dipakai

- `/media/sirobo/HDD/autopilot/PX4-Autopilot`: PX4 firmware/source. Generated MAVLink headers from `build/px4_sitl_default/mavlink` have been copied locally into `pixhawk_bridge/third_party/mavlink`, giving this ROS 2 package independent build headers.
- `/media/sirobo/HDD/autopilot/PX4-Autopilot-v1.17.0`: PX4 firmware revision comparison relevant to the detected PX4 version.
- `/media/sirobo/HDD/autopilot/px4_msgs` and `px4_ros_com`: future ROS 2 native micro-XRCE-DDS/uORB integration; **not required for the current direct-USB MAVLink bridge**.
- `/media/sirobo/HDD/autopilot/mavlink-router`: future transport fan-out when GCS, ROS and log capture need the same telemetry stream.
- `/media/sirobo/HDD/ardupilot/ardupilot`, `mavlink`, `pymavlink`, and `MAVProxy`: protocol reference and Python simulator tooling; currently connected Cube was verified as **PX4**, not ArduPilot.

No additional repository was cloned because these local repositories already provide the required protocol headers, PX4 sources, and reference implementations. The build copies generated MAVLink headers (approximately 8 MB) into the ROS workspace.

## Official references

- https://mavlink.io/en/messages/common.html
- https://docs.px4.io/main/en/flight_modes_mc/offboard
- https://docs.px4.io/main/en/ros2/user_guide
- https://docs.px4.io/main/en/config/airframe
- https://docs.ros.org/en/humble/

## Required extension work for comprehensive vehicle control

- Proper mission protocol upload/download, `MISSION_ACK`, sequence/retransmit support.
- Vehicle-specific action server for waypoint reaching and timeout/abort/feedback, geofence and obstacle constraints.
- RC/actuator telemetry and control authorization, power-management fields, prearm and failsafe diagnostics.
- Valid GPS/RTK or visual odometry with estimator health and ROS 2/Nav2 integration.
- SITL/HIL regression across power/disconnect and emergency recovery.

This revision has tested direct serial telemetry, parameters, command ACK, guarded offboard setpoint streaming and cancellation; it does not assert full flight-control certification.
