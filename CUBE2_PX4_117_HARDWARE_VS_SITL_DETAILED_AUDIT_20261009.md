# Cube 2 / PX4 v1.17.0 — Hardware Datasheet and Verified Mission/GOTO Audit
**Date:** 2026-10-09 | **Machine:** sirobo | **No firmware flashing or physical arming/motor motion**

## 1. Executive verdict
- **PASS on real USB Cube:** ROS2 ↔ serial ↔ MAVLink, exact PX4 firmware identification, board protocol capabilities, read-only parameter retrieval, actual MissionDownload(0 items), read-only VERSION request, safety lock deny tests. New ROS2 USB bridge is running and enabled only for read-only traffic.
- **PASS on full PX4 v1.17.0 Differential SITL + Gazebo:** Same ROS2 bridge (separate process on a PTY↔localhost UDP relay in ROS_DOMAIN_ID=103) uploaded 2 waypoints to the *actual PX4 SITL firmware* and read them back; PX4 SITL AUTO Mission selected, preflight PASS, normal ARM succeeded, vehicle changed position in simulation, second waypoint reached and mission reported COMPLETE. Simulation explicitly DISARMED at the end.
- **NOT READY on physical Cube:** no validated differential Rover firmware modules, no rover airframe, no RC receiver/channel mapping, no left/right motor wiring assignment, no GPS/local estimator validity and no physical E-STOP proof. Real GOTO, auto mode change, mission upload and motor outputs **remain disabled**.
- **No-flash constraint is respected:** installed Cube firmware is NOT modified.

## 2. Cube 2 / Cube Black hardware datasheet versus real USB identity
Official PX4 docs identify *Hex Cube Black* (often called Pixhawk 2.1) as FMUv3 STM32F427 Cortex-M4F 168 MHz, 256 KB RAM and 2 MB Flash, with STM32F103 fail-safe IO co-processor, triple IMU architecture (two vibration isolated), 8 MAIN plus 6 AUX PWM outputs, RCIN, 5 UARTs, 2 CAN, USB and redundant power modules.
- PX4 Cube Black hardware: https://docs.px4.io/main/en/flight_controller/pixhawk-2
- PX4 Cube wiring: https://docs.px4.io/main/en/assembly/quick_start_cube
- Cube carrier/connector specifications: https://github.com/CubePilot/cubepilot-docs/blob/master/autopilot/the-cube/introduction/interface-specifications.md
- FMUv3 vs FMUv2: https://docs.px4.io/main/en/flight_controller/pixhawk_series

### Physical wiring target for differential Rover
- RC transmitter → paired receiver → **RCIN** or proper Spektrum port → PX4, independently of ROS2/USB. Check protocol, supply and mode-switch physically.
- Left/right ESC control through two *firmware-verified* function/pin assignments on MAIN/AUX output. The serial USB connector is for data, not motor power. The 5V servo rail normally requires an external source/BEC if powering servo or receiver equipment; review the exact carrier board.
- E-STOP and motor-enable cut must be independent of mini-PC.
- **WARNING:** Do not assume MAIN1 is left ESC / MAIN2 right ESC without inspecting the actual carrier/wiring and checking the firmware output-function table. DShot/PWM protocols and shared channel groups vary.

### Observed actual device ID (not enough to uniquely confirm carrier/module color)
- Linux: `/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v2.x_0-if00` → `/dev/ttyACM0`.
- USB vendor/product: `26ac:0011`; reported product `PX4 FMU v2.x`. This *USB compatibility string is NOT definitive identification as FMUv2 or Cube Black*.
- MAVLink `AUTOPILOT_VERSION` from real Cube: **1.17.0 / OFFICIAL stable (release type 255)**; raw `17891583` (`0x011100ff`), `board_version=17`, capabilities `59647`, flight git version field `000000c4d12af1d6`. Board-version raw alone does not establish the color of Cube or the installed rover actuator firmware variant.
- Firmware response now exposed through a transient-local ROS2 topic `/pixhawk/firmware/version` on the upgraded read-only bridge. Runtime confirms one serial owner.
- No ECU reboot, reflash, parameter writes or motor arming took place.

## 3. Real Cube findings, tested over USB
ROS2 USB bridge was migrated, while the Cube was confirmed **disarmed** and all four movement/write flags false. The previous serial owner exited before the new one opened the USB port. Backup of previous configuration: `results/cube2_probe/original_bridge_ros_params_20261009.yaml`. New bridge log: `results/cube2_probe/cube2_new_bridge.log`.
- The upgraded binary still runs from `/home/sirobo/.cache/mid360_pixhawk_ros/install/pixhawk_bridge/...`, USB single-owner. All flags:
  `enable_motion_commands=false`, `allow_parameter_writes=false`, `allow_mode_changes=false`, `allow_mission_writes=false`, `allow_stock_rover_velocity=false`, `stock_rover_interface_confirmed=false`, `enable_avoidance_advisory_tx=false`.
- `SYS_AUTOSTART=0` and `MAV_TYPE=0`. Cube is currently not configured as a Rover.
- `COM_RC_IN_MODE=3`, `RC_CHAN_CNT=0`: RC takeover is **not physically verified**. PX4 1.17 requires appropriate manual input source policy and mode switch.
- Differential Rover parameter `RD_WHEEL_TRACK`: real Cube returned **PARAM_VALUE timeout**. No `RD_*` or `CA_*` parameters in the prior 732-parameter backup. The presence of Rover module in this actual firmware is **not verified** and no unsupported parameter was forced.
- Real Cube GPS/local estimator invalid; horizontal position accuracy >1 km in prior bridge measurements.
- Real Cube MissionDownload: **success; zero waypoint items**. It can return mission data but has no active mission.
- ROS2 `goto_local`, `goto_global`, `set_mode` AUTO, `mission_upload` requests **rejected by bridge before write**. Device remains DISARMED and connected. This is intentional: not a failed communication path.
- Sensor monitoring with correct `SensorDataQoS` in 8-second window: `/pixhawk/imu` 398, `/pixhawk/imu/instance_0` 80, `/pixhawk/imu/instance_1` 80, `/pixhawk/mag` 397, `/pixhawk/barometer` 397, `/pixhawk/battery` 31, `/pixhawk/rover/feedback` 557, `/pixhawk/status` 7. There was no data on IMU instance_2 during that period; do not assert a third IMU failed without further PX4 sensor health tests.
- Complete USB safe probe program: `scripts/probe_cube_usb_safe.py`.
- JSON report: `results/cube_usb_mission_goto_readonly_20261009.json`.
- QA log: `results/cube2_probe/cube2_final_live_usb_audit.log`.

## 4. PX4 v1.17 differential firmware facts (hardware incompatibility is NOT fixed by setting SYS_AUTOSTART)
- Matching version source `/media/sirobo/HDD/autopilot/PX4-Autopilot-v1.17.0` provides distinct `boards/px4/fmu-v3/default.px4board` and `rover.px4board`; the Rover configuration explicitly sets `CONFIG_MODULES_ROVER_DIFFERENTIAL=y` and disables selected multi/fixed-wing modules.
- PX4 rover setup explicitly requires a separately built **Rover firmware variant**. `SYS_AUTOSTART=50000` selects Generic Differential Rover ONLY if Rover modules are built in.
- See https://docs.px4.io/v1.17/en/config_rover/ and https://docs.px4.io/main/en/config_rover/differential.html
- The source checkout is **v1.17.0-dirty** (there are local Ackermann/MAVLink modifications): it is not claimed byte-for-byte identical to the firmware on the Cube.
- No attempt to flash PX4 Rover firmware was made. Under a strict no-flash constraint, one cannot invent an absent Rover controller by writing parameters.

## 5. True PX4 v1.17 SITL/Gazebo simulation, end-to-end evidence
- Build and run command (software isolated): `cd /media/sirobo/HDD/autopilot/PX4-Autopilot-v1.17.0 && HEADLESS=1 make px4_sitl gz_rover_differential`.
- This finished the compiled PX4 binary, world started, model `rover_differential_0`, MAVLink UDP 18570→14550 on loopback; `SYS_AUTOSTART=50000`, Rover `MAV_TYPE=10`, `RD_WHEEL_TRACK=0.60` in **SITL only**.
- ROS2 SITL test `scripts/test_px4_real_sitl_ros_mission.py` bridges loopback UDP to a **temporary /dev/pts/**, and launches the actual compiled ROS2 `pixhawk_node` inside **ROS_DOMAIN_ID=103**. No reference to the physical Cube /dev/ttyACM0 inside the simulated bridge. Actual PX4/Gazebo, not a canned Python MAVLink ACK emulator.
- MissionDownload: 2 or 0 items depending on previous saved SITL mission. ROS2 MissionUpload: **success `MISSION_ACK=0 sent=2`**, then MissionDownload **success 2 items**. The coordinates were corrected to the actual Gazebo GPS home **47.3979711, 8.546164** with nearby offsets (not the earlier far-away placeholder).
- Unarmed ROS2 GOTO local service: bridge can accept/stream an OFFBOARD setpoint when PX4 is in valid standby, but after an earlier OFFBOARD mode and `mav_state=UNINIT` heartbeat it intentionally refused GOTO; real PX4 state and estimator quality remain mandatory. **An accepted GOTO setpoint request is not itself arrival**.
- The simulator **AUTO Mission** navigation was subsequently tested with actual PX4 console `px4-commander mode auto:mission` (success), `px4-commander check` (Preflight OK), normal `px4-commander arm` (success, no force override). Gazebo Rover physically moved inside simulation.
- Initial local XY: **(-0.06877, -0.00305) metres**.
- After 7 seconds: **(1.39909, 0.90330) metres**.
- PX4 `mission_result`: **seq_reached=1, seq_current=1, seq_total=2, valid=True, finished=True, failure=False**. SIMULATOR DISARMED at test completion.
- These are simulated motor physics and waypoint arrival **from PX4 itself**; they do not establish that the physical Cube can presently move or has a correct output map.
- SITL log: `/home/sirobo/.cache/mid360_pixhawk_ros/px4_117_rover_sitl_continuous.log`.
- ROS2 actual PX4 mission evidence: `results/cube2_probe/px4_117_real_sitl_ros_mission.json` and `px4_117_real_sitl_ros_bridge_test*.log`.
- Note: `mode auto:mission` and simulated `arm` were issued via the local PX4 SITL CLI after the mission was loaded from ROS2. This proves integrated ROS2 mission provisioning → real PX4 SITL mission controller → Gazebo motion, not yet fully automatic ROS-only arm/start/arrival sequence.

## 6. Remaining requirements before physical hardware GOTO
1. Verify real module/carrier model from **photographs of Cube label + carrier board**, identify RC receiver brand/protocol and ESC left/right signal wiring. USB 26ac:0011 cannot fully establish pinout or actuator mapping.
2. Confirm receiver is powered, actually reporting valid channels and reliable hardware MANUAL switch before any actuator enable; `RC_CHAN_CNT=0` blocks takeover proof.
3. Under **no firmware flashing**, check installed PX4 modules; absent Rover module means physical differential Rover cannot work as desired via mere `SYS_AUTOSTART=50000`/MAV_TYPE writes. Obtain explicit authorization if a compatible firmware change is later required; never flash implicitly.
4. Measure wheel-track/diameter and ESC protocols, map left and right output **by firmware function**, test with motor power physically disconnected and independent E-STOP.
5. Complete valid horizontal localization and external LiDAR extrinsic calibration; real Cube has no valid mission target today.
6. Only after a separated safe-bench stage and explicit test setup, change Cube parameters with backups/readback, test RC override and two ESC directions, then bounded physical GOTO in a protected area.
7. Keep USB read-only until those prerequisites are verified. This report intentionally does not provide an unsafe bypass around the bridge's disarmed/motion locks.

## 7. One-command safe hardware verification
```bash
cd /media/sirobo/HDD/pixhawk
source env.sh
python3 scripts/probe_cube_usb_safe.py
```
This test reads actual Cube data and verifies write-denial safety; does not move motors.

## 8. New milestone: FULL ROS2-ONLY PX4 v1.17.0 SITL AUTO Mission — VERIFIED

After the earlier mixed ROS2+PX4 CLI navigation test, the independent ROS2-only test was completed on the **actual PX4 v1.17.0 Differential Rover SITL process**:

- Program: `scripts/test_px4_117_sitl_ros_auto.py` in `ROS_DOMAIN_ID=104`, **not** the production ROS_DOMAIN_ID=0.
- Runtime endpoint: temporary Linux pseudo-terminal (/dev/pts/0) ↔ localhost UDP port 18570/14550, wired only to PX4 SITL Gazebo. The real Cube USB serial remained owned solely by `/pixhawk/pixhawk_bridge` in the production ROS domain.
- Guards: checks the SITL process path, exact simulator domain, `/dev/pts/` device and PX4 Rover type. Never opens `/dev/ttyACM0`.
- ROS2→SITL MissionUpload(2 waypoints near actual Gazebo GPS home): **PASS** (`MISSION_ACK=0 sent=2`).
- ROS2→SITL MissionDownload readback and coordinate equality: **PASS**.
- ROS2→SITL `SetMode(main=4,sub=4)` AUTO Mission: **PASS**, `COMMAND_ACK result=0`, PX4 heartbeat reported actual AUTO Mission.
- ROS2→SITL `CommandLong(MAV_CMD_COMPONENT_ARM_DISARM, arm=1, force=0)`: **PASS**, `COMMAND_ACK result=0`, PX4 heartbeat armed.
- Gazebo Rover changed local odometry from **(0.880596, 1.404854)m** to **(1.761726, 2.724802)m**, displacement **1.587027 metres**. Pure simulator; no physical motor output.
- ROS2→SITL `CommandLong(MAV_CMD_COMPONENT_ARM_DISARM, arm=0)`: **PASS**, `COMMAND_ACK result=0`, confirmed simulator disarm; additionally the local SITL CLI disarm is registered as a failsafe `finally`.
- Actual PX4 `mission_result`: `seq_reached=1`, `seq_current=1`, `seq_total=2`, `valid=True`, `finished=True`, `failure=False`.
- Recorded JSON: `results/cube2_probe/px4_117_auto_from_ros2.json`.
- Console test log: `results/cube2_probe/px4_117_ros_only_auto_mission_test.log`.
- Python `ruff --select F401,F841`: PASS after removing an unused import, and Python syntax compilation PASS.
- Existing USB physical Cube status after simulator test: disarmed, connected, exactly one physical serial owner, firmware 1.17.0 readback, all hardware motion/mission/parameter write flags disabled.

**This result verifies complete ROS2→MAVLink→PX4 v1.17 Differential SITL→Gazebo movement→mission completed→ROS2 disarm. It is not a physical hardware GOTO success claim.** Firmware physical module Rover and RC/motor wiring remain unverified, and the no-flash requirement remains active.

To reproduce only on SITL:
```bash
# Terminal 1 (simulation only)
cd /media/sirobo/HDD/autopilot/PX4-Autopilot-v1.17.0
HEADLESS=1 make px4_sitl gz_rover_differential

# Terminal 2 (simulation isolated from Cube USB)
cd /media/sirobo/HDD/pixhawk
source env.sh
ROS_DOMAIN_ID=104 python3 scripts/test_px4_117_sitl_ros_auto.py
```

## 9. Repeatability / source cleanup
The exact ROS2-only SITL test was rerun **after Ruff F401/F841 and Python compilation passed**. It again returned SUCCESS for MissionUpload 2 items, coordinate readback, AUTO mode ACK=0, ARM ACK=0, movement **1.57833823124769 metres**, PX4 `mission_result finished=True failure=False seq_reached=1`, and DISARM ACK=0. No physical device used.
The latest JSON reflects this second run: `results/cube2_probe/px4_117_auto_from_ros2.json`. Captured console log: `results/cube2_probe/px4_117_ros_only_auto_mission_final_clean.log`. PX4 post-test telemetry and disarmed state: `results/cube2_probe/sitl_final_mission_telemetry.txt`.
