# PX4 Cube 2 — Gazebo Simulation Verification
**Host:** `sirobo` | **Date:** 9 October 2026 | **Mode:** PX4 SITL + Gazebo Harmonic + ROS2 Humble | **Safety:** Cube USB read-only

## Result

**SUCCESS — repeatable Gazebo simulation of PX4 1.17 Differential Rover.** The simulator ran actual PX4 SITL code, Gazebo wheel physics, and ROS2 MissionUpload/Download/AUTO/ARM/DISARM through an isolated pseudo-terminal → local UDP MAVLink pathway. **The physical Cube 2 was never armed, flashed, configured, or moved.**

This is **not** a hardware-in-the-loop emulator of Cube 2's STM32F427, STM32F103, redundant IMUs, ESC pins, RC receiver, or onboard firmware boot process. PX4 version 1.17.0 in the SITL and Cube match **at release version level**, but the local PX4 source is modified (`git describe: v1.17.0-dirty`) and it is not byte-identical to the image flashed in the physical Cube.

## Verified references and local dependencies

- PX4 source `/media/sirobo/HDD/autopilot/PX4-Autopilot-v1.17.0` with `CONFIG_MODULES_ROVER_DIFFERENTIAL=y` in `boards/px4/fmu-v3/rover.px4board`. This is a board build configuration in source, not evidence the physical Cube already has that module.
- PX4 official model: `Tools/simulation/gz/models/rover_differential/model.sdf`, world `Tools/simulation/gz/worlds/rover.sdf`.
- Gazebo 8.15.0 (Harmonic), actual PX4 model `rover_differential_0`; `SYS_AUTOSTART=50000` checked via SITL parameter CLI.
- Gazebo model contains virtual IMU, magnetometer, air-pressure sensor, GNSS; Gazebo does **not** emulate three physical IMU banks.
- 2 actively controlled Gazebo motor joints: `wheel_rear_left_joint` uses actuator index 1, `wheel_rear_right_joint` uses actuator index 0. These are virtual actuator indices, **not** physical MAIN/AUX Cube pin mappings.
- ArduPilot alternative source available `/media/sirobo/HDD/ardupilot/ardupilot_gazebo`, `/media/sirobo/HDD/ardupilot/ardupilot_gz`, and `/media/sirobo/HDD/ardupilot/ardupilot/build/sitl/bin/ardurover`; not used for the PX4 tests because physical Cube firmware is PX4.

Official documentation:
- https://docs.px4.io/v1.17/en/sim_gazebo_gz/vehicles
- https://docs.px4.io/v1.17/en/config_rover/basic_setup
- https://docs.px4.io/v1.17/en/simulation/hitl
- https://ardupilot.org/dev/docs/sitl-with-gazebo.html

PX4 official docs list HITL for multicopters and VTOL using Gazebo Classic, not Cube 2 Differential Rover using current Gazebo Harmonic. Physical Cube Rover support would require special Rover firmware build and physical RC/actuator wiring validation. No firmware flashing was done.

## Simulation process verification

- A new isolated instance of `build/px4_sitl_default/bin/px4` and `gz sim .../rover.sdf` started using the reproducible script.
- Gazebo startup log contains `world: rover, model: rover_differential_0`, `Startup script returned successfully`.
- MAVLink localhost UDP: SITL 18570 ↔ ROS2 temporary PTY relay → 14550. **No test opens physical USB `/dev/ttyACM0`.**
- Four missions were uploaded and read back through the **compiled ROS2 bridge** to actual PX4 SITL. Each completed in PX4 `mission_result finished=True failure=False` and reported all reached waypoints in order.
- Simulator ARM and DISARM both occurred with normal MAVLink acknowledgment; SITL was disarmed and cleaned on suite completion.

| Route | Waypoint sequence reached | Traveled distance from Gazebo-based odometry | Result |
|---|---|---:|---|
| Lurus, 2 titik | 0,1 | 3.221m | PASS |
| Belok siku-siku, 3 titik | 0,1,2 | 4.595m | PASS |
| Persegi, 4 titik | 0,1,2,3 | 5.550m | PASS |
| Zigzag, 5 titik | 0,1,2,3,4 | 8.413m | PASS |

Additional negative tests:
- Latitude 95° in uploaded mission -> locally rejected, original mission preserved.
- Unsupported MAVLink mission frame 255 -> locally rejected, original mission preserved.
- Simulator workflow only permits `ROS_DOMAIN_ID=105` for ROS2 motion tests.
- An independent second smoke run of 2-point mission PASSED (3.257m), proving the launcher can restart/clean a new simulator instance.

### Differential wheel physics inspection

`results/gazebo_joint_velocity_verification.json` records:
- 1,918 paired Gazebo joint-state messages examined
- both left/right motor joints received nonzero velocity
- 963 samples with unequal velocities between left and right joints
- left wheel observed max ~14 rad/s, right wheel -10 to +12 rad/s across sampled turns

Those are **SIMULATOR** joint angular speeds, not measured RPM of the real hardware ESC/motor.

### Visual GUI test

- GUI X display `DISPLAY=:0` authenticated successfully at 1680×1050.
- Interactive Gazebo startup showed a real `Gazebo Sim` window 1000×845 in GNOME while the model/world was running.
- Screenshot: `results/gazebo_visual_desktop_20261009.png` (PNG 1000×845).
- A time-boxed 33-second GUI test was terminated deliberately; timeout exit 124 reflects the deliberate time limit, **not inability to load the GUI**. All simulator processes from the GUI test were stopped.

## Usage

Single-command end-to-end headless regression (has a real Cube read-only safety preflight):

```bash
cd /media/sirobo/HDD/pixhawk
source env.sh
python3 simulation/cube2_px4_gazebo/run_gazebo_regression.py
```

Launch Gazebo desktop GUI (interactive, no automatic ARM):

```bash
cd /media/sirobo/HDD/pixhawk
bash simulation/cube2_px4_gazebo/start_visual_gazebo.sh
```

For detailed instructions see `README.md`. Full-run log `results/repeatable_suite_20261009.log`; summary `results/full_suite_20261009.json`; compact canonical `results/summary.json`; GUI log `results/visual_gui_smoke.log`.

## Hardware status on completion

- Physical Cube USB bridge still the single owner; hardware PX4 reads v1.17.0.
- `SYS_AUTOSTART=0`, `MAV_TYPE=0` on actual Cube, RC channels not physically verified, and real `RD_*` support unproven.
- Physical Cube remains disarmed and all bridge write flags false. Real motion and mission upload **NOT TESTED**.
- No Gazebo/PX4 SITL processes left running after regressions, so the host is not consuming simulator CPU when idle.

## Remaining differences / limitations

- This is software-in-the-loop for the **right autopilot family/version and differential vehicle class**, but not a microcontroller/board-specific HITL.
- Actual Cube/ESC PWM timing, two motor directions, RC receiver overrides, GNSS/IMU redundancies and independent E-STOP require safe physical bench testing. SDF's wheel-track/dynamics should not be copied to physical chassis without measurement.
- PX4 rover SDF emits non-fatal `gz_frame_id` and caster joint-name warnings on Gazebo 8.15.0; the two actuated rear joints and all missions operate correctly. Do not silently patch upstream vendor models based solely on warnings.
- Real MID-360 pointcloud, wall avoidance, ROS Nav2 replanning, and ROS/PX4 mixing while executing mission are not integrated into this particular Gazebo model. They are distinct follow-on tests and **should not be described as proven**.
