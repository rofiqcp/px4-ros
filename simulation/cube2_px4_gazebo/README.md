# Cube 2 / PX4 v1.17 Differential Rover in Gazebo — Verified Software Simulation

**Host:** sirobo (Ubuntu 22.04, ROS2 Humble, Gazebo Harmonic 8.15.0)
**Date:** 2026-10-09
**Cube actual firmware:** PX4 v1.17.0 OFFICIAL, as read from `AUTOPILOT_VERSION` over USB.
**Actual Cube hardware:** USB-compatible PX4 FMU v2.x identifier; exact module/carrier requires visual identification.
**Simulator:** PX4 v1.17.0 SITL running as Linux process; Gazebo world `rover`, model `rover_differential`.
**Physical Cube:** remains DISARMED, real parameter/mission write and motion commands disabled.
**Important:** This is SITL on the PC, **not HITL on the physical Cube 2**. The simulated model is not a cycle-accurate FMUv3 emulator.

## Existing local repositories

| Reference | Local path | Purpose |
|---|---|---|
| PX4 v1.17.0 | `/media/sirobo/HDD/autopilot/PX4-Autopilot-v1.17.0` | Same firmware version family as Cube; the source checkout is locally **dirty** |
| PX4 model | `Tools/simulation/gz/models/rover_differential/model.sdf` | Differential physics, 2 actuated wheel joints |
| PX4 world | `Tools/simulation/gz/worlds/rover.sdf` | Optimized rover world |
| PX4 FMUv3 rover board recipe | `boards/px4/fmu-v3/rover.px4board` | Includes `CONFIG_MODULES_ROVER_DIFFERENTIAL=y`; **not proof real firmware includes it** |
| PX4 generic differential airframe | `ROMFS/px4fmu_common/init.d/airframes/50000_generic_rover_differential` | SITL airframe: `SYS_AUTOSTART=50000` |
| ArduPilot + Gazebo | `/media/sirobo/HDD/ardupilot/ardupilot_gazebo` | Alternative autopilot plugin; not PX4 Cube firmware |
| ArduPilot ROS2/Gazebo | `/media/sirobo/HDD/ardupilot/ardupilot_gz` | Additional reference, **not used in PX4 regression** |

The original Cube parameter snapshot has `SYS_AUTOSTART=0`, `MAV_TYPE=0`, no verified `RD_*` differential parameters and no verified RC input or motor output assignments. **Do not write `SYS_AUTOSTART=50000` onto real Cube just because the SITL uses it.** PX4 documentation says a separate Rover firmware build is needed for the physical differential airframe.

## One command: start simulator, run 4 missions, stop and save evidence

```bash
cd /media/sirobo/HDD/pixhawk
source env.sh
python3 simulation/cube2_px4_gazebo/run_gazebo_regression.py
```

The launcher:
1. **Before launching anything**, probes the actual Cube USB with a read-only script and checks the disarmed state + all write locks.
2. Starts Gazebo Harmonic and PX4 v1.17.0 Differential Rover in a separate process group; waits for both processes and firmware startup.
3. Runs the actual compiled ROS2 MAVLink bridge over temporary **`/dev/pts/*`** ↔ loopback UDP, under isolated **`ROS_DOMAIN_ID=105`**.
4. Uploads 2/3/4/5 point missions, re-downloads coordinates from PX4, requests AUTO Mission and normal arm/disarm on the **simulator only**.
5. Checks all individual `MISSION_ITEM_REACHED` events in order, fresh mission IDs, `finished=True`, `failure=False`, odometry traveled and simulator disarm.
6. Injects invalid latitude and unsupported MAVLink frame: both must be locally rejected without modifying the valid mission.
7. Saves `results/summary.json` and separate logs per scenario, disarms simulator and stops Gazebo/PX4 processes even on test failure.

The launcher will refuse to take control of an existing PX4/Gazebo rover session. For individual test routes:

```bash
python3 simulation/cube2_px4_gazebo/run_gazebo_regression.py --routes line2
python3 simulation/cube2_px4_gazebo/run_gazebo_regression.py --routes corner3 square4
```

## Launch in visual mode on the sirobo desktop

Use the authenticated GNOME X display (detected `DISPLAY=:0` with 1680x1050). The helper checks access to the display first:

```bash
cd /media/sirobo/HDD/pixhawk
bash simulation/cube2_px4_gazebo/start_visual_gazebo.sh
```

This **starts the Gazebo window** and runs PX4 in the foreground. It deliberately does not arm, upload a mission or touch the physical USB device. To run waypoint scenarios against this manually launched simulation, use another terminal in the same host:

```bash
cd /media/sirobo/HDD/pixhawk
source env.sh
ROS_DOMAIN_ID=105 python3 scripts/test_px4_117_multiroute_sitl.py --route corner3
```

**GUI smoke test passed:** on the authenticated sirobo X display (`:0`, 1680x1050), a real 1000x845 `Gazebo Sim` window opened with PX4 rover world/model, and `results/gazebo_visual_desktop_20261009.png` was captured. The test was intentionally time-boxed and shut down after the window/model were verified. The automated 4-mission headless run is separately verified.

## Verified full-run evidence

| Mission | Reached events | PX4 mission result | Gazebo odometry path |
|---|---|---|---|
| line2 | 0,1 | finished=true, failure=false | 3.221 m |
| corner3 | 0,1,2 | finished=true, failure=false | 4.595 m |
| square4 | 0,1,2,3 | finished=true, failure=false | 5.550 m |
| zigzag5 | 0,1,2,3,4 | finished=true, failure=false | 8.413 m |

All 4 test processes returned exit code 0, all missions PASS. Logs and results:
- `results/summary.json`
- `results/repeatable_suite_20261009.log`
- `results/{line2,corner3,square4,zigzag5}.log`
- `results/px4_gazebo.log`
- `results/gazebo_joint_velocity_verification.json`: 1918 paired joint states; both wheels actuated, 963 unequal velocities, proving differential steering in Gazebo
- Full audit: `/media/sirobo/HDD/pixhawk/ROS2_PX4_NAV2_WAYPOINT_BUG_AUDIT_20261009.md`.

## Cubepilot hardware vs simulated rover

The actual Cube Black/FMUv3 has STM32F427/STM32F103 processors and redundant hardware sensors. Gazebo supplies **one** virtual IMU, magnetometer, barometer and NavSat GPS plus physics/joint telemetry; it does **not** reproduce the Cube's multiple IMU chips, exact PCB, RC receiver, left/right ESC protocol/pin wiring, watchdog or FMUv3 bootloader/firmware image.

The simulated software outputs are connected to two Gazebo wheel controllers:
- **Actuator 1 → left rear wheel joint**
- **Actuator 0 → right rear wheel joint**

These are **Gazebo virtual actuator indices**, NOT `MAIN1/MAIN2` pin wiring for the physical Cube!

The physical firmware is v1.17.0 but an installed Rover build is NOT proven. Hardware-in-the-loop for PX4 v1.17 is community-supported and docs list multicopter/VTOL for Gazebo Classic, NOT generic Differential Rover on Cube 2. Real differential GOTO requires a validated Rover firmware build and matching ESC, RC/manual override, E-STOP, track calibration and localization. No PX4 Cube parameter or firmware changes were made.

## Official references

- PX4 1.17 vehicles: https://docs.px4.io/v1.17/en/sim_gazebo_gz/vehicles
- PX4 1.17 Rover Basic Setup (requires special Rover firmware): https://docs.px4.io/v1.17/en/config_rover/basic_setup
- PX4 1.17 Hardware-in-the-Loop: https://docs.px4.io/v1.17/en/simulation/hitl
- ArduPilot Gazebo: https://ardupilot.org/dev/docs/sitl-with-gazebo.html
- ArduPilot ROS2/Gazebo: https://ardupilot.org/dev/docs/ros2-gazebo.html

### Current known limitations
- The PX4 v1.17.0 source checkout has local changes; compiled SITL source is not byte-identical to firmware flashed in Cube.
- Rover model inertia, wheel dimensions and traction reflect **stock Gazebo rover**, not measured real chassis parameters.
- Nav2-to-PX4 obstacle-replanning closed loop and real receiver takeover remain separate unverified work.
- ROS2 multiroute test can arm **SITL** when explicitly invoked. Never point it at physical Cube; it requires a running PX4 SITL binary and an isolated `ROS_DOMAIN_ID=105`.
