# PX4–ROS 2 bridge for Cube/Pixhawk

ROS 2 Humble / C++17 workspace for MAVLink telemetry, rover supervision, differential-drive guidance, and an optional perception/avoidance advisory bridge. This repository contains **ROS software, not PX4 firmware**. The PX4 firmware project is maintained separately in [px4-firmware](https://github.com/rofiqcp/px4-firmware).

## Supported scope

| ROS 2 package | Purpose |
| --- | --- |
| `pixhawk_interfaces` | Typed MAVLink-facing messages and service definitions |
| `pixhawk_bridge` | MAVLink serial telemetry and opt-in command bridge |
| `stock_px4_supervisor` | Safety supervision and differential guidance |
| `pixhawk_avoidance_adapter` | Perception/obstacle advisory input |

The project targets Ubuntu 22.04 and ROS 2 Humble. MAVLink headers under `src/pixhawk_bridge/third_party/mavlink` are vendored dependencies; do not regenerate them during routine cleanup. `avoidance_core/` contains reusable safety code.

## Checkout locations

| Machine | Source path |
| --- | --- |
| Sirobo | `/media/sirobo/HDD/Project/px4-ros` |
| MiniPC1 | `/home/otomasi/Project/px4-ros` |

Source paths are machine-specific; build artifacts are kept outside Git, in a local cache. The build wrapper uses a conservative single-worker, memory-checked colcon configuration.

## Build and inspect (no firmware flash)

```bash
cd /path/to/px4-ros
./build.sh
source ./env.sh
./verify.sh
```

`verify.sh` checks installed ROS packages; it does **not** certify vehicle safety, connection to real hardware, or a successful mission. For a connected device, first run safe telemetry-only checks, then review the configured serial device and PX4 estimator status.

```bash
source ./env.sh
ros2 launch pixhawk_bridge pixhawk.launch.py
# In another terminal with the same environment:
ros2 topic echo --once /pixhawk/status
```

Edit `src/pixhawk_bridge/config/default.yaml` and launch settings to match the device. Inspect `src/stock_px4_supervisor/config/` for safety and differential parameters. The script `./run.sh` launches the bridge after sourcing the locally built workspace.

## Safety contract

- **No flash, arming, parameter writes, flight-mode changes, or motor actuation as part of installation or Git synchronization.**
- Launch defaults must remain restrictive: `enable_motion_commands=false`, `allow_mode_changes=false`, and `allow_parameter_writes=false`.
- Do not assume GPS, local pose, RC failsafe, or OFFBOARD safety is available without explicit live verification.
- A stopped MAVLink setpoint stream is not a physical emergency stop. Preserve a hardware kill mechanism, RC takeover, disarm and timeout/failsafe protections.
- Separate **simulation**, **read-only hardware checks**, and **live actuation** in all logs and documentation; none should be presented as equivalent.

See [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) and [docs/DEVELOPMENT.md](docs/DEVELOPMENT.md) for package boundaries and maintenance checks.

## Repository policy

Only maintained source, ROS configuration, dependency files, documentation and required automated unit tests belong on GitHub. Temporary SITL/Gazebo experiments, probes, scripts named `test_*.py`, backup scripts for one-off hardware trials, captured parameters, telemetry, audit snapshots and local outputs stay on each computer under ignored paths. The optional CMake unit tests inside `src/stock_px4_supervisor/test/` are intentionally retained to guard safety logic.

Read [AGENTS.md](AGENTS.md) before making AI-assisted changes and [SYNC_GUIDE.md](SYNC_GUIDE.md) before switching computers.
