# Repository instructions for AI coding agents

Applies to the entire `px4-ros` repository. Prefer concise changes that retain existing ROS message/service contracts and safety behavior.

## Mandatory workflow
1. Inspect `README.md`, relevant package manifests, CMake targets, existing implementation and `git status` before changes.
2. Ensure branch `v1` tracks `origin/v1`; pull with `git pull --ff-only` only if the worktree is clean. Never overwrite local edits or force-push.
3. State what you will change and why. Edit the smallest possible set of maintained files. Do not copy directories or regenerate vendor code unnecessarily.
4. Use ROS 2 Humble, C++17, existing ament/colcon structure and stable public topics/services. Keep safety defaults fail-closed.
5. No hard-coded home directories, USB device paths, machine IP addresses, serial numbers or developer usernames in programs; resolve workspace root from script path and expose hardware settings as configuration.
6. Never run a command that flashes PX4, arms a vehicle, commands motion, writes parameters, changes network settings, or manipulates real actuators without an explicit separate request and safety checks.
7. Distinguish offline checks, SITL, hardware read-only tests, and energized physical tests. Do not claim tests passed unless they ran and show results.
8. Keep generated files, test experiments, captures, backups and dated research/audit notes out of Git. Put them in ignored `.local-only/`, `results/`, or `backups/`; **do not delete them unless requested**.
9. Update `README.md`, `docs/` and the relevant config samples for functional changes. Do not create many duplicate Markdown status reports at repo root.
10. Validate before committing: run `python3 tools/check_repo_hygiene.py`, `bash -n` on modified shells, `python3 -m py_compile` on modified Python files, and a targeted ROS build/unit test where available. Describe remaining untested areas.
11. Review `git diff --cached` and `git status`; commit only intended maintained files, then push `v1`. Synchronize the other PC with a fast-forward pull after checking its worktree.

## Architecture and boundaries

- `src/pixhawk_interfaces`: stable messages and services.
- `src/pixhawk_bridge`: MAVLink serial, telemetry, guarded commands.
- `src/stock_px4_supervisor`: supervisory safety and differential controller.
- `src/pixhawk_avoidance_adapter`: perception/obstacle advisories.
- `avoidance_core`: shared core logic.
- `tools/`: production maintenance tooling only, not ad-hoc migration scripts.
- `src/.../test/`: maintained deterministic safety regression tests are allowed.
- `simulation/`, `scripts/test_*.py` and dated one-off experiments: local only.

## Clean coding rules

No dead code, unused imports, speculative fallback behavior, unreviewed bulk renames or blanket changes to CMake/vendor sources. Use explicit error handling, bounds checks and meaningful names. Never bury an unsafe control change in documentation cleanup. Keep commands idempotent where possible. Preserve third-party license notices. Never commit secrets, data dumps, hardware logs or telemetry. Do not use `git add -f` to override ignore policy.

## PX4 calibration and ROS-only controls

11. For PX4 calibration, prefer local read-only dashboard and MAV_CMD_RUN_PREARM_CHECKS (401); never expose an ARM HTTP endpoint or bypass checks. Sensor calibration (241) must require both independent opt-ins, disarmed PX4 and isolated motors.
12. No RC receiver is installed: a fresh ROS operator heartbeat and independent physical E-stop must be verified before actuator authority, with defaults fail-closed. COM_RC_IN_MODE must not be changed on hardware during docs or web tasks.
