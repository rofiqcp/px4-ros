# Architecture

```text
Pixhawk/Cube (PX4 MAVLink)
          |
          v
    pixhawk_bridge ----> pixhawk_interfaces (typed ROS interfaces)
          |                          |
          +---- telemetry -----------+
          |                          |
          +---- opt-in commands      v
                       stock_px4_supervisor
                                 ^
                                 |
                   pixhawk_avoidance_adapter
                    (obstacle advisory input)
```

The bridge translates PX4's MAVLink frames and telemetry into ROS messages, with validity and age checks. Supervisory controllers and perception advisories must not silently enable arming or motion.

## Workspace layout

- `src/pixhawk_interfaces`: message/service contracts shared by the packages.
- `src/pixhawk_bridge`: MAVLink transport, guarded command interfaces, launch configuration.
- `src/stock_px4_supervisor`: safety gates, differential advisor, production tests (in `test/`).
- `src/pixhawk_avoidance_adapter`: obstacle input and collision-warning bridge.
- `config/`: checked-in sample/staged configuration, not raw flight controller parameter dumps.
- `scripts/`: maintained operator utilities; one-off experiments and reports remain local.
- `tools/`: portable build/readiness commands.

## Frame and safety assumptions

ROS uses ENU/FLU conventions; MAVLink PX4 may use NED/FRD. Never replace transformed orientation or position with raw MAVLink coordinates. The presence of MAVLink telemetry does not establish valid localization. Missing/expired quality signals must result in stop/withhold behavior, not continued commands.

Test hierarchy: syntax and lint; deterministic unit tests; ROS build; SITL with simulated sensors; hardware read-only diagnostics; explicitly authorized hardware motion testing. Report each separately.
