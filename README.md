# Vista-Tracker

Drone target-tracking simulation with a layered perception → estimation → planning → control architecture. Designed to swap in AirSim, an EKF, and an MPC planner without touching interfaces.

---

## Build

```bash
cd cpp
mkdir build && cd build
cmake ..
make -j$(nproc)
```

---

## Run

```bash
# Default config
./tracker

# List available scenarios
./tracker --list

# Run a named scenario
./tracker go_around
./tracker buildings
./tracker corridor

# Run any YAML file directly
./tracker ../../config/scenarios/go_around.yaml
```

---

## Scenarios

Scenario files live in `config/scenarios/`. Each one declares a `base: ../config.yaml` and only overrides what differs — drone start, world bounds, obstacles, and target trajectory. Shared params (`dt`, `horizon`, `desired_distance`) are inherited from the base.

| Scenario | Description |
|---|---|
| `go_around` | Person approaches a large obstacle, detours around it, resumes course |
| `buildings` | L-shaped urban route through building footprints, with a corner slow-down and junction stop |
| `corridor` | Forest-like tree corridor — cautious entry, sprint through open section, narrow exit |

### Adding a scenario

Create `config/scenarios/my_scenario.yaml`:

```yaml
base: ../config.yaml

drone_init:
  x: 0.0
  y: 0.0
  z: 2.0

sim:
  T: 15.0

target_trajectory:
  type: person          # person | bicycle | car
  max_accel: 1.0
  max_speed: 2.0
  max_lateral_accel: 4.0
  loop: false
  waypoints:
    - pos: [0.0, 0.0, 0.0]
      speed: 1.5
    - pos: [10.0, 0.0, 0.0]
      speed: 0.0
      hold: 2.0         # stop for 2 s
    - pos: [20.0, 0.0, 0.0]
      speed: 2.0

world:
  grid:
    x_min: -5.0
    x_max: 25.0
    y_min: -5.0
    y_max:  5.0
  obstacles: []
```

Then run it immediately:

```bash
./tracker my_scenario
```

### Trajectory knobs

| Field | Effect |
|---|---|
| `max_accel` | How fast the target speeds up / brakes (m/s²) |
| `max_speed` | Hard cap on waypoint speeds (m/s) |
| `max_lateral_accel` | Controls turning radius — person ≈ 4.0, car ≈ 3.0 (m/s²) |
| `waypoint.speed` | Desired speed approaching that waypoint |
| `waypoint.hold` | Seconds to stop at that waypoint before continuing |

---

## Visualise

```bash
cd python
python3 visualize.py
```

Or use the convenience script:

```bash
cd scripts
./run.sh
```

---

## Architecture

```
SensorData
    │
IPerception          (GroundTruthPerception → CameraPerception)
    │ Detection
ITargetEstimator     (PerfectEstimator → EKF / UKF)
    │ TargetEstimate  { horizon[0..N] }
IPlanner             (SimplePlanner → MPCPlanner)   ← IESDFMap
    │ Reference
IController          (PIDController → MPCController)
    │ ControlCommand
ISimulator           (KinematicSim → AirSim / Gazebo)
```

Each layer is a pure interface. Concrete implementations are swapped via config or compile-time substitution — no changes to `main.cpp` or adjacent layers required.
