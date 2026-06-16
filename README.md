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

### Config layout

```
config/
  base.yaml          ← algorithm params only (dt, gains, horizon)
  config.yaml        ← default standalone scenario (used by ./tracker)
  scenarios/
    go_around.yaml   ┐
    buildings.yaml   ├─ each declares "base: ../base.yaml" and owns
    corridor.yaml    ┘  drone_init, world, trajectory, and sim.T
```

**`base.yaml`** owns what is scenario-independent: `sim.dt`, `estimator.*`, `controller.*` defaults.
**Scenario files** own the environment: `drone_init`, `world`, `target_trajectory`, `sim.T`.
Scenarios can still override any `base.yaml` field (e.g. tighter PID gains for a fast-moving target).

### Adding a scenario

Create `config/scenarios/my_scenario.yaml`:

```yaml
base: ../base.yaml

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

Or use the convenience script (builds, simulates, and visualises in one step):

```bash
./scripts/run.sh                # default config
./scripts/run.sh go_around      # named scenario
./scripts/run.sh --list         # list available scenarios
./scripts/run.sh --help         # usage
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
