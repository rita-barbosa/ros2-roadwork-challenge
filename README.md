# IASYS Roadwork Challenge — student solution (ROS2 Jazzy, C++17)

A three-node sense–plan–act pipeline that drives the simulated vehicle through the
roadwork zone in all five scenarios: it interprets the STOP/GO panel, checks
beacon, radar and LiDAR evidence before entering, yields to opposing traffic,
passes the works on the left, brakes for an obstacle that appears during the
crossing, returns to its lane and stops at the goal. The simulator, evaluator and
external message contracts are the supplied Student v2 workspace (unchanged).

- **D1** (S1–S2) is preserved as git tag `d1-submission`, evidence in `results/d1/`.
- **D2** (S1–S5) is the current `main`, evidence in `results/d2/` (overview in
  [`results/d2/SUMMARY.md`](results/d2/SUMMARY.md)).

## Status

All final-method ROS runs pass with no collision, off-road, entry, lane or speed
violation: **15/15 default runs** and **54/54 variation runs** (work_shift ±2,
traffic_speed 2/4, noise 0.05); the offline closed-loop suite passes 378/378.
Numbers below are mean values over seeds 1–3 of the default set
([`results/d2/SUMMARY.md`](results/d2/SUMMARY.md)).

| Scenario | Behaviour | D2 evidence (default set) |
| --- | --- | --- |
| S1 nominal | Bypass on the left, return to own lane, stop at goal | 3/3, 25.0 s, min clearance 1.17 m, 0 unexplained stops |
| S2 STOP → GO | Approaches and holds 1 m before the entry line while STOP/UNKNOWN/stale; enters after 3 consecutive GO frames | 3/3, 29.6 s, GO confirmed 0.22 s after the event |
| S3 opposing traffic | Radar tracks the oncoming car; YIELD at the line until it has passed the opposing-lane interval; then bypass | 3/3, 37.6 s, ≈17 s yielding, 1.69 m to the car |
| S4 missing information | Blank camera → WAIT_GO, stale beacon detected at t = 2.7 s → no entry; resumes when fresh GO and beacon return | 3/3, 27.2 s, resumes 0.20 s after recovery |
| S5 obstacle during traversal | LiDAR corridor check detects it on the next scan, BLOCKED: brakes to a 2.5 m standoff, waits, resumes 0.3 s after it disappears | 3/3, 32.3 s, detected within one scan, 1.97 m to the obstacle |

Ablations (same seeds): without the radar gap check S3 collides 3/3; without the
LiDAR corridor check S5 collides 3/3. Offline, the no-gap variant survives only at
traffic_speed 4 m/s, where the fast car is already inside the 40 m LiDAR range
when the vehicle reaches the line, so the corridor check alone holds it; at 2–3 m/s
the car is still out of LiDAR range at commitment. Tracking error to the planned
path is 0.06–0.09 m mean and ≤ 0.30 m max (the maximum occurs in the lane
changes).

## Requirements

Ubuntu 24.04, ROS2 Jazzy (`ros-jazzy-desktop` or `ros-jazzy-ros-base` + `rviz2`),
`colcon`, `rosdep`, a C++17 compiler. For recording and analysing runs:
`ros-jazzy-rosbag2-storage-mcap` (installed with the desktop variant). No GPU,
Gazebo, CARLA or Autoware.

## Build from a clean workspace

Open a terminal in the workspace root (the folder containing `src/` and
`scripts/`) and source **only** ROS2 Jazzy:

```bash
source /opt/ros/jazzy/setup.bash
bash scripts/check_environment.sh
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
colcon build --symlink-install
source install/setup.bash
```

Packages: `roadwork_core` / `roadwork_sim` / `roadwork_interfaces` (supplied
simulator, evaluator and external contracts), `roadwork_student_interfaces`
(our internal `Scene`, `Track`, `Obstacle`, `Plan` messages) and
`roadwork_student` (`sense_node`, `plan_node`, `act_node`, ROS-free algorithm
headers, parameters, launch file, RViz config, tests).

## Run an episode

Every episode needs a **fresh launch** (Ctrl+C to stop). Use an isolated
`ROS_DOMAIN_ID` per group and never run two simulator episodes in the same domain.

```bash
export ROS_DOMAIN_ID=37   # any free id
# Simulator + the three solution nodes (with the parameter file) + RViz:
ros2 launch roadwork_student solution.launch.py scenario:=3 seed:=1
# Headless, with a unique evaluator result file:
ros2 launch roadwork_student solution.launch.py scenario:=5 seed:=2 rviz:=false visualize:=false \
  result_path:=/tmp/team_s5_seed2.json
```

| Launch argument | Default | Meaning |
| --- | --- | --- |
| `scenario` | 1 | 1–5 (`docs/SCENARIOS.md`) |
| `seed` | 1 | Noise seed |
| `work_shift` | 0.0 | Work-zone shift, −2…2 m |
| `traffic_speed` | 3.0 | S3 opposing vehicle speed, 2…4 m/s |
| `noise` | 0.02 | Sensor noise, 0…0.05 m |
| `rviz` / `visualize` | true / true | RViz window / simulator debug markers |
| `result_path` | `roadwork_result.json` | Evaluator terminal JSON |
| `use_traffic_gap` | true | `false` = ablation without the radar gap check |
| `use_obstacle_check` | true | `false` = ablation without the LiDAR corridor check |

The RViz configuration (`src/roadwork_student/rviz/solution.rviz`) shows the
world, LiDAR, the planned path (`/solution/path`), Sense's LiDAR clusters and
radar tracks (`/solution/perception`) and the panel image. Decisions are logged on
the console (state transitions with reasons, panel/beacon/gap/corridor events) and
published as JSON on `/solution/state`:

```bash
ros2 topic echo /solution/state --field data
```

The starter launch `ros2 launch roadwork_sim challenge.launch.py scenario:=1`
also starts the three nodes, but with built-in defaults instead of the parameter
file; `student:=false` starts the simulator alone.

## Configuration

All parameters live in
[`src/roadwork_student/config/solution_params.yaml`](src/roadwork_student/config/solution_params.yaml)
(same values as the C++ defaults) and are documented in
[`docs/INTERFACES.md`](docs/INTERFACES.md#parameters-roadwork_studentconfigsolution_paramsyaml).
With `--symlink-install` an edited YAML takes effect at the next launch without
rebuilding. The D2 evidence uses the file as committed; variations only change
launch arguments.

## Tests

```bash
colcon test --packages-select roadwork_student && colcon test-result --verbose
```

Three ROS-free executables: `test_perception` (incl. a confusion matrix on
held-out generated panel images), `test_planning` and `test_closed_loop` (378
closed-loop runs of Sense→Plan→Act against the supplied simulator core, S1–S5 ×
seeds × work_shift × noise × extra command delay × traffic speed). Single
offline episodes are useful for debugging:

```bash
B=build/roadwork_student
$B/test_closed_loop 3 1                          # S3 seed 1 (defaults)
$B/test_closed_loop 5 2 -2.0 0.05 2 3.0          # S5 seed 2, work_shift -2, noise 0.05, +0.1 s delay
$B/test_closed_loop 3 1 0 0.02 0 4 --no-gap      # ablation, traffic 4 m/s
$B/test_closed_loop 5 1 --trace=/tmp/s5.csv      # per-tick CSV trace (state, speeds, reasons)
```

## Reproduce the D2 evidence

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
export ROS_DOMAIN_ID=37
bash scripts/run_d2.sh                      # all sets, ~50 min wall time (real-time simulator)
bash scripts/run_d2.sh default ablation_no_gap   # or selected sets
```

| Set (`results/d2/…`) | Runs | Arguments |
| --- | --- | --- |
| `default/` | S1–S5 × seeds 1–3 (15) | defaults |
| `var_shift_m2/`, `var_shift_p2/` | S1–S5 × seeds 1–3 (15 each) | `work_shift:=-2.0` / `2.0` |
| `var_traffic_2/`, `var_traffic_4/` | S3 × seeds 1–3 (3 each) | `traffic_speed:=2.0` / `4.0` |
| `var_noise_005/` | S1–S5 × seeds 1–3 (15) | `noise:=0.05` |
| `ablation_no_gap/` | S3 × seeds 1–3 | `use_traffic_gap:=false` |
| `ablation_no_obstacle/` | S5 × seeds 1–3 | `use_obstacle_check:=false` |
| `offline/` | unit tests, closed-loop suite, offline ablations | ROS-free |

Each run folder `s<S>_seed<N>/` holds `result.json` (evaluator), `run.json`
(software version, arguments, visualisation off, wall time), `launch.log` (node
logs), `metrics.json` (measurements from the bag: tracking error to our own
path, entry time, S2/S4 latencies, S3 yield and clearance to the car, S5
detection latency, brake onset, stop position and obstacle clearance, command
rate; definitions in [`scripts/analyze_bag.py`](scripts/analyze_bag.py)) and
`bag/` (MCAP, not committed). `environment.txt` records date, software version,
ROS distro, CPU and memory per set; `summary.md` is the per-set table;
`results/d2/SUMMARY.md` is the combined report (`scripts/d2_report.py`).

Single sets can be run directly:
`EXTRA_ARGS="work_shift:=-2.0" bash scripts/run_episodes.sh results/tmp "1 5" "1 2"`.

Notes on the committed evidence: it was produced from a clean checkout of
commit `63c8bf9` (recorded as `d1-submission-4-g63c8bf9` in `run.json` and
`environment.txt`), so the raw `launch.log` files name that checkout's
`result_path`. The simulator clock starts before the solution nodes are up, so
first motion varies by up to ≈ 0.5 s between runs. `rejected_commands` counts
startup ticks only: in all 72 recorded runs the largest gap between consecutive
`/vehicle/command` messages from 0.5 s before first motion onward is 0.10 s (the
simulator rejects commands older than 0.30 s); the only longer gaps follow a
command Act stamped t = 0 before `/clock` arrived.

### Replay a recorded episode in RViz

Bags contain the sensor, internal and command topics but not `/clock` or `/tf`;
`scripts/replay_tf.py` rebuilds both from the recorded odometry:

```bash
ros2 bag play results/d2/default/s5_seed1/bag &
python3 scripts/replay_tf.py &
rviz2 -d src/roadwork_student/rviz/solution.rviz --ros-args -p use_sim_time:=true
```

## Repository layout

```
src/roadwork_student/            three nodes (src/), algorithms (include/roadwork_student/),
                                 config/, launch/, rviz/, test/
src/roadwork_student_interfaces/ Scene, Track, Obstacle, Plan
src/roadwork_{core,sim,interfaces}/  supplied simulator, evaluator, external contracts (unchanged)
scripts/                         run_episodes.sh, run_d2.sh, analyze_bag.py, summarize_results.py,
                                 d2_report.py, replay_tf.py, check_environment.sh
results/d1/, results/d2/         evidence (D1: S1–S2; D2: S1–S5, variations, ablations)
docs/                            assignment, scenarios, interfaces, architecture, D1 report
```

## Changes since D1

- **Sense** now uses all five observation topics: radar (ego-motion compensation,
  moving-object tracks with coasting outside the FOV) and LiDAR (map-frame
  projection with the pose at the scan stamp, road-edge removal, clustering).
  Each sensor sample is associated with the odometry at its own stamp.
- **Plan**: entry permission also requires a fresh beacon, a radar-certified
  traffic gap and a clear LiDAR corridor; new states YIELD, WAIT_INFO and
  BLOCKED; documented information-loss policy before/after commitment; obstacle
  braking with standoff during traversal; ablation switches.
- **Nodes** reset all episode state when `/clock` jumps backwards.
- **Evidence tooling**: bags are now finalised (D1 bags lacked `metadata.yaml`
  because the recorder never received SIGINT), bag-based measurements, combined
  D2 report.

## Documentation

[Architecture and decision model](docs/ARCHITECTURE.md#implemented-architecture-d2-s1s5) ·
[interfaces, topics, QoS, parameters](docs/INTERFACES.md#topics-used-by-the-student-solution) ·
[scenarios](docs/SCENARIOS.md) · [assignment](docs/ASSIGNMENT.md) ·
[acceptance checklist](docs/ACCEPTANCE.md) · [D1 report](docs/report_d1_1220841_1220962.pdf)

Only `act_node` publishes `/vehicle/command`. All solution nodes run on
simulation time. The simulator is an educational 2D model with exact odometry and
circular collision geometry; simulated success is not evidence of real-world
autonomous-vehicle safety (see the limitations in
[ARCHITECTURE.md](docs/ARCHITECTURE.md#model-limitations-and-residual-risks)).
