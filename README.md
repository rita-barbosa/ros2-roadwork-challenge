# IASYS Roadwork Challenge — Student v2 / ROS2 Jazzy

Implement the complete sense–plan–act chain in C++17 using **at least three ROS2
nodes**. This workspace supplies the simulator and embedded evaluator, external
message contracts, RViz configuration, launch files and a three-node student
solution. The current D1 implementation handles STOP/GO panel evidence and the
static-work bypass for S1–S2. LiDAR, radar and S3–S5 recovery behavior remain
outside the implemented D1 scope. There is no `ros_runner.hpp`.

## Build and launch (Ubuntu 24.04 / ROS2 Jazzy)

```bash
source /opt/ros/jazzy/setup.bash
cd IASYS_Roadwork_Student_v2
bash scripts/check_environment.sh
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
colcon build --symlink-install
source install/setup.bash
ros2 launch roadwork_sim challenge.launch.py scenario:=1
```

The three solution processes are `sense_node`, `plan_node` and `act_node` in
`roadwork_student`. Sense classifies the panel, requiring three consecutive GO
frames and treating stale camera evidence as UNKNOWN. Without a confirmed GO
(STOP, UNKNOWN or stale evidence), Plan drives in its own lane and stops with the
vehicle front `stop_distance` (default 1.0 m) before the entry line. GO is
re-checked every cycle and latched only at the point where the vehicle could no
longer stop before the line; after that Plan follows the bypass.
Act validates and executes short-lived plans and is the only publisher of
`/vehicle/command`.

```bash
# Headless episode with a unique result file:
ros2 launch roadwork_sim challenge.launch.py scenario:=5 seed:=42 rviz:=false visualize:=false result_path:=/tmp/team_s5_seed42.json
# To launch your own packages separately, disable all three starter nodes:
ros2 launch roadwork_sim challenge.launch.py student:=false
```

Every episode requires a fresh launch. Only one node may publish
`/vehicle/command`. Use an isolated `ROS_DOMAIN_ID` per group. Source only Jazzy
in the terminal. All solution nodes must use `use_sim_time=true`.

**D1: S1–S2, 30%. D2: S1–S5, 70%.** Both require the complete three-node
architecture, with functionality assessed within the delivery's scenario scope.
The rubric and submission limits are in [ASSIGNMENT.md](docs/ASSIGNMENT.md).

Read [architecture](docs/ARCHITECTURE.md), [implementation](docs/IMPLEMENTATION.md),
[external interfaces](docs/INTERFACES.md), [unchanged scenarios](docs/SCENARIOS.md)
and [Jazzy acceptance](docs/ACCEPTANCE.md).

## Release status

The simulator is an educational 2D model with exact odometry and circular vehicle
collision geometry, not evidence of real-world autonomous-vehicle safety.

## Student solution (D1 in progress, S1–S2 scope)

| Package | Content |
| --- | --- |
| `roadwork_student_interfaces` | Internal contracts: `Scene`, `Plan` |
| `roadwork_student` | `sense_node`, `plan_node`, `act_node`; ROS-free algorithm headers in `include/roadwork_student/`; tests in `test/` |

```bash
# Supplied launch:
ros2 launch roadwork_sim challenge.launch.py scenario:=1
# Solution launch (parameter file + RViz with /solution/path):
ros2 launch roadwork_student solution.launch.py scenario:=1 seed:=1
# D1 evidence: S1 and S2, 3 seeds each, fresh launch each:
bash scripts/run_episodes.sh results/d1 "1 2" "1 2 3"
```
