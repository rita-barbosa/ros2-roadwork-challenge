# IASYS Roadwork Challenge — Student v2 / ROS2 Jazzy

Implement the complete sense–plan–act chain in C++17 using **at least three ROS2
nodes**. This workspace supplies the simulator and embedded evaluator, external
message contracts, RViz configuration, launch files and three empty node entry
points. No sensor subscriptions, internal messages, control publisher or solution
algorithms are supplied in the starter. There is no `ros_runner.hpp`.

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

The starter is intentionally inactive: the simulator watchdog holds the vehicle
stationary because no command publisher exists. This is not a completed solution.
The three empty processes are `sense_node`, `plan_node` and `act_node` in
`roadwork_student`. Add subscriptions, internal ROS messages, algorithms,
publishers, dependencies, tests and observable outputs yourself.

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
