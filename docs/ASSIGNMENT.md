# IASYS Roadwork Challenge 2026 — revised assignment

Master in Critical Computing Systems · Ricardo Severino

Build a C++17 ROS2 Jazzy system that interprets a STOP/GO panel, uses LiDAR and
radar to assess a roadwork bypass, plans a feasible route, controls the vehicle
and stops at the goal. Students implement **all subscriptions, internal ROS
interfaces, perception, decisions, planning, control and command publication**.
Use at least three ROS2 nodes in both deliveries; see ARCHITECTURE.md.

## Scope and permitted information

Provided: simulator, external contracts, embedded evaluator, RViz and minimal
node entry points. No Autoware, Gazebo, CARLA, GPU or neural-network training is
required. Use the course Ubuntu 24.04/Jazzy VM.

Allowed inputs: `/vehicle/odometry`, `/lidar/scan`, `/radar/detections`,
`/camera/image_raw`, `/infra/roadwork/beacon` and `/clock`. Do not use debug
markers, evaluator metrics, scenario ID, random seed, schedules, simulator object
lists or process memory to select actions. Published road geometry, vehicle
limits and the mission goal are allowed; obtain work-zone geometry from the
beacon and permission/obstacle evidence from sensors.

## Required behavior and observability

Classify STOP/GO/UNKNOWN with documented validity and temporal evidence. Process
LiDAR for obstacles/free space and radar for moving traffic, compensating ego
motion. Preserve source timestamps and explain measurement association. Missing
messages are not an empty road. Require current permission before entry; define
commitment and loss-of-beacon policies. Plan the left bypass and return; use
bounded closed-loop targets with stopping-distance and delay allowances.
Publish perception results, decision state and a planned path. Record enough
information to explain failures. A stationary vehicle does not solve a scenario.

## Scenarios and evaluator

S1: GO and static roadworks. S2: STOP until 10 s then GO. S3: opposing traffic.
S4: blank camera/no new beacon from 2 to 8 s. S5: temporary obstacle during the
crossing for 8 s. Exact geometry and schedules remain in SCENARIOS.md.

Road edges ±3.5 m; own-lane centre −1.75 m; start (5, −1.75); goal near
(80.2, −1.75). Wheelbase 2.4 m; collision radius 0.90 m; speed limit 4 m/s;
acceleration 1.5 m/s²; braking 3 m/s²; steering ±0.45 rad and slew 0.6 rad/s.
Dynamics/odometry 20 Hz; camera/LiDAR/radar 10 Hz; beacon 2 Hz with 1.2 s validity.
Timeout 180 s. Completion requires x ≥80, |y+1.75| <0.6, |yaw| <0.15 and speed
<0.15. No collision, off-road, entry, opposing-lane or speed violation is allowed.
The speed tolerance is 0.05 m/s. Invalid or >0.30 s old commands trigger simulator
braking; this is not a replacement for student safety logic. See INTERFACES.md.

Variations remain work_shift −2..2 m, traffic_speed 2..4 m/s and noise 0..0.05 m.
Seeds affect noise. All groups use the same combinations; withheld combinations
must stay inside the published bounds and existing five scenario classes.

## Deliveries

Groups of three; a fourth member requires prior agreement. Deadlines are on Moodle.

| Delivery | Scope and weight | Required evidence |
|---|---|---|
| D1 | **S1–S2, 30%** | Three-node pipeline; source/README; architecture, interfaces, decisions; six runs (2 scenarios × 3 seeds); report ≤4 pages; video ≤3 minutes; initial failure analysis |
| D2 | **S1–S5, 70%** | Extend the same pipeline; reproducible source; at least 15 default runs plus two geometry/traffic variations; baseline or ablation; English report ≤8 pages; video ≤5 minutes; contributions and reuse declarations |

Use LNCS format; page limits include figures and references. D2 explains changes
after D1 feedback. Radar yielding and S3–S5 recovery are assessed in D2. Include
logs/configurations and all failures. Demonstration and individual defence are
mandatory. Declare libraries, reused code and AI assistance; every member must
be able to explain and modify the implementation.

Report completion, violations, clearance, maximum speed, traversal/stopped time,
hardware, ROS version and visualization settings. Measure held-out image
recognition error, reaction latency and tracking error against your own path;
define each metric. Discuss exact odometry, panel crop, circular envelopes,
idealized radar and other limits of the model.

| Criterion | Weight | Marks |
|---|---:|---:|
| Architecture and ROS2 integration | 20% | 4/20 |
| Perception and information handling | 20% | 4/20 |
| Decision, planning, control and safe response | 25% | 5/20 |
| Experimental evaluation and critical analysis | 25% | 5/20 |
| Reproducibility, communication and individual defence | 10% | 2/20 |

Apply the rubric separately within each delivery's scope. Project grade =
**0.30 × D1 + 0.70 × D2**, each marked out of 20. The old Word brief has been
replaced by this authoritative version to avoid contradictory scaffold/Foxy rules.
