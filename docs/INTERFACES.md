# Interface contract

All distances are metres, angles radians and times seconds of simulation time. +x is forward along the road, +y left. World frame is `map`; vehicle/sensor frame is `base_link`. The pose is a rear-axle reference. For collision checks only, the vehicle is a disc of radius 0.90 m centred on that reference. It is NOT a realistic car footprint. Ego odometry is exact; localisation is outside scope.

## Topics

| Topic | Type | Rate | QoS |
| --- | --- | ---: | --- |
| `/vehicle/odometry` | `nav_msgs/msg/Odometry` | 20 Hz | SensorDataQoS |
| `/lidar/scan` | `sensor_msgs/msg/LaserScan` | 10 Hz | SensorDataQoS |
| `/radar/detections` | `roadwork_interfaces/msg/RadarDetections` | 10 Hz | SensorDataQoS |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | 10 Hz | SensorDataQoS |
| `/infra/roadwork/beacon` | `roadwork_interfaces/msg/RoadworkBeacon` | 2 Hz except outage | Reliable, volatile, depth 10 |
| `/vehicle/command` | `roadwork_interfaces/msg/VehicleCommand` | expected 20 Hz | Reliable, volatile, depth 10 |
| `/clock` | `rosgraph_msgs/msg/Clock` | 20 Hz | Reliable, volatile |
| `/solution/state` | `std_msgs/msg/String` | solution-defined (reference 20 Hz) | Reliable, volatile |
| `/world/markers` | `visualization_msgs/msg/MarkerArray` | 10 Hz when enabled | Debug only |
| `/challenge/metrics` | `std_msgs/msg/String` containing JSON | 2 Hz and terminal | Reliable, transient local, depth 1; evaluation only |

SensorDataQoS uses best-effort delivery. A reliable-only subscriber may not match these publishers. Use SensorDataQoS or a compatible profile. All timestamps come from `/clock`, never wall time. Students implement timestamping and must enable `use_sim_time=true` for ROS timers/`now()` logic. RViz does so already. The simulator's fixed-step wall timer intentionally does not.

## Dynamics and commands

Fixed step 0.05 s; wheelbase 2.4 m; speed command clamp 0..8 m/s; steering clamp +/-0.45 rad; maximum acceleration 1.5 m/s², braking 3 m/s², steering slew 0.6 rad/s. Nominal challenge speed limit is 4 m/s; >4.05 flags a violation. Thus the command clamp is NOT the speed limit. Reverse is outside scope.

Nonfinite commands, future timestamps or commands older than 0.30 s trigger braking to zero, holding current steering. There is no automatic obstacle avoidance. The watchdog is a simulator safeguard, not a replacement for student safety logic. Slight numerical timestamp error up to 1e-6 s is tolerated. Simulation time can run slower than real time under load; deadlines and metrics use simulation time.

## LiDAR

360 co-located rays, -pi inclusive to pi exclusive in 1-degree increments; 0.10..40 m. Nearest intersection with road edges or circular objects. No return = positive infinity. Additive bounded uniform range noise +/-`noise`, clipped to the valid interval. No 3D elevation, reflectivity, ground plane or ground segmentation. Transform each ray into map coordinates using the time-associated ego pose; consider the errors of using a newer pose with an older scan. The starter supplies no measurement collector. Students implement and justify measurement association.

## Radar

Object-level detections, range <=80 m, azimuth +/-60 degrees, all expressed in `base_link`. No object ID, class, ground-truth velocity or tracking is given. Range noise +/-3*`noise`, radial-velocity noise +/-`noise` m/s, angular noise +/-0.002 rad. Radial velocity is the range derivative: negative means approaching, including ego motion. The teaching radar does not model occlusion, multipath, clutter or RF processing. A stationary obstacle can have negative radial speed when the ego vehicle moves.

## Panel camera

96x64 RGB8, packed rows, frame name `panel_crop`; a cropped panel received from the infrastructure camera, not a perspective scene rendered from the vehicle. Red disk = STOP, green disk = GO, blank/ambiguous = UNKNOWN. Bounded +/-18 intensity variation; no label or confidence is published. The crop is available throughout the episode, independent of distance and heading. There is no CameraInfo or projection exercise in this version; perspective geometry from LAB6 is a possible future extension. A pretrained neural network is unnecessary. Evaluate classification plus temporal stability, particularly false GO detections.

## Beacon

Includes timestamp, 1.2 s validity, entry x, work start/end, exit x, speed limit and pass_side=+1 (left). The nominal work area is x=42..49, shifted together by work_shift. The entry is always x=25; exit is x=68+work_shift. Geometry can be cached. Before crossing the entry line, permission must be based on current visual evidence and a fresh beacon. After commitment, execute a documented policy using cached geometry and current obstacle evidence; blanket braking on a lost beacon is not required.

## Completion and violations

Start (5,-1.75), heading 0, at rest. Goal x>=80, |y+1.75|<0.6, |yaw|<0.15, speed<0.15. Nominal aiming point (80.2,-1.75). Timeout 180 s. First collision or road-boundary violation terminates the episode. Entry/speed/lane violations are latched but permit continued diagnostic execution.

Road y=-3.5..+3.5. Any part of the collision disc outside those lateral edges, or reference x<0, is off-road. Overlap/touch with an obstacle disc is a collision. Opposing-lane occupancy (ego y+0.9>0) is prohibited entirely before the entry region (ego x+0.9<25) and entirely beyond the exit region (ego x-0.9>exit_x).

First entry occurs when x+0.9>25. Crossing during STOP or the S4 visual/beacon blackout is an entry violation. The evaluator cannot infer whether your algorithm actually read a fresh sensor; assess that through logs and code review. A pass requires completion and no collision, off-road, entry, lane or speed violation. Clearance is measured disc-to-disc, not inferred from noisy sensors. Report both mission completion and safety violations rather than only the boolean pass flag.

## Topics used by the student solution

| Topic | Type | Direction | QoS |
| --- | --- | --- | --- |
| `/vehicle/odometry` | `nav_msgs/msg/Odometry` | sim → `sense` | SensorDataQoS |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | sim → `sense` | SensorDataQoS |
| `/lidar/scan` | `sensor_msgs/msg/LaserScan` | sim → `sense` | SensorDataQoS |
| `/radar/detections` | `roadwork_interfaces/msg/RadarDetections` | sim → `sense` | SensorDataQoS |
| `/infra/roadwork/beacon` | `roadwork_interfaces/msg/RoadworkBeacon` | sim → `sense` | Reliable, depth 10 |
| `/solution/scene` | `roadwork_student_interfaces/msg/Scene` | `sense` → `plan` | Reliable, volatile, depth 1 |
| `/solution/plan` | `roadwork_student_interfaces/msg/Plan` | `plan` → `act` | Reliable, volatile, depth 1 |
| `/vehicle/command` | `roadwork_interfaces/msg/VehicleCommand` | `act` → sim (sole publisher) | Reliable, depth 10 |
| `/solution/state` | `std_msgs/msg/String` (JSON) | `plan` → debug/logs | Reliable, depth 10 |
| `/solution/path` | `nav_msgs/msg/Path` (`map`) | `plan` → RViz | Reliable, transient local, depth 1 |
| `/solution/perception` | `visualization_msgs/msg/MarkerArray` (`map`) | `sense` → RViz | Reliable, depth 5 |

All five observation topics and `/clock` are used. Debug and evaluator topics
(`/world/markers`, `/challenge/metrics`, `/solution/perception`) never feed the
algorithm.

## Internal interfaces (`roadwork_student_interfaces`)

All times are simulation time; units SI; frame `map` unless stated. Depth 1 is
used because only the newest scene and plan are meaningful. Unset stamps (never
received) are encoded as time 0; validity flags or the receiver's own age checks
carry the meaning.

### `Scene` — `/solution/scene`, 20 Hz (on odometry; 50 ms fallback timer)

`header.stamp` = time Sense assembled the scene. Every evidence block keeps its
**original sensor stamp**, so Plan can judge source age with its own clock.

| Block | Fields | Meaning |
| --- | --- | --- |
| ego | `ego_valid`, `odom_stamp`, `ego_x/y/yaw/speed` | Ego state; `ego_valid` false if missing or older than `odom_timeout` (0.20 s) |
| panel | `panel_raw`, `panel_state` | Newest accepted frame class / filtered state: STOP immediately, GO after 3 consecutive frames, UNKNOWN if ambiguous or older than `camera_timeout` (0.30 s). `PANEL_UNKNOWN=0, PANEL_GO=1, PANEL_STOP=2` |
| | `camera_stamp`, `panel_green_fraction`, `panel_go_count`, `camera_rejected` | Panel diagnostics |
| beacon | `geometry_known`, `beacon_stamp`, `beacon_valid_for` | Cached newest valid beacon and its announced validity; Plan decides freshness |
| | `entry_x`, `work_start`, `work_end`, `exit_x`, `speed_limit`, `pass_side`, `beacon_rejected` | Cached work-zone geometry |
| radar | `radar_stamp`, `radar_pose_dt` | Stamp of the newest accepted scan; gap to the odometry sample used to transform it |
| | `radar_detections`, `radar_moving`, `radar_rejected` | Valid / moving (ego-compensated) detections in that scan; cumulative rejections |
| | `tracks[]` (`Track`) | Moving-object tracks (below) |
| LiDAR | `lidar_stamp`, `lidar_pose_dt` | Stamp of the newest accepted scan; association gap |
| | `lidar_returns`, `lidar_rejected` | Finite returns in that scan; cumulative rejected scans + malformed rays |
| | `obstacle_x[]`, `obstacle_y[]` (float32) | Map-frame returns of that scan with road-edge returns (\|y\| ≥ 3.25 m) removed — the evidence used by the corridor check |
| | `obstacles[]` (`Obstacle`) | Clusters of the same returns (observability) |

Plan rejects a scene older than `scene_timeout` (0.25 s), drops non-finite tracks
and treats mismatched obstacle arrays as missing LiDAR evidence.

### `Track` (inside `Scene`)

| Field | Meaning |
| --- | --- |
| `id` | Track id (unique per episode) |
| `stamp` | Time of the state below = last radar update (not predicted) |
| `x`, `y`, `vx` | Position at `stamp`; along-road velocity (negative = oncoming) |
| `hits` | Radar updates received |
| `confirmed` | `hits ≥ radar_confirm_hits`; Plan ignores unconfirmed tracks |
| `coasting` | Predicted outside the radar FOV/range and kept by constant-velocity prediction |

Consumers predict `x(t) = x + vx·(t − stamp)`.

### `Obstacle` (inside `Scene`)

`x`, `y` (centroid of the member returns), `radius` (max distance of a member from
the centroid), `range_min` (closest return to the sensor), `n_points`. The
centroid of a partially seen object lies on its visible surface.

### `Plan` — `/solution/plan`, 20 Hz (per scene; 50 ms watchdog hold plan)

| Field | Meaning |
| --- | --- |
| `header.stamp` | Plan creation time |
| `scene_stamp`, `odom_stamp` | Inputs actually used |
| `valid_until` | `stamp + plan_validity` (0.25 s) |
| `target_speed` (m/s, ≥0), `target_steering` (rad, + left) | Targets for Act |
| `speed_limit` | Cap Act must apply in addition to its own (`min(4, beacon) − 0.3`) |
| `hold` | true → Act commands speed 0 |
| `state`, `state_name`, `reason` | `INIT=0, WAIT_GO=1, APPROACH=2, BYPASS=3, RETURN=4, FINAL=5, GOAL=6, SAFE_STOP=7, YIELD=8, WAIT_INFO=9, BLOCKED=10` |

Act rejects a plan that is non-finite, future-stamped, past `valid_until` or older
than `max_plan_age` (0.30 s), and then commands speed 0 while holding steering.

### `/solution/state` JSON (20 Hz)

| Keys | Meaning |
| --- | --- |
| `t, state, reason, go` | Plan time, state, reason text, entry latched |
| `panel, go_count, camera_age` | Filtered panel, consecutive GO frames, camera age |
| `beacon_fresh, beacon_age` | Beacon freshness used for permission |
| `x, y, yaw, v, target_speed, steering` | Ego state and plan targets |
| `cte, path_s, v_path, v_goal, v_stop, v_yield, v_obs, d_line` | Tracking error to the own path, arc length, speed caps (−1 = no cap), front-to-entry-line distance |
| `gap_ok, radar_fresh, radar_age, n_tracks, conflict_id, conflict_x, conflict_vx, t_clear` | Traffic gap decision and the conflicting track |
| `lidar_fresh, lidar_age, n_obstacle_points, blocked, block_x, block_y, block_dist` | Corridor decision and the first blocking return |
| `scene_age, odom_age` | Input ages at plan time |

## Parameters (`roadwork_student/config/solution_params.yaml`)

| Node | Parameter | Default | Meaning |
| --- | --- | --- | --- |
| `sense` | `odom_timeout` / `camera_timeout` / `go_confirm_frames` | 0.20 s / 0.30 s / 3 | Ego validity, panel staleness, GO confirmation |
| `sense` | `max_assoc_dt` | 0.15 s | Max gap between a sensor stamp and the odometry used to transform it |
| `sense` | `radar_moving_threshold` / `radar_gate` | 0.6 m/s / 3.0 m | Moving-object threshold on the compensated radial velocity; association gate |
| `sense` | `radar_confirm_hits` / `radar_miss_limit` / `radar_max_coast` | 2 / 3 / 20 s | Track confirmation, deletion inside the FOV, coasting lifetime |
| `plan` | `bypass_y` / `cruise_speed` / `max_lateral_accel` | 1.25 m / 3.5 m/s / 1.5 m/s² | Path and speed profile |
| `plan` | `scene_timeout` / `plan_validity` / `stop_distance` | 0.25 s / 0.25 s / 1.0 m | Input deadline, plan expiry, stop-line gap |
| `plan` | `beacon_max_age` / `camera_timeout` | 1.2 s / 0.30 s | Beacon freshness cap; panel staleness (explanation only) |
| `plan` | `use_traffic_gap` | true | Ablation switch (launch argument `use_traffic_gap:=false`) |
| `plan` | `radar_timeout` / `gap_time` / `gap_margin` / `traffic_radius` | 0.35 s / 3.0 s / 1.0 m / 1.0 m | Gap check freshness, time margin, longitudinal margin, assumed vehicle radius |
| `plan` | `gap_v_nominal` / `gap_a_nominal` | 3.0 m/s / 1.0 m/s² | Conservative ego motion for the clearing time |
| `plan` | `use_obstacle_check` | true | Ablation switch (launch argument `use_obstacle_check:=false`) |
| `plan` | `lidar_timeout` / `corridor_half_width` / `obstacle_horizon` | 0.35 s / 1.3 m / 40 m | Corridor check |
| `plan` | `obstacle_standoff` / `obstacle_decel` / `obstacle_clear_time` | 2.5 m / 2.0 m/s² / 0.3 s | Stopping before a blocking return and release hysteresis |
| `act` | `max_speed` / `max_plan_age` | 3.8 m/s / 0.30 s | Hard speed cap; plan age limit |

All nodes force `use_sim_time=true` if it was not set.
