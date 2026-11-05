# External interface contract — retained from v1

All distances are metres, angles radians and times seconds of simulation time. +x is forward along the road, +y left. World frame is `map`; vehicle/sensor frame is `base_link`. The pose is a rear-axle reference. For collision checks only, the vehicle is a disc of radius 0.90 m centred on that reference. It is NOT a realistic car footprint. Ego odometry is exact; localisation is outside scope.

## Topics

| Topic | Type | Rate | QoS |
|---|---|---:|---|
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
