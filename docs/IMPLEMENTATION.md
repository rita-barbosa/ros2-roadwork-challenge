# Implementation milestones

## Both deliveries

1. Inspect external message definitions and QoS; build the empty three-node starter.
2. Design internal scene and plan messages in a student-owned ROS interface package.
3. Implement every sensor subscription and publish meaningful perception results.
4. Subscribe to the scene in Plan; implement explicit decisions, path and speed.
5. Subscribe to the plan in Act; implement control and `/vehicle/command` publication.
6. Enable simulation time and check source age, future stamps, malformed values,
   repeated frames, missing streams and internal-message deadlines.
7. Publish state and path; test node loss and use recorded intermediate messages
   to explain a failure. Keep only one command publisher.

## D1 — S1–S2, 30%

Demonstrate the entire three-node chain with valid GO/STOP handling, safe entry,
LiDAR clearance, bypass, return and final stop. Submit six runs and the D1 material
specified in ASSIGNMENT.md. S3–S5 behavior is not assessed at D1.

## D2 — S1–S5, 70%

Extend the same pipeline with ego-compensated radar yielding, dropout recovery
and obstacle braking during traversal. Submit 15 default runs, two variations,
an ablation and the complete D2 evidence. Explain improvements since D1.

The starter contains no observation collector or ready-made transport wrapper.
Students choose and implement measurement association. Document the errors of
latest-sample projection if used. An expired GO frame must not remain permission.
