# Common challenge scenarios

All five scenarios are mandatory. They are separate episodes, not five simultaneous hazards. Defaults below apply to every group. Road y=+/-3.5; own lane centre -1.75; opposing centre +1.75. Works are two circles of radius 1.0 at (43+shift,-1.75) and (47+shift,-1.75).

| ID | Scenario | Exact event definition | Required behaviour |
|---|---|---|---|
| 1 | Nominal | GO throughout; static works | Pass left, return to own lane and stop at goal |
| 2 | Stop then go | STOP for t<10 s; GO from t=10 s | Stop before front crosses x=25, then resume |
| 3 | Opposing traffic | GO; another disc radius .9 starts at (82,1.75), moves at -traffic_speed along x and disappears at x<=-5 | Yield using radar/perception; cross only with a safe gap |
| 4 | Missing information | GO normally; blank camera and no new beacon for 2<=t<8 s | Detect unknown/stale evidence; wait before entry; resume after recovery |
| 5 | Obstacle during traversal | At first ego x>=45+shift, radius .65 obstacle appears at (57+shift,1.75); remains 8 s then disappears | Detect, brake, wait or choose a justified safe response, then finish |

The S5 obstacle appears at least 12 m ahead of the ego reference at the trigger. At the 4 m/s speed limit, ideal stopping distance at 3 m/s² is 2.67 m, before sensing and command delay margins. Students must still account for their own control delay and geometry. The event is artificial and explicitly designed to be avoidable within the published bounds; it does not model a realistic pedestrian trajectory.

## Training and assessment

Publish the default suite and use the same assessment parameter combinations for every group. Permitted variations: work_shift in [-2,2] m, traffic_speed in [2,4] m/s and noise in [0,.05] m; seed varies the deterministic noise. Thresholds, event semantics, road width, vehicle dynamics, camera dimensions and message contracts do not change. Teacher may withhold selected seeds/parameter combinations, but not scenario classes or bounds. Do not use absolute time or the scenario number to trigger a manoeuvre. The event times are specified for reproducibility and analysis, not as algorithm inputs.

For the report: all five defaults, at least three noise seeds per scenario, plus at least two geometry/traffic variations. Include failed runs. Repeated seeds with a fixed synchronous core are deterministic; ROS scheduling may introduce small differences. Record distro, hardware and whether visualization was enabled. Compare your final method to one deliberately simpler baseline (e.g. no temporal confirmation or no radar-based gap check), in simulation only.

## Metrics

Automatic JSON: completion, collision, offroad, entry_violation, lane_violation, speed_violation, timeout, minimum obstacle clearance, maximum speed, stopped time and elapsed simulation time. `rejected_commands` also counts startup without a valid command; distinguish this from runtime failures.

Student measurements: sign confusion matrix with independently generated or held-out image samples; reaction latency; tracking error relative to the trajectory actually planned; unnecessary stops; state transitions and reasoning. The evaluator does not automatically produce these measures. Explain how you measured them. Do not claim the simulated camera scores establish real-world recognition accuracy.
