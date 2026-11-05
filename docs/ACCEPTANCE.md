# Jazzy acceptance checklist — v2

Portable tests do not establish ROS build, DDS behavior or RViz correctness.
The reported v1 Jazzy success is useful history, not v2 release acceptance.

1. Extract each ZIP independently in a clean Ubuntu 24.04/Jazzy workspace.
   Source Jazzy; run rosdep, `colcon build`, `colcon test` and
   `colcon test-result --verbose`. Save logs and dependency versions.
2. Student: launch S1. Confirm exactly three empty solution nodes, no publisher
   on `/vehicle/command`, a stationary vehicle, and working RViz displays.
3. Instructor: run `python3 scripts/run_ros_acceptance.py`. It checks the live
   graph and terminal JSON for S1–S5, and interrupts each pipeline stage in turn
   to verify speed falls to zero. Retain the generated `ros_validation` evidence.
4. Confirm odometry 20 Hz, scan/radar/camera 10 Hz and beacon 2 Hz except S4.
   Inspect `/solution/scene`, `/solution/plan`, `/solution/state` and
   `/solution/path`. Check original sensor stamps survive transport.
5. Run S4 and S5 in RViz. Add Path `/solution/path` if desired. Fixed frame `map`;
   LiDAR best-effort QoS; use simulation time. Record visual acceptance separately.
6. Repeat selected geometry/traffic/noise variations with visualization on/off.
   Record CPU, RAM and simulation-to-wall-time behavior on the student VM.
7. Confirm one command publisher, clean restarts and no debug-topic algorithm inputs.
   Reset the entire episode; restarting just Plan mid-crossing loses commitment.
8. Freeze approved ZIP hashes and record actual Jazzy outcomes before distribution.
