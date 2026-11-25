#!/usr/bin/env python3
"""Node-loss test: kill one pipeline node mid-crossing, measure the stop, restart it.

Usage (workspace root, after `source install/setup.bash`, isolated ROS_DOMAIN_ID):
  python3 scripts/node_loss_test.py OUT_DIR NODE [--scenario 1] [--seed 1]
          [--kill-at 12.0] [--down 5.0] [--wall-timeout 240]
  NODE: sense | plan | act

One fresh episode (headless solution.launch.py). At simulation time --kill-at
the NODE process is stopped with SIGKILL (no clean shutdown, like a crash); it is
restarted --down seconds later with the same parameter file (`ros2 run`). The
monitor (this script, observation only) records odometry, commands and the
decision state and writes OUT_DIR/node_loss.json:
  kill: time, x, y, speed and state at the kill
  stop: time from the kill until v < 0.1 m/s, distance travelled, lateral drift
  commands_after_kill: who kept commanding (count, max speed commanded)
  states_after_kill: decision states/reasons seen while the node was down
  restart: time of restart, first motion after it
  evaluator: terminal result.json (completion and violations)
Expected (docs/ARCHITECTURE.md, information-loss table): sense lost -> Plan's
scene deadline -> SAFE_STOP hold plans -> Act commands 0; plan lost -> Act's plan
expiry -> Act commands 0, holds steering; act lost -> no commands -> simulator
watchdog brakes after 0.30 s holding steering.
"""
import argparse
import json
import math
import os
import pathlib
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from roadwork_interfaces.msg import VehicleCommand
from ament_index_python.packages import get_package_share_directory

EXE = {"sense": "sense_node", "plan": "plan_node", "act": "act_node"}


class Monitor(Node):
    def __init__(self):
        super().__init__("node_loss_monitor")  # reads /clock directly; observation only
        self.t = 0.0
        self.odom = []    # (t, x, y, v)
        self.cmds = []    # (stamp, speed)
        self.states = []  # (t, state, reason)
        self.create_subscription(Clock, "/clock", self.on_clock, 10)
        self.create_subscription(Odometry, "/vehicle/odometry", self.on_odom, qos_profile_sensor_data)
        self.create_subscription(VehicleCommand, "/vehicle/command", self.on_cmd, 10)
        self.create_subscription(String, "/solution/state", self.on_state, 10)

    def on_clock(self, m):
        self.t = m.clock.sec + 1e-9 * m.clock.nanosec

    def on_odom(self, m):
        t = m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec
        self.odom.append((t, m.pose.pose.position.x, m.pose.pose.position.y, m.twist.twist.linear.x))

    def on_cmd(self, m):
        self.cmds.append((self.t, m.speed))

    def on_state(self, m):
        try:
            s = json.loads(m.data)
            self.states.append((s["t"], s["state"], s["reason"]))
        except (ValueError, KeyError):
            pass


def pids_of(exe):
    out = subprocess.run(["pgrep", "-f", f"roadwork_student/{exe}"], capture_output=True, text=True).stdout
    return [int(p) for p in out.split()]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("node", choices=list(EXE))
    ap.add_argument("--scenario", type=int, default=1)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--kill-at", type=float, default=12.0)
    ap.add_argument("--down", type=float, default=5.0)
    ap.add_argument("--wall-timeout", type=float, default=240.0)
    a = ap.parse_args()
    out = pathlib.Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    result = (out / "result.json").resolve()
    result.unlink(missing_ok=True)
    params = os.path.join(get_package_share_directory("roadwork_student"), "config", "solution_params.yaml")

    rclpy.init()
    mon = Monitor()
    launch = subprocess.Popen(
        ["ros2", "launch", "roadwork_student", "solution.launch.py", f"scenario:={a.scenario}", f"seed:={a.seed}",
         "rviz:=false", "visualize:=false", f"result_path:={result}"],
        stdin=subprocess.DEVNULL, stdout=open(out / "launch.log", "w"), stderr=subprocess.STDOUT,
        start_new_session=True)
    restart = None
    here = pathlib.Path(__file__).resolve().parent
    version = subprocess.run(["git", "-C", str(here.parent), "describe", "--always", "--dirty", "--tags"],
                             capture_output=True, text=True).stdout.strip() or "unknown"
    info = {"node": a.node, "scenario": a.scenario, "seed": a.seed, "kill_at": a.kill_at, "down": a.down,
            "software": version, "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", "0"), "visualization": False}
    killed_t = restarted_t = None
    t0 = time.time()
    try:
        while time.time() - t0 < a.wall_timeout and not result.exists():
            rclpy.spin_once(mon, timeout_sec=0.02)
            if killed_t is None and mon.t >= a.kill_at:
                pids = pids_of(EXE[a.node])
                for p in pids:
                    os.kill(p, signal.SIGKILL)
                killed_t = mon.t
                info["killed_pids"] = pids
            if killed_t is not None and restarted_t is None and mon.t >= killed_t + a.down:
                restart = subprocess.Popen(
                    ["ros2", "run", "roadwork_student", EXE[a.node], "--ros-args", "--params-file", params],
                    stdin=subprocess.DEVNULL, stdout=open(out / "restart.log", "w"), stderr=subprocess.STDOUT,
                    start_new_session=True)
                restarted_t = mon.t
        for _ in range(20):
            rclpy.spin_once(mon, timeout_sec=0.05)
    finally:
        for p in (restart, launch):
            if p is not None and p.poll() is None:
                os.killpg(p.pid, signal.SIGINT)
        for p in (restart, launch):
            if p is not None:
                try:
                    p.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(p.pid, signal.SIGKILL)

    odom = sorted(mon.odom)
    if killed_t is not None and odom:
        k = min(odom, key=lambda o: abs(o[0] - killed_t))
        st = [s for s in mon.states if s[0] <= killed_t]
        info["kill"] = {"t": round(killed_t, 2), "x": round(k[1], 2), "y": round(k[2], 3), "v": round(k[3], 2),
                        "state": st[-1][1] if st else None}
        down_end = restarted_t if restarted_t is not None else killed_t + a.down
        after = [o for o in odom if killed_t <= o[0] <= down_end]
        stop = next((o for o in after if o[3] < 0.1), None)
        info["stop"] = None if stop is None else {
            "time_to_stop": round(stop[0] - killed_t, 2), "distance": round(stop[1] - k[1], 2),
            "lateral_drift": round(stop[2] - k[2], 3), "x": round(stop[1], 2)}
        cmds = [c for c in mon.cmds if killed_t <= c[0] <= down_end]
        info["commands_after_kill"] = {"count": len(cmds), "max_speed": round(max((c[1] for c in cmds), default=0), 2),
                                       "last_before_restart_s": round(cmds[-1][0] - killed_t, 2) if cmds else None}
        seen = []
        for t, s, r in mon.states:
            if killed_t <= t <= down_end and (not seen or seen[-1][1] != s):
                seen.append([round(t, 2), s, r])
        info["states_after_kill"] = seen
    if restarted_t is not None:
        moved = next((o for o in odom if o[0] > restarted_t and o[3] > 0.1), None)
        info["restart"] = {"t": round(restarted_t, 2),
                           "first_motion_after_restart": round(moved[0] - restarted_t, 2) if moved else None}
    try:
        info["evaluator"] = json.loads(result.read_text())
    except (OSError, ValueError):
        info["evaluator"] = None
    (out / "node_loss.json").write_text(json.dumps(info, indent=1) + "\n")
    ev = info["evaluator"] or {}
    print(f"{a.node}: kill={info.get('kill')} stop={info.get('stop')} restart={info.get('restart')} "
          f"pass={ev.get('pass')} collision={ev.get('collision')} time={ev.get('time')}")
    mon.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
