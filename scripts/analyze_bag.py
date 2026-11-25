#!/usr/bin/env python3
"""Extract per-run measurements from an episode bag into <run>/metrics.json.

Usage: python3 scripts/analyze_bag.py RUN_DIR [RUN_DIR ...]
       (RUN_DIR contains bag/ and result.json, as written by run_episodes.sh)

The solution never reads this script's inputs; it is ANALYSIS ONLY. Where a
measurement needs the scenario ground truth (event times, the S3 vehicle or
the S5 obstacle), it uses the documented event definitions from
docs/SCENARIOS.md together with the parameters echoed in result.json.

Measurements (simulation time, SI units):
  tracking      |cte| of the ego to the solution's OWN planned path (from
                /solution/state) while moving (v > 0.5 m/s): mean, RMS, max
  entry_time    first time the vehicle front (x + 0.9) crosses x = 25
  stops         intervals with v < 0.1 m/s for >= 0.5 s after the first motion
                and before the goal, with the decision state during the stop
  go_latency    S2/S4: documented GO/recovery event -> panel GO confirmed in
                the published scene state; resume_latency: event -> first
                state that is no longer WAIT_*
  traffic       S3: time in YIELD, time the gap became SAFE, minimum
                disc-to-disc clearance to the opposing vehicle
  obstacle      S5: trigger (first ego x >= 45 + shift), detection latency
                (trigger -> first plan cycle with the corridor blocked), brake
                onset (trigger -> BLOCKED), stop position, minimum disc-to-disc
                clearance to the temporary obstacle, resume delay after it
                disappears (trigger + 8 s)
  timing        /vehicle/command rate and largest gap, largest scene age seen
                by Plan, largest sensor-to-pose association gap in Sense
"""
import json
import math
import pathlib
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

EGO_R = 0.9
WAIT_STATES = {"WAIT_GO", "WAIT_INFO", "YIELD", "BLOCKED"}


def stamp(h):
    return h.stamp.sec + 1e-9 * h.stamp.nanosec


def read_bag(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
    wanted = {"/solution/state", "/vehicle/odometry", "/vehicle/command", "/solution/scene"}
    out = {k: [] for k in wanted}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic in wanted and topic in types:
            out[topic].append(deserialize_message(data, types[topic]))
    return out


def analyze(run):
    run = pathlib.Path(run)
    res = json.loads((run / "result.json").read_text())
    scen, shift = int(res["scenario"]), float(res["work_shift"])
    traffic = float(res["traffic_speed"])
    bag = read_bag(run / "bag")
    states = [json.loads(m.data) for m in bag["/solution/state"]]
    odom = [(stamp(m.header), m.pose.pose.position.x, m.pose.pose.position.y, m.twist.twist.linear.x)
            for m in bag["/vehicle/odometry"]]
    odom.sort()
    cmds = sorted(stamp(m.header) for m in bag["/vehicle/command"])
    out = {"scenario": scen, "seed": res["seed"], "work_shift": shift, "traffic_speed": traffic,
           "noise": res["noise"], "pass": res["pass"]}

    # Decision trace
    trans, prev = [], None
    for s in states:
        if s["state"] != prev:
            trans.append([round(s["t"], 2), s["state"]])
            prev = s["state"]
    out["transitions"] = trans
    tin = {}
    for a, b in zip(states, states[1:]):
        tin[a["state"]] = tin.get(a["state"], 0.0) + max(0.0, min(b["t"] - a["t"], 0.2))
    out["time_in_state"] = {k: round(v, 2) for k, v in sorted(tin.items())}

    # Tracking error against the solution's own planned path
    cte = [abs(s["cte"]) for s in states
           if s["state"] in ("APPROACH", "BYPASS", "RETURN", "FINAL", "BLOCKED", "YIELD") and s["v"] > 0.5]
    if cte:
        out["tracking"] = {"mean_abs_cte": round(sum(cte) / len(cte), 4),
                           "rms_cte": round(math.sqrt(sum(c * c for c in cte) / len(cte)), 4),
                           "max_abs_cte": round(max(cte), 4), "samples": len(cte)}

    # Entry, first motion, stops
    entry = next((t for t, x, _, _ in odom if x + EGO_R > 25.0), None)
    first_motion = next((t for t, _, _, v in odom if v > 0.1), None)
    out["entry_time"] = round(entry, 3) if entry is not None else None
    out["first_motion"] = round(first_motion, 3) if first_motion is not None else None
    stops, start = [], None
    goal_t = next((s["t"] for s in states if s["state"] == "GOAL"), None)
    for t, x, _, v in odom:
        moving_phase = first_motion is not None and t > first_motion and (goal_t is None or t < goal_t - 0.5)
        if v < 0.1 and moving_phase:
            start = t if start is None else start
        else:
            if start is not None and t - start >= 0.5:
                st = next((s["state"] for s in states if s["t"] >= start), "?")
                stops.append({"from": round(start, 2), "to": round(t, 2), "x": round(x, 2), "state": st})
            start = None
    out["stops"] = stops
    out["unexplained_stops"] = sum(1 for s in stops if s["state"] not in WAIT_STATES)

    def first_state(pred, after):
        return next((s["t"] for s in states if s["t"] >= after - 1e-6 and pred(s)), None)

    # S2 / S4: reaction to the documented GO / recovery event
    event = {2: 10.0, 4: 8.0}.get(scen)
    if event is not None:
        go = first_state(lambda s: s["panel"] == "GO", event)
        resume = first_state(lambda s: s["state"] not in ("WAIT_GO", "WAIT_INFO"), event)
        out["event_time"] = event
        out["go_latency"] = round(go - event, 3) if go is not None else None
        out["resume_latency"] = round(resume - event, 3) if resume is not None else None
        before = [s for s in states if s["t"] < event and s["t"] > 2.0]
        out["waiting_before_event"] = sorted({s["state"] for s in before})
        if scen == 4:
            stale = first_state(lambda s: not s.get("beacon_fresh", True), 2.0)
            out["beacon_stale_detected"] = round(stale, 2) if stale is not None else None

    # S3: opposing traffic (vehicle radius 0.9 from (82, 1.75) at -traffic_speed; removed at x <= -5)
    if scen == 3:
        clear = []
        for t, x, y, _ in odom:
            cx = 82.0 - traffic * t
            if cx > -5:
                clear.append(math.hypot(cx - x, 1.75 - y) - 0.9 - EGO_R)
        out["traffic_clearance_min"] = round(min(clear), 3) if clear else None
        out["yield_seconds"] = round(tin.get("YIELD", 0.0), 2)
        unsafe = first_state(lambda s: not s.get("gap_ok", True), 0.0)
        safe = first_state(lambda s: s.get("gap_ok", False), unsafe) if unsafe is not None else None
        out["gap_unsafe_from"] = unsafe
        out["gap_safe_at"] = safe
        if safe is not None:
            # where the vehicle was when it passed (analysis: true position of the vehicle then)
            out["traffic_x_when_gap_safe"] = round(82.0 - traffic * safe, 2)

    # S5: temporary obstacle (radius 0.65 at (57+shift, 1.75) for 8 s after the trigger)
    if scen == 5:
        trig = next((t for t, x, _, _ in odom if x >= 45.0 + shift), None)
        out["trigger_time"] = trig
        if trig is not None:
            ox, oy = 57.0 + shift, 1.75
            det = first_state(lambda s: s.get("blocked", False), trig)
            brake = first_state(lambda s: s["state"] == "BLOCKED", trig)
            out["detect_latency"] = round(det - trig, 3) if det is not None else None
            out["brake_onset"] = round(brake - trig, 3) if brake is not None else None
            during = [(t, x, y, v) for t, x, y, v in odom if trig <= t < trig + 8.0]
            if during:
                out["obstacle_clearance_min"] = round(min(math.hypot(ox - x, oy - y) - 0.65 - EGO_R
                                                          for _, x, y, _ in during), 3)
                stopped = [(t, x, y) for t, x, y, v in during if v < 0.05]
                if stopped:
                    out["stop_x"] = round(stopped[0][1], 3)
                    out["stop_time"] = round(stopped[0][0], 2)
            gone = trig + 8.0
            resume = next((t for t, _, _, v in odom if t >= gone and v > 0.1), None)
            out["resume_delay"] = round(resume - gone, 3) if resume is not None else None
            out["blocked_seconds"] = round(tin.get("BLOCKED", 0.0), 2)

    # Timing
    if len(cmds) > 1:
        gaps = [b - a for a, b in zip(cmds, cmds[1:])]
        out["command_rate_hz"] = round((len(cmds) - 1) / (cmds[-1] - cmds[0]), 2)
        out["command_max_gap"] = round(max(gaps), 3)
    ages = [s["scene_age"] for s in states if s.get("scene_age", -1) >= 0]
    if ages:
        out["scene_age_max"] = round(max(ages), 3)
    scenes = bag["/solution/scene"]
    if scenes:
        out["lidar_pose_dt_max"] = round(max(m.lidar_pose_dt for m in scenes), 3)
        out["radar_pose_dt_max"] = round(max(m.radar_pose_dt for m in scenes), 3)
        out["lidar_rejected"] = int(scenes[-1].lidar_rejected)
        out["radar_rejected"] = int(scenes[-1].radar_rejected)
        out["camera_rejected"] = int(scenes[-1].camera_rejected)
    return out


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    for run in sys.argv[1:]:
        m = analyze(run)
        (pathlib.Path(run) / "metrics.json").write_text(json.dumps(m, indent=1) + "\n")
        print(f"{run}: {json.dumps({k: v for k, v in m.items() if k not in ('transitions', 'stops')})}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
