#!/usr/bin/env python3
"""Summarise one set of episodes produced by run_episodes.sh into Markdown.

Usage: python3 scripts/summarize_results.py SET_DIR   (e.g. results/d2/default)

Reads <SET_DIR>/*/result.json (evaluator), metrics.json (analyze_bag.py, if
the run was recorded) and launch.log (solution logs). Event times from
docs/SCENARIOS.md are used for ANALYSIS ONLY (inside analyze_bag.py), never
by the algorithm.
"""
import json
import pathlib
import re
import sys

RE_GO = re.compile(r"GO latched at t=([0-9.]+) x=([0-9.\-]+)")
RE_STATE = re.compile(r"state \w+ -> (\w+) at t=([0-9.]+)")
VIOLATIONS = ("collision", "offroad", "entry_violation", "lane_violation", "speed_violation", "timeout")


def load_run(run):
    r = {"name": run.name}
    try:
        r["result"] = json.loads((run / "result.json").read_text())
    except (OSError, ValueError):
        r["result"] = None
    try:
        r["metrics"] = json.loads((run / "metrics.json").read_text())
    except (OSError, ValueError):
        r["metrics"] = {}
    r["go"], r["states"] = None, 0
    log = run / "launch.log"
    if log.exists():
        for line in log.read_text(errors="replace").splitlines():
            if (m := RE_GO.search(line)) and r["go"] is None:
                r["go"] = (float(m.group(1)), float(m.group(2)))
            if RE_STATE.search(line):
                r["states"] += 1
    return r


def load_set(root):
    root = pathlib.Path(root)
    return [load_run(p) for p in sorted(root.iterdir()) if p.is_dir() and (p / "launch.log").exists()]


def fmt(v, f="{:.2f}"):
    return "-" if v is None else f.format(v)


def scenario_note(m, scen):
    """One compact scenario-specific measurement column."""
    if not m:
        return "-"
    if scen == 2:
        return f"GO +{fmt(m.get('go_latency'))} s, resume +{fmt(m.get('resume_latency'))} s"
    if scen == 3:
        return (f"yield {fmt(m.get('yield_seconds'), '{:.1f}')} s, gap safe t={fmt(m.get('gap_safe_at'), '{:.1f}')}, "
                f"car clr {fmt(m.get('traffic_clearance_min'))} m")
    if scen == 4:
        return f"beacon stale t={fmt(m.get('beacon_stale_detected'))}, GO +{fmt(m.get('go_latency'))} s, " \
               f"resume +{fmt(m.get('resume_latency'))} s"
    if scen == 5:
        return (f"detect +{fmt(m.get('detect_latency'))} s, brake +{fmt(m.get('brake_onset'))} s, "
                f"stop x={fmt(m.get('stop_x'))}, obst clr {fmt(m.get('obstacle_clearance_min'))} m")
    return f"unexplained stops {m.get('unexplained_stops', '-')}"


def table(runs):
    lines = [
        "| run | result | violations | sim time [s] | min clearance [m] | max speed [m/s] | stopped [s] | "
        "rejected cmds | entry t [s] | GO latched | mean/max \\|cte\\| [m] | scenario measurements | transitions |",
        "|---|---|---|---:|---:|---:|---:|---:|---:|---|---|---|---:|",
    ]
    for r in runs:
        res, m = r["result"], r["metrics"]
        if res is None:
            lines.append(f"| {r['name']} | NO RESULT | | | | | | | | | | | |")
            continue
        viol = ", ".join(k for k in VIOLATIONS if res.get(k)) or "none"
        trk = m.get("tracking", {})
        cte = f"{trk['mean_abs_cte']:.3f} / {trk['max_abs_cte']:.3f}" if trk else "-"
        go = f"t={r['go'][0]:.2f} x={r['go'][1]:.2f}" if r["go"] else "-"
        lines.append(
            f"| {r['name']} | {'PASS' if res.get('pass') else 'FAIL'} | {viol} | {res.get('time', 0):.2f} | "
            f"{res.get('min_clearance', -1):.3f} | {res.get('max_speed', 0):.3f} | {res.get('stopped_seconds', 0):.2f} | "
            f"{res.get('rejected_commands')} | {fmt(m.get('entry_time'))} | {go} | {cte} | "
            f"{scenario_note(m, int(res.get('scenario', 0)))} | {r['states']} |")
    return "\n".join(lines)


def main():
    root = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else "results/d2/default")
    runs = load_set(root)
    env = root / "environment.txt"
    print(f"# Episode summary: {root}\n")
    if env.exists():
        print("```\n" + env.read_text().strip() + "\n```\n")
    print(table(runs))
    n_pass = sum(1 for r in runs if r["result"] and r["result"].get("pass"))
    print(f"\n**{n_pass}/{len(runs)} runs pass.** min clearance is the evaluator's disc-to-disc minimum over all "
          "objects; rejected_commands includes startup ticks before the first valid command.")


if __name__ == "__main__":
    main()
