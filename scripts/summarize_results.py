#!/usr/bin/env python3
"""Summarise episode evidence produced by run_episodes.sh into a Markdown table.

Reads <dir>/*/result.json (evaluator) and <dir>/*/launch.log (solution logs).
From the logs it extracts the time GO was latched by Plan and the number of
plan state transitions. Event times from
SCENARIOS.md would be used here for ANALYSIS ONLY, never by
the algorithm.
"""
import json
import pathlib
import re
import sys

GO_EVENT = {}  # documented panel change times, analysis only (none for S1)

RE_PANEL_GO = re.compile(r"panel \w+ -> GO at t=([0-9.]+)")
RE_GO = re.compile(r"GO latched at t=([0-9.]+) x=([0-9.\-]+)")
RE_STATE = re.compile(r"state \w+ -> (\w+) at t=([0-9.]+)")


def parse_log(path):
    out = {"go_confirmed": [], "go": None, "states": []}
    if not path.exists():
        return out
    for line in path.read_text(errors="replace").splitlines():
        if m := RE_PANEL_GO.search(line):
            out["go_confirmed"].append(float(m.group(1)))
        if (m := RE_GO.search(line)) and out["go"] is None:
            out["go"] = tuple(float(g) for g in m.groups())
        if m := RE_STATE.search(line):
            out["states"].append((m.group(1), float(m.group(2))))
    return out


def main():
    root = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else "results/d1")
    rows = []
    for run in sorted(p for p in root.iterdir() if p.is_dir()):
        res_file = run / "result.json"
        try:
            r = json.loads(res_file.read_text())
        except (OSError, ValueError):
            rows.append(f"| {run.name} | NO RESULT | | | | | | | | | | |")
            continue
        log = parse_log(run / "launch.log")
        violations = [k for k in ("collision", "offroad", "entry_violation", "lane_violation",
                                  "speed_violation", "timeout") if r.get(k)]
        scen = int(r.get("scenario", 0))
        latency = ""
        if scen in GO_EVENT:
            after = [t for t in log["go_confirmed"] if t >= GO_EVENT[scen] - 1e-6]
            if after:
                latency = f"{after[0] - GO_EVENT[scen]:.2f}"
        go = f"t={log['go'][0]:.2f} x={log['go'][1]:.2f}" if log["go"] else "-"
        rows.append(
            f"| {run.name} | {'PASS' if r.get('pass') else 'FAIL'} | {r.get('finished')} | "
            f"{', '.join(violations) or 'none'} | {r.get('time', 0):.2f} | {r.get('min_clearance', -1):.3f} | "
            f"{r.get('max_speed', 0):.3f} | {r.get('stopped_seconds', 0):.2f} | {r.get('rejected_commands')} | "
            f"{go} | {latency or '-'} | {len(log['states'])} |")
    print(f"# Episode summary: {root}\n")
    print("| run | result | finished | violations | sim time [s] | min clearance [m] | max speed [m/s] | "
          "stopped [s] | rejected cmds | GO latched | GO confirm latency [s] | state transitions |")
    print("|---|---|---|---|---:|---:|---:|---:|---:|---|---:|---:|")
    print("\n".join(rows))
    print("\nrejected_commands includes startup ticks before the first valid command.")


if __name__ == "__main__":
    main()
