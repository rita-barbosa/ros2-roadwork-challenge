#!/usr/bin/env bash
# Runs headless episodes, each in a FRESH launch, and stores per-run evidence:
#   <out>/s<S>_seed<N>/result.json   evaluator terminal JSON
#   <out>/s<S>_seed<N>/run.json      run metadata (software version, arguments, visualisation, wall time)
#   <out>/s<S>_seed<N>/launch.log    node logs (state transitions, panel/beacon/gap/corridor events, rejections)
#   <out>/s<S>_seed<N>/bag/          rosbag (MCAP) of sensor, internal and command topics (RECORD=1, not in git)
#   <out>/s<S>_seed<N>/metrics.json  measurements extracted from the bag by analyze_bag.py (RECORD=1)
# Usage (from the workspace root, after `source install/setup.bash`):
#   bash scripts/run_episodes.sh [OUT_DIR] ["SCENARIOS"] ["SEEDS"]
#   bash scripts/run_episodes.sh results/d2/default "1 2 3 4 5" "1 2 3"
# Env: WALL_TIMEOUT (s, default 400), RECORD (1/0), EXTRA_ARGS (launch arguments, e.g.
#      "work_shift:=-2.0 noise:=0.05" or "use_traffic_gap:=false"), LABEL (free text stored in run.json)
# Use an isolated ROS_DOMAIN_ID per group; never run two episodes in the same domain.
set -u
# Job control: every background job gets its own process group and, unlike in a
# plain non-interactive shell, does NOT ignore SIGINT. Without it `ros2 bag
# record` never sees the SIGINT and is killed before it finalises the bag.
# Background jobs read stdin from /dev/null so they are never stopped by SIGTTIN.
set -m
OUT=${1:-results/d2/default}
SCENARIOS=${2:-"1 2 3 4 5"}
SEEDS=${3:-"1 2 3"}
WALL_TIMEOUT=${WALL_TIMEOUT:-400}
RECORD=${RECORD:-1}
EXTRA_ARGS=${EXTRA_ARGS:-}
LABEL=${LABEL:-}
HERE="$(cd "$(dirname "$0")" && pwd)"
BAG_TOPICS="/solution/scene /solution/plan /solution/state /solution/path /vehicle/command /vehicle/odometry
  /infra/roadwork/beacon /radar/detections /lidar/scan /challenge/metrics"

test "${ROS_DISTRO:-}" = jazzy || { echo 'Source /opt/ros/jazzy/setup.bash and install/setup.bash first'; exit 1; }
mkdir -p "$OUT"
VERSION="$(git -C "$HERE/.." describe --always --dirty --tags 2>/dev/null || echo unknown)"
{
  echo "date: $(date -Iseconds)"
  echo "software: $VERSION"
  echo "ros_distro: $ROS_DISTRO  ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-0}"
  echo "host: $(uname -a)"
  echo "cpu: $(grep -m1 'model name' /proc/cpuinfo 2>/dev/null | cut -d: -f2)"
  echo "cpus: $(nproc 2>/dev/null)  mem: $(free -h 2>/dev/null | awk '/Mem:/{print $2}')"
  echo "visualization: off (rviz:=false visualize:=false)"
  echo "scenarios: $SCENARIOS  seeds: $SEEDS  extra: ${EXTRA_ARGS:-none}  label: ${LABEL:-none}"
} > "$OUT/environment.txt"

stop_group() {  # SIGINT the job's process group, give it time to shut down cleanly, then SIGKILL
  local pid=${1:-}
  [ -n "$pid" ] || return 0
  kill -INT -- "-$pid" 2>/dev/null
  for _ in $(seq 1 20); do kill -0 -- "-$pid" 2>/dev/null || return 0; sleep 0.5; done
  echo "process group $pid did not stop after SIGINT; killing" >&2
  kill -KILL -- "-$pid" 2>/dev/null
}

for s in $SCENARIOS; do
  for seed in $SEEDS; do
    tag="s${s}_seed${seed}"
    dir="$OUT/$tag"
    rm -rf "$dir"; mkdir -p "$dir"
    result="$(cd "$dir" && pwd)/result.json"
    BPID=""
    if [ "$RECORD" = 1 ]; then
      # shellcheck disable=SC2086
      ros2 bag record -s mcap -o "$dir/bag" --topics $BAG_TOPICS < /dev/null > "$dir/bag.log" 2>&1 &
      BPID=$!
      sleep 2
    fi
    t0=$(date +%s)
    # shellcheck disable=SC2086
    ros2 launch roadwork_student solution.launch.py scenario:="$s" seed:="$seed" rviz:=false \
      visualize:=false result_path:="$result" $EXTRA_ARGS < /dev/null > "$dir/launch.log" 2>&1 &
    LPID=$!
    while [ ! -s "$result" ]; do
      sleep 1
      if (( $(date +%s) - t0 > WALL_TIMEOUT )); then echo "$tag: wall-clock timeout" | tee -a "$dir/launch.log"; break; fi
      kill -0 "$LPID" 2>/dev/null || { echo "$tag: launch exited early" | tee -a "$dir/launch.log"; break; }
    done
    wall=$(( $(date +%s) - t0 ))
    sleep 1
    stop_group "$LPID"
    stop_group "$BPID"
    wait 2>/dev/null
    cat > "$dir/run.json" <<EOF
{"software": "$VERSION", "scenario": $s, "seed": $seed, "extra_args": "$EXTRA_ARGS", "label": "$LABEL",
 "visualization": false, "rviz": false, "ros_distro": "$ROS_DISTRO", "ros_domain_id": "${ROS_DOMAIN_ID:-0}",
 "wall_seconds": $wall, "date": "$(date -Iseconds)"}
EOF
    if [ "$RECORD" = 1 ] && [ -f "$dir/bag/metadata.yaml" ]; then
      python3 "$HERE/analyze_bag.py" "$dir" > "$dir/analyze.log" 2>&1 || echo "$tag: bag analysis failed (see analyze.log)"
    fi
    echo "$tag: $(cat "$result" 2>/dev/null || echo 'NO RESULT')"
  done
done

python3 "$HERE/summarize_results.py" "$OUT" | tee "$OUT/summary.md"
