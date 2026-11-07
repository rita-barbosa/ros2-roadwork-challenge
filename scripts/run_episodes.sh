#!/usr/bin/env bash
# Runs headless episodes, each in a FRESH launch, and stores per-run evidence:
#   <out>/s<S>_seed<N>/result.json   evaluator terminal JSON
#   <out>/s<S>_seed<N>/launch.log    node logs (state transitions, panel changes, rejections)
#   <out>/s<S>_seed<N>/bag/          rosbag of internal + command topics (RECORD=1)
# Usage (from the workspace root, after `source install/setup.bash`):
#   bash scripts/run_episodes.sh [OUT_DIR] ["SCENARIOS"] ["SEEDS"]
#   bash scripts/run_episodes.sh results/d1 "1" "1 2 3"        # S1: 3 runs
# Env: WALL_TIMEOUT (s, default 400), RECORD (1/0), EXTRA_ARGS (e.g. "work_shift:=-2.0 noise:=0.05")
# Use an isolated ROS_DOMAIN_ID per group.
set -u
OUT=${1:-results/d1}
SCENARIOS=${2:-"1"}
SEEDS=${3:-"1 2 3"}
WALL_TIMEOUT=${WALL_TIMEOUT:-400}
RECORD=${RECORD:-1}
EXTRA_ARGS=${EXTRA_ARGS:-}

test "${ROS_DISTRO:-}" = jazzy || { echo 'Source /opt/ros/jazzy/setup.bash and install/setup.bash first'; exit 1; }
mkdir -p "$OUT"
{
  echo "date: $(date -Iseconds)"
  echo "ros_distro: $ROS_DISTRO  ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-0}"
  echo "host: $(uname -a)"
  echo "cpu: $(grep -m1 'model name' /proc/cpuinfo 2>/dev/null | cut -d: -f2)"
  echo "cpus: $(nproc 2>/dev/null)  mem: $(free -h 2>/dev/null | awk '/Mem:/{print $2}')"
  echo "visualization: off (rviz:=false visualize:=false)  extra: ${EXTRA_ARGS}"
} > "$OUT/environment.txt"

stop_group() { [ -n "${1:-}" ] && kill -INT -- "-$1" 2>/dev/null; }

for s in $SCENARIOS; do
  for seed in $SEEDS; do
    tag="s${s}_seed${seed}"
    dir="$OUT/$tag"
    rm -rf "$dir"; mkdir -p "$dir"
    result="$(cd "$dir" && pwd)/result.json"
    BPID=""
    if [ "$RECORD" = 1 ]; then
      setsid ros2 bag record -o "$dir/bag" /solution/scene /solution/plan /solution/state /solution/path \
        /vehicle/command /vehicle/odometry /infra/roadwork/beacon /challenge/metrics > "$dir/bag.log" 2>&1 &
      BPID=$!
      sleep 2
    fi
    # shellcheck disable=SC2086
    setsid ros2 launch roadwork_student solution.launch.py scenario:="$s" seed:="$seed" rviz:=false \
      visualize:=false result_path:="$result" $EXTRA_ARGS > "$dir/launch.log" 2>&1 &
    LPID=$!
    t0=$(date +%s)
    while [ ! -s "$result" ]; do
      sleep 1
      if (( $(date +%s) - t0 > WALL_TIMEOUT )); then echo "$tag: wall-clock timeout" | tee -a "$dir/launch.log"; break; fi
    done
    sleep 1
    stop_group "$BPID"; stop_group "$LPID"
    sleep 3
    kill -KILL -- "-$LPID" 2>/dev/null; [ -n "$BPID" ] && kill -KILL -- "-$BPID" 2>/dev/null
    wait 2>/dev/null
    echo "$tag: $(cat "$result" 2>/dev/null || echo 'NO RESULT')"
  done
done

python3 "$(dirname "$0")/summarize_results.py" "$OUT" | tee "$OUT/summary.md"
