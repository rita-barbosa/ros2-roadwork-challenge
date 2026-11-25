#!/usr/bin/env bash
# Reproduces the complete D2 evidence set (fresh launch per episode, headless).
#   results/d2/default/              15 default runs: S1-S5 x seeds 1 2 3
#   results/d2/var_shift_m2/          geometry variation: work_shift=-2.0, S1-S5 x seeds 1 2 3
#   results/d2/var_shift_p2/          geometry variation: work_shift=+2.0, S1-S5 x seeds 1 2 3
#   results/d2/var_traffic_2/         traffic variation: traffic_speed=2.0, S3 x seeds 1 2 3
#   results/d2/var_traffic_4/         traffic variation: traffic_speed=4.0, S3 x seeds 1 2 3
#   results/d2/var_noise_005/         sensor-noise variation: noise=0.05, S1-S5 x seeds 1 2 3
#   results/d2/ablation_no_gap/       ablation: radar gap check off (use_traffic_gap:=false), S3 x seeds 1 2 3
#   results/d2/ablation_no_obstacle/  ablation: LiDAR corridor check off (use_obstacle_check:=false), S5 x seeds 1 2 3
#   results/d2/offline/               ROS-free evidence: unit tests, held-out panel confusion matrix,
#                                     closed-loop suite (378 runs) and offline ablations
#   results/d2/SUMMARY.md             combined report (scripts/d2_report.py)
# Usage (workspace root, after `colcon build` and `source install/setup.bash`):
#   bash scripts/run_d2.sh [SETS...]      # default: all sets; e.g. `bash scripts/run_d2.sh default ablation_no_gap`
# Takes about 50 minutes of wall time (the simulator runs in real time). Use an isolated ROS_DOMAIN_ID.
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
cd "$ROOT" || exit 1
OUT="results/d2"   # relative, so summaries name repository paths
ALL="offline default var_shift_m2 var_shift_p2 var_traffic_2 var_traffic_4 var_noise_005 ablation_no_gap ablation_no_obstacle"
SETS=${*:-$ALL}
test "${ROS_DISTRO:-}" = jazzy || { echo 'Source /opt/ros/jazzy/setup.bash and install/setup.bash first'; exit 1; }
mkdir -p "$OUT"

run_set() {  # name scenarios seeds extra_args label
  LABEL="$5" EXTRA_ARGS="$4" bash "$HERE/run_episodes.sh" "$OUT/$1" "$2" "$3"
}

for set in $SETS; do
  echo "=== $set ==="
  case "$set" in
    offline)
      mkdir -p "$OUT/offline"
      B="build/roadwork_student"
      "$B/test_perception" > "$OUT/offline/test_perception.txt" 2>&1
      "$B/test_planning" > "$OUT/offline/test_planning.txt" 2>&1
      "$B/test_closed_loop" > "$OUT/offline/closed_loop_suite.txt" 2>&1
      {
        echo "# Offline ablations (ROS-free closed loop, same seeds as the ROS runs)"
        for seed in 1 2 3; do for tr in 2 3 4; do
          "$B/test_closed_loop" 3 "$seed" 0 0.02 0 "$tr"
          "$B/test_closed_loop" 3 "$seed" 0 0.02 0 "$tr" --no-gap
        done; done
        for seed in 1 2 3; do for sh in -2 0 2; do
          "$B/test_closed_loop" 5 "$seed" "$sh" 0.02 0 3
          "$B/test_closed_loop" 5 "$seed" "$sh" 0.02 0 3 --no-obstacle
        done; done
      } > "$OUT/offline/ablations.txt" 2>&1
      tail -1 "$OUT/offline/closed_loop_suite.txt" ;;
    default)              run_set default "1 2 3 4 5" "1 2 3" "" "default parameters" ;;
    var_shift_m2)         run_set var_shift_m2 "1 2 3 4 5" "1 2 3" "work_shift:=-2.0" "geometry variation" ;;
    var_shift_p2)         run_set var_shift_p2 "1 2 3 4 5" "1 2 3" "work_shift:=2.0" "geometry variation" ;;
    var_traffic_2)        run_set var_traffic_2 "3" "1 2 3" "traffic_speed:=2.0" "traffic variation" ;;
    var_traffic_4)        run_set var_traffic_4 "3" "1 2 3" "traffic_speed:=4.0" "traffic variation" ;;
    var_noise_005)        run_set var_noise_005 "1 2 3 4 5" "1 2 3" "noise:=0.05" "noise variation" ;;
    ablation_no_gap)      run_set ablation_no_gap "3" "1 2 3" "use_traffic_gap:=false" "ablation: no radar gap check" ;;
    ablation_no_obstacle) run_set ablation_no_obstacle "5" "1 2 3" "use_obstacle_check:=false" "ablation: no LiDAR corridor check" ;;
    *) echo "unknown set $set (known: $ALL)"; exit 1 ;;
  esac
done
python3 "$HERE/d2_report.py" "$OUT" > "$OUT/SUMMARY.md" && echo "wrote $OUT/SUMMARY.md"
