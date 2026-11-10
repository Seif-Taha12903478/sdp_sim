#!/usr/bin/env bash
# Starts the hospital-system demo (test step e) DETACHED: it keeps running when the
# terminal closes, the SSH / internet connection drops or Claude Code is closed, and it
# shuts the simulation down by itself at the end (~5-6 h).
#
#   bash ~/sdp_sim/tools/run_demo.sh [extra args for run_demo.py, e.g. --fast-speed 0.12]
#
# Progress:  tail -f ~/sdp_sim/results/system/demo_<date>/scheduler.log
# Result:    ~/sdp_sim/results/system/demo_<date>/summary.txt, video media/system_demo.mp4
# Stop early (after the current delivery):  touch ~/sdp_sim/results/system/demo_<date>/STOP
# Keep the laptop awake: a sleeping host pauses the VM (the run continues afterwards, but
# the wall times are then wrong).
set -e
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export CAMERA_TYPE=GEMINI
OUT=~/sdp_sim/results/system/demo_$(date +%Y%m%d_%H%M%S)
mkdir -p "$OUT"
setsid nohup python3 -u ~/sdp_sim/tools/run_demo.py --out "$OUT" "$@" > "$OUT/run.log" 2>&1 < /dev/null &
echo $! > "$OUT/run_demo.pid"
echo "demo started (pid $(cat "$OUT/run_demo.pid")), output in $OUT"
echo "progress:  tail -f $OUT/scheduler.log"
