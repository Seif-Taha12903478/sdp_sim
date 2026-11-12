#!/usr/bin/env bash
# All offline tests of TrackMed in one command (no Gazebo, ~3-5 min on 2 cores):
#     bash tools/run_offline_tests.sh            # every group
#     bash tools/run_offline_tests.sh unit route # only these groups
# Groups:
#   unit     tools/test_system_offline.py - 19 unit tests: route planner, scheduler, scoring
#   layout   tools/system_layout.py - clearance / geometry checks of the hospital layout
#   worlds   tools/make_hospital_world.py + make_system_world.py run in a temporary copy;
#            the generated worlds, robot xacro, map and container models must equal the
#            committed files (absolute repo path normalised: /home/ubuntu/sdp_sim or this
#            checkout's own path); the generators assert colours
#            and clearances themselves
#   rescore  tools/rescore.py on copies of results/eval_5runs*/: the scores re-computed from
#            the raw run JSONs (and the recorded wall-time validity) must equal the committed
#            runs.csv
#   armwait  tools/test_arm_wait.py - 7 tests of the sim-time arm wait + frozen-cache recovery
#            (rclpy, fake /joint_states publisher in ROS domain 77)
#   route    tools/offline_route_sim.py - the line/route follower against a synthetic wrist
#            camera: 109 checks (4 runs of the old route, 17 delivery legs x 3 start offsets =
#            51 runs, the same 51 in fast mode, 3 fault cases)
# Needs ROS 2 Humble (sourced here if found in /opt/ros/humble), python3-opencv, numpy, yaml,
# matplotlib. Logs: results/offline_tests/<group>.log. Exit code 0 = all passed.
REPO="$(cd "$(dirname "$0")/.." && pwd)"
cd "$REPO" || exit 2
GROUPS_ALL="unit layout worlds rescore armwait route"
GROUPS_RUN="${*:-$GROUPS_ALL}"
LOGS="$REPO/results/offline_tests"
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT
mkdir -p "$LOGS"
export PYTHONDONTWRITEBYTECODE=1

if ! python3 -c "import rclpy" 2>/dev/null && [ -f /opt/ros/humble/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi
missing=""
for m in rclpy sensor_msgs cv2 numpy yaml matplotlib; do
  python3 -c "import $m" 2>/dev/null || missing="$missing $m"
done
if [ -n "$missing" ]; then
  echo "missing Python modules:$missing"
  echo "(ROS 2 Humble sourced? sudo apt install python3-opencv python3-numpy python3-yaml python3-matplotlib)"
  exit 2
fi

t_unit()    { python3 tools/test_system_offline.py -v; }
t_layout()  { python3 tools/system_layout.py "$TMP/layout.png"; }
t_armwait() { python3 tools/test_arm_wait.py -v; }
t_route()   { python3 tools/offline_route_sim.py; }

t_worlds() {
  local G="$TMP/gen" f n=0
  mkdir -p "$G" && cp -r tools scripts config robot_gazebo "$G/" || return 1
  python3 "$G/tools/make_hospital_world.py" && python3 "$G/tools/make_system_world.py" || return 1
  local files
  files=$(find robot_gazebo config -type f ! -path '*__pycache__*' | sort)
  for f in $files; do
    # the committed files name /home/ubuntu/sdp_sim; after install.sh regenerated them on another
    # machine they name this checkout ($REPO) - both are normalised
    if ! cmp -s <(sed "s|$REPO|ROOT|g; s|/home/ubuntu/sdp_sim|ROOT|g" "$f") <(sed "s|$G|ROOT|g" "$G/$f"); then
      echo "DIFFERS from the generator output: $f"; n=$((n + 1))
    fi
  done
  echo "$(echo "$files" | wc -l) files compared, $n differ"
  [ "$n" -eq 0 ]
}

t_rescore() {
  local d bad=0
  for d in eval_5runs eval_5runs_confirmed; do
    mkdir -p "$TMP/$d" && cp results/$d/run*_result.json results/$d/runs.csv "$TMP/$d/" || return 1
    python3 tools/rescore.py "$TMP/$d" > "$TMP/$d.log" 2>&1 || { cat "$TMP/$d.log"; return 1; }
    python3 - "$TMP/$d/runs.csv" "results/$d/runs.csv" "$d" <<'EOF' || bad=1
import csv, sys
new, old = (list(csv.DictReader(open(p))) for p in sys.argv[1:3])
cols = ('success', 'mission_ok', 'placement_error_mm', 'tilt_deg', 'at_target', 'on_table')
if 'wall_time_valid' in old[0]:          # recorded during the run: a re-score must keep it
    cols += ('wall_time_valid',)
bad = [(r['run'], c, r[c], o[c]) for r, o in zip(new, old) for c in cols if r[c] != o[c]]
ok = sum(r['success'] == 'True' for r in new)
print('%s: %d runs re-scored, %d/%d success, mismatches: %s' % (sys.argv[3], len(new), ok, len(new), bad or 'none'))
sys.exit(1 if bad or len(new) != len(old) else 0)
EOF
  done
  return $bad
}

declare -A RESULT
fail=0
for g in $GROUPS_RUN; do
  case " $GROUPS_ALL " in *" $g "*) ;; *) echo "unknown group: $g (groups: $GROUPS_ALL)"; exit 2;; esac
  printf '%-8s ... ' "$g"
  start=$SECONDS
  if "t_$g" > "$LOGS/$g.log" 2>&1; then RESULT[$g]="PASS"; else RESULT[$g]="FAIL"; fail=1; fi
  echo "${RESULT[$g]}  ($((SECONDS - start)) s, log results/offline_tests/$g.log)"
  [ "${RESULT[$g]}" = PASS ] || tail -20 "$LOGS/$g.log" | sed 's/^/    | /'
done
echo
if [ $fail -eq 0 ]; then echo "ALL OFFLINE TESTS PASSED"; else echo "SOME OFFLINE TESTS FAILED"; fi
exit $fail
