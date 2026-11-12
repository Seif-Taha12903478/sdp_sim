# TrackMed - tests and results

Two kinds of tests:

- **Offline** - no Gazebo, a few minutes, run by CI on every push.
- **Gazebo** - a real simulation, 20 min to 6 h on the 2-core development VM, run by hand.

Every Gazebo test starts its own fresh headless simulation and shuts it down afterwards.
None may run while another simulation is running (see TROUBLESHOOTING.md).

Dates are 2026 (Asia/Dubai). Result files are under `results/` (the folder is git-ignored;
the files listed here are committed with `git add -f`). "sim s" = simulated seconds.
File paths inside `results/deliveries.csv` and `results/rtf.csv` are relative to the
repository.

## 1. Offline tests (one command)

```bash
source /opt/ros/humble/setup.bash
bash tools/run_offline_tests.sh            # all groups, ~3 min on the VM, ~1 min in CI
bash tools/run_offline_tests.sh unit route # selected groups
```
Logs: `results/offline_tests/<group>.log`. CI: `.github/workflows/offline-tests.yml`
(container `ros:humble` + python3-opencv/numpy/yaml/matplotlib); the logs are uploaded
as the artifact `offline-test-logs`.

| group | what it tests | result |
|---|---|---|
| `unit` | `tools/test_system_offline.py`: route planner (every delivery leg, turns, lengths), scheduler (queue order, lateness, out of stock, slot planning of `--list`, demo schedule, hospital clock, `deliveries.csv` shared by both schedulers without losing columns), delivery scoring | 19/19 |
| `layout` | `tools/system_layout.py`: clearances furniture <-> line (>= 0.165 m) and turn-round circles (> 0.285 m), route table | all checks pass |
| `worlds` | both world generators run in a temporary copy; all 36 files under `robot_gazebo/` and `config/` must equal the committed ones (absolute repo path normalised, also after `install.sh` regenerated the worlds for another path); the generators assert colours and clearances | 36/36 identical |
| `rescore` | `tools/rescore.py` re-scores both 5-run evaluations from the raw per-run JSONs; success, placement error, tilt, at-target, on-table and the recorded wall-time validity must equal the committed `runs.csv` | eval_5runs 2/5, eval_5runs_confirmed 5/5, 0 mismatches |
| `armwait` | `tools/test_arm_wait.py`: the sim-time arm wait with a fake `/joint_states` publisher (ROS domain 77): reached, settled, too short, stuck, **frozen cache recovered**, gripper settling, no messages | 7/7 |
| `route` | `tools/offline_route_sim.py`: the real line/route follower code against a synthetic wrist camera (same camera geometry, rendered colours, 4 Hz, one-step command latency) | 109/109 |

Details of `route`:
- **Old single-room route**, 4 runs (camera frame phases): the tested follower stops at
  (1.6993, 1.4752) in three of them and at (1.6993, 1.4767) in one, after 38.8-39.0 sim s.
  Gazebo gave (1.6992, 1.4723) after 38.6-39.0 sim s: 3-4 mm apart.
- **17 delivery legs x 3 starts = 51 runs**: all arrive, stops within 16 mm.
  - The legs: home -> pharmacy; pharmacy slot -> room (from S1-S6 for Room 1, from S3 and
    S6 for Rooms 2-4); room -> home for each room.
  - The starts: nominal, +12 mm / +1 deg, -12 mm / -1 deg.
- **The same 51 runs at 0.12 m/s** (fast mode): all arrive.
- **3 fault cases** (junction marker missing, junction marker red, room stop marker
  missing): the robot stops with an error each time.

## 2. Single patient room (`hospital.launch.py`)

| test | how to reproduce | result | result files |
|---|---|---|---|
| Stages 1-3: world loads, 3 controllers active, arm home, camera, drive, line follower | README "Quick start" + `scripts/line_follower.py` | arrived at the patient marker, stop (1.6992, 1.4723, 89.99 deg) in 38.7 sim s; return trip stop (0.044, -0.001, 180.0 deg) | development log (`CLAUDE.md`, "Line follower status") |
| 5-run evaluation, **old weld code** (one-shot `ign topic` attach/detach) | historical: code from before the confirmed weld (introduced in commit `4137a13`); re-scored with today's `tools/rescore.py` | **2/5** - a lost detach (container carried home), a lost attach (container never left the station), a late detach (container on the floor) | `results/eval_5runs/` (`summary.txt`, `runs.csv`, `runNN_result.json`, `runNN_mission.log`) |
| 5-run evaluation, **confirmed weld** + sim-time gripper wait (beige table, before the smart table) | `git checkout b60e882`, `bash install.sh`, `python3 tools/evaluate.py --runs 5` (~50 min per run) | **5/5** - placement error mean 0.19 mm (max 0.20), tilt 0.0 deg, 160.2 sim s per mission, ~48 min wall, every attach/detach confirmed on the first try. Run 4's wall time is invalid (the laptop slept; detected and left out) | `results/eval_5runs_confirmed/` |
| Smart table alone | fresh `ros2 launch robot_gazebo hospital.launch.py gui:=false`, then `python3 tools/test_smart_table.py` | 28/29 checks; the failed check was a wrong test assumption (the container touches the platform while still welded), fixed in the test, not rerun. Lift 0.10 -> 0.7997 m in 18.6 sim s, cruise 39.9 mm/s, container z 0.830 / tilt 0.0 deg after the lift, auto-lower 3.04 s after removal | `results/room_mission/table_test.{json,log}` |
| Scheduled patient-room missions with the smart table | launch `hospital.launch.py gui:=false` (and `video:=true` for the second), `python3 scripts/scheduler.py --in-minutes 1 --once` | **2/2** - placement 0.27 mm and 0.25 mm, raised in 18.56 / 18.68 s, container on the raised platform at z 0.830, 178.3 / 178.4 sim s | `results/room_mission/` (`mission_*.json`, `scheduler*.log`), rows in `results/deliveries.csv` |
| Real-time factor | `python3 tools/measure_rtf.py` (interleaved runs) | single room with smart table 8.74 % vs 9.37 % before (-6.7 %, inside the run-to-run spread); hospital system 8.35 % vs 10.68 % single room (-22 %) | `results/rtf.csv` |

Videos: `media/mission_confirmed.mp4` (confirmed weld), `media/mission_room.mp4` (smart table).

## 3. Hospital system (`hospital_system.launch.py`)

The tests below were run in this order (steps a-e of the system plan).

| test | how to reproduce | result | result files |
|---|---|---|---|
| a. Offline unit + route tests | section 1 | as section 1 | - |
| b. World load + photos | `python3 tools/system_photos.py` | world loads, 4 tables report "lowered", 6 containers in their slots, 5 pictures | `results/system/photos_20261110_153229/summary.json`, `steps.log`; `media/system_*.png` |
| Route test, Room 3, 0.08 m/s (no pick/place) | `python3 tools/test_route_gazebo.py --room 3` | **9/9** - stops 7 / 16 / 7 mm from the map, legs 76.9 + 69.1 sim s, 39 min wall | `results/system/route_test_20261110_155758/summary.json`, `route_test_run1.log` |
| Route test, Room 3, `--fast-speed 0.12` | `python3 tools/test_route_gazebo.py --room 3 --fast-speed 0.12` | **9/9** - stops 7 / 5 / 5 mm, legs 65.6 + 57.9 sim s, 33 min wall -> 0.12 m/s adopted | `results/system/route_test_20261110_163731_fast/summary.json`, `route_test_fast.log` |
| c. Pick + replace at S1, S3, S6 | `python3 tools/test_pharmacy_pick.py` | **12/12** - every attach/detach confirmed first try, back 0.4 / 1.3 / 0.7 mm from the slot, tilt <= 0.1 deg, neighbours moved 0.00 mm | `results/system/pick_test_20261110_171036/` (`pick_test.json`, cap images), `pick_test_run1.log` |
| Cap-colour check (after the amber cap became purple) | `python3 tools/test_pharmacy_pick.py --caps-only` | **9/9** - all 6 caps read right; the robot lined up within 0.5 mm of every slot | `results/system/pick_test_20261111_081711/`, `cap_check_run1.log` |
| d. One full delivery, Room 3 | `python3 tools/run_in_sim.py --name delivery_room3 -- python3 scripts/hospital_scheduler.py --now 3` | **SUCCESS** - Ibuprofen S3 to Patient C, placed 2.8 mm from the target (fixed afterwards: fine line-up at the slot), table raised, the 5 other containers in place, parked within 2 mm; 232.3 sim s, 72 min wall | `results/system/delivery_room3_20261110_185826/`, `results/system/deliveries/delivery_185946_R3_S3*.json`, `delivery_room3_run1.log` |
| Arm home while carrying (+ frozen cache on purpose) | `python3 tools/run_in_sim.py --name arm_home_held -- python3 tools/test_arm_home_held.py` | **PASS** - home reached in 3.6 sim s; with the cache frozen: detected after 15 s, recovered, reached in 3.6 sim s | `results/system/arm_home_held_20261111_124815/`, `arm_home_held_run1.log` |
| e. Demo, attempt 1 | `bash tools/run_demo.sh` | **2/3, stopped** - Room 1 OK (1.6 mm), Room 3 OK (1.5 mm), Room 4 FAILED: the joint-state cache of the delivery process froze, and the wall-clock arm timeout failed the delivery. Led to the sim-time arm wait + cache recovery above | `results/system/demo_attempt1_20261111_084304/` (`README.txt`, `summary.txt`, `scheduler.log`) |
| e. **Demo, attempt 2** (final) | `bash tools/run_demo.sh` (~5.7 h) | **4/4 SUCCESS** (table below), clean shutdown | `results/system/demo_20261111_131344/` (`summary.txt`, `summary.json`, `scheduler.log`), `results/system/deliveries/delivery_08*.json`, rows in `results/deliveries.csv`, video `media/system_demo.mp4` |

Demo attempt 2, on the hospital clock (doses due 08:00 Room 1 + Room 3, 08:02 Room 4,
08:04 Room 2; one robot, so later doses queue):

| room | patient | medicine (slot) | due | late | placement error | sim time | wall time |
|---|---|---|---|---|---|---|---|
| 1 | Patient A | Paracetamol 500 mg (S1) | 08:00 | 0.0 min | 2.0 mm | 185.2 s | 67 min |
| 3 | Patient C | Ibuprofen 400 mg (S3) | 08:00 | 3.1 min | 1.4 mm | 237.7 s | 103 min |
| 4 | Patient D | Metformin 500 mg (S4) | 08:02 | 5.1 min | 0.6 mm | 238.8 s | 93 min |
| 2 | Patient B | Amoxicillin 250 mg (S2) | 08:04 | 7.0 min | 1.7 mm | 181.4 s | 74 min |

Across the four deliveries:
- all caps were read right, and every attach/detach was confirmed on the first try;
- the tables were raised in 18.4-18.7 s, with the container at z 0.830 and tilt 0.0 deg;
- the other containers moved <= 0.14 mm, and the robot parked within 2 mm;
- no joint-state cache recovery was needed.

Placement error is measured at the moment the table's contact sensor confirmed the
container. Its source:
- the robot lines up within 0.5 mm of the slot (the tolerance of the fine line-up), on the
  side it comes from;
- the containers sit 1.1-1.3 mm -x of their slot after spawning.

What the number includes: the target is the spot 0.20 m to the left of where the robot
**actually stopped** at the room, so the error measures the pick and place only. The stop
itself was 5.1-6.6 mm from the nominal room stop of the map in the four deliveries, and the
containers stood 3.8-6.2 mm from the map's nominal place target (the platform is
25 x 25 cm). In `results/deliveries.csv` the three rows of demo attempt 1 have no
`result_json`: attempt 2 overwrote those files (same hospital-clock file names; new runs
now carry their start date in the name).

## 4. Known gaps
- No 5-run repeat of the full hospital demo (one demo takes ~5.7 h on the VM).
- The scheduler's real-time mode (`hospital_scheduler.py` without `--now` / `--demo`) was not
  run in Gazebo. One run of the simulation serves **one dose per room**: a delivered
  container stays on its raised table, so a second delivery to that room would stop at
  "place"; the stock is not refilled. `config/doses.yaml` has one dose per room for that
  reason.
- **No motion watchdog.** The base driver holds its last velocity command, and the nodes
  wait on simulated feedback without a wall-clock limit. If the camera, the odometry or the
  joint states stopped, or a follower process was killed, the robot would keep its last
  motion and the mission would wait forever. The only backstop is `run_demo.py --max-hours`.
- The smart-table test file `results/room_mission/table_test.json` says `"passed": false`
  (28/29). The failed check was a wrong assumption of the test; the test was corrected but
  not run again.
- The 5-run evaluation was done before the smart table was added. With the table there
  are 2 scheduled missions in the single room and 5 deliveries in the hospital system
  (test d + demo attempt 2).
- The scheduler process exits with code -6 (abort inside rclpy shutdown) after it has
  finished and written its results. Harmless; not fixed. It happened in both `--demo` runs
  ("terminate called without an active exception") and not with `--now 3`. Probable cause:
  only `--demo` starts a background thread that spins rclpy (the sim-time clock), and
  nothing shuts rclpy down before the interpreter exits.
- The root cause of the frozen joint-state cache (demo attempt 1) is unknown; it is
  detected and recovered (0 recoveries needed in attempt 2).

## 5. Known code issues, not fixed
Found in the pre-submission review and left as they are (none affects a result above):
- `scripts/hospital_scheduler.py`: the -6 exit of `--demo` (section 4).
- `scripts/delivery.py`: only `RuntimeError` is turned into a failed delivery with an
  `error` text. Any other exception still writes the result JSON, but with `error: null`
  and a Python traceback in the log.
- Unused variables and imports in several scripts, and two unused helpers
  (`pharmacy.Pharmacy.find`, the alias `smart_table.SmartTable`).
- `tools/evaluate.py` (`gazebo_pids`, `stop_sim`): after a run the tools kill every leftover
  `robot_state_publisher`, `parameter_bridge`, `planar_base.py`, `smart_table.py` and
  follower process on the machine, not only the ones they launched. Do not run another
  ROS project next to the Gazebo tests.
- Fixed output paths in `/tmp`: `/tmp/line_follower_debug.jpg` (both followers) and the
  default `/tmp/sim_shot.png` of `tools/sim_screenshot.py`.
- `scripts/jetarm_pick_place.py run | attach | detach` are legacy commands of the first test
  stage (one-shot weld command, wall-clock sleep). They are marked and print a warning; the
  missions do not use them.
