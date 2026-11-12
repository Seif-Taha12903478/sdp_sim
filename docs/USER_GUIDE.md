# TrackMed - user guide

All commands assume the installation from README.md: the repo in `~/sdp_sim`, ROS sourced
(`source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash`) and
`CAMERA_TYPE=GEMINI` exported. Only one simulation may run at a time.

## 1. Run the hospital system

```bash
ros2 launch robot_gazebo hospital_system.launch.py gui:=false   # terminal 1 (headless)
python3 ~/sdp_sim/scripts/jetarm_pick_place.py home              # terminal 2: arm to its ready pose
python3 ~/sdp_sim/scripts/hospital_scheduler.py                  # real time, runs forever
```

`gui:=false` starts the server only; the cameras still render. Drop it to get the Gazebo
window, which is slower. `video:=true` runs the overview camera at 2 Hz for recordings.

| scheduler command | what it does |
|---|---|
| `hospital_scheduler.py` | real local time (time zone of `config/doses.yaml`), runs the doses as they fall due, forever |
| `hospital_scheduler.py --once` | runs what is due now, then exits |
| `hospital_scheduler.py --now 3` | one dose for Room 3 due now (test), then exits |
| `hospital_scheduler.py --demo` | the 4-dose demo on a hospital clock starting 08:00, then exits |
| `hospital_scheduler.py --list` | prints today's doses and their queue order (no simulation needed) |
| `--cap-check` | look at the container's cap colour before the pick (on in the demo) |
| `--stop-file FILE` | when FILE exists, no further delivery is started |

At start the scheduler sets up the pharmacy (`scripts/pharmacy.py setup`: the containers
are spawned into their slots). Each delivery is appended to `results/deliveries.csv`. Its
JSON result goes to `results/system/deliveries/`.

**The whole demo, unattended** (launch, video, scheduler, shutdown, summary, ~5-6 h):
```bash
bash ~/sdp_sim/tools/run_demo.sh            # detached; survives closing the terminal
tail -f ~/sdp_sim/results/system/demo_<date>/scheduler.log
touch ~/sdp_sim/results/system/demo_<date>/STOP     # optional: stop after the current delivery
```
The run writes `summary.txt` / `summary.json` in its demo folder and copies the video to
`media/system_demo.mp4` (this overwrites the committed video).

**One delivery by hand**, after the pharmacy is set up and with the robot parked at home:
`python3 ~/sdp_sim/scripts/delivery.py --room 3 --slot S3 --patient "Patient C"`.

## 2. Edit the medication schedule - `config/doses.yaml`

```yaml
time_zone: Asia/Dubai
doses:
  - {patient: Patient A, room: 1, medicine: Paracetamol, dose: 500 mg, times: ["08:00", "20:00"]}
```
- `room`: 1-4 (Rooms 1 and 2 at junction J1, Rooms 3 and 4 at J2).
- `medicine` + `dose` must match an entry of `config/inventory.yaml` exactly.
- `times`: daily due times, 24 h, in `time_zone`.

How the queue works:
- Due doses are ordered by due time, then by route length (nearest room first).
- One robot means one delivery at a time. A dose that falls due during another delivery
  waits; its lateness is logged.
- No stocked container for a dose -> logged as "out of stock" and skipped.

Check your edit without a simulation: `python3 scripts/hospital_scheduler.py --list`.

`--demo` ignores this file. Its schedule is built into `hospital_scheduler.py`: Rooms 1 + 3
at 08:00, Room 4 at 08:02, Room 2 at 08:04.

## 3. Edit the pharmacy inventory - `config/inventory.yaml`

```yaml
slots:
  S1: {medicine: Paracetamol, dose: 500 mg, cap: white,  quantity: 1}
  S5: {medicine: Omeprazole,  dose: 20 mg,  cap: black,  quantity: 0}   # empty slot
```
- 6 slots, S1..S6, 0.10 m apart on the pick counter. One container per slot:
  `quantity` 1 = a container stands there, 0 = empty.
- A medicine may sit in several slots. The scheduler takes the first filled slot.
- `cap`: white, yellow, purple, grey or black. Blue, red and green are reserved for the
  tape and the floor markers.
- After changing a medicine, dose or cap, regenerate the container models (labels and
  caps) and reinstall:
  ```bash
  python3 ~/sdp_sim/tools/make_system_world.py && bash ~/sdp_sim/install.sh
  ```

## 4. Single patient room (the earlier stage)

```bash
ros2 launch robot_gazebo hospital.launch.py gui:=false
python3 ~/sdp_sim/scripts/jetarm_pick_place.py home
python3 ~/sdp_sim/scripts/mission.py                       # one full mission (11 steps)
python3 ~/sdp_sim/scripts/scheduler.py --in-minutes 1 --once   # the first schedule entry, 1 min from now
```
The schedule is `schedule.yaml`: `patient`, `medicine`, `time` ("HH:MM" daily or
"YYYY-MM-DD HH:MM" once) and `bed` (only bed 1 exists).

Other single-room tools:
- `scripts/mission.py --from N --to M`: run only some steps.
- `scripts/smart_table.py raise|lower|status`: control the table by hand.
- `python3 tools/sim_screenshot.py OUT.png [--wrist]`: a picture from the fixed overview
  camera, no GUI needed.

## 5. Add a room (developer task - not tested)

All four sides of the two junctions are taken (J2 is a T junction at the west end). A new
room therefore needs a new junction further west. Every number lives in
`tools/system_layout.py`; the worlds, the robot's welds and the map are generated from it.

1. In `tools/system_layout.py`:
   - add the junction, e.g. `JUNCTIONS = {'J1': -0.75, 'J2': -3.65, 'J3': -6.55}`
     (2.90 m pitch);
   - add the rooms, e.g. `5: ('J3', 'N'), 6: ('J3', 'S')` in `ROOMS`;
   - replace the places that assume J2 is the west end: `CORR_X0`, the `edges` list in
     `graph()` (add `('J2', 'J3')`) and the drawing code (`grep -n "J2" tools/system_layout.py`).
2. In `tools/make_system_world.py` (`floor_lines`) and `tools/offline_route_sim.py` (the
   synthetic floor), the main line ends at `JUNCTIONS['J2']`. Change both to the
   westernmost junction.
3. Run `python3 tools/system_layout.py` (clearance checks, `results/system/layout.png`),
   then `python3 tools/make_system_world.py` and `bash install.sh`. The launch file builds
   its bridges and the smart-table controller from `config/hospital_map.yaml`, so no
   launch change is needed.
4. Give the room a patient in `config/doses.yaml`. Then run the offline tests and
   `tools/test_route_gazebo.py --room 5` before a real delivery. The unit tests check
   the legs of Rooms 1-4 explicitly, so add the new legs there.

The planner, follower and scheduler work on the map graph and need no change. Expect a
lower real-time factor: four rooms already cost 22 % compared with the single room.
