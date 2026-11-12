# SDP simulation: medicine-delivery robot (JetArm Pro on mecanum chassis)

Senior design project of a computer-engineering student. The real robot is broken,
so this Gazebo simulation is the final deliverable. Keep explanations short and
practical; the user reviews everything.

## Next session
Repo prepared for the professors (2026-11-12, on main): README.md (entry point, project name
TrackMed), docs/ (ARCHITECTURE, TESTING, USER_GUIDE, TROUBLESHOOTING), LICENSE (MIT; Hiwonder
files NOT in the repo - their package.xml declare no licence), tools/run_offline_tests.sh (6
groups, also run by .github/workflows/offline-tests.yml in ros:humble), media/system_demo_preview.gif,
tag v1.0. Visibility / collaborators are the user's job - never change them.
Branch hospital-system (from main bedafd6) is DONE: demo attempt 2 (2026-11-11 13:13-18:54)
4/4 SUCCESS (results/system/demo_20261111_131344/summary.txt, media/system_demo.mp4 113.8 s).
MERGED into main by the user's request on 2026-11-11 (fast-forward, main = 2a08b1d; branch
hospital-system kept). The single-room version still runs from main (`ros2 launch robot_gazebo
hospital.launch.py`, unchanged files); its last pure state is commit bedafd6. Report bundle: ~/Desktop/sdp_system_bundle.zip (REPORT_NOTES.md inside).
Open (not started): placement 0.5-1.7 mm (robot stops 0.8 mm short of the slot when coming from
+x, containers sit ~0.9 mm -x of their slot after spawn); scheduler exits -6 at rclpy shutdown
after finishing (harmless abort); root cause of the frozen joint-state cache unknown (recovered
by check_cache, 0 recoveries needed in attempt 2). Earlier, on main: report bundle
~/Desktop/sdp_report_bundle.zip; open decision: a 5-run evaluation with the smart table (ask first).

## Hospital system (branch hospital-system, 2026-11-10)
- Layout = tools/system_layout.py (the numbers; `python3 tools/system_layout.py` prints the
  clearance checks + route table and draws results/system/layout.png). Corridor 1.2 m,
  x -4.25..1.55, main line y = 0. J1 x -0.75 (4-way: Room 1 north, Room 2 south), J2 x
  -3.65 (T: Room 3 north, Room 4 south): green 15 cm markers. Red markers: pharmacy P x
  0.55 (line end), HOME x -0.08 (under the parked robot), R1..R4 at (Jx, +-1.90). Robot
  home = spawn (0, 0) facing +x (odom = world), right next to the pharmacy counter.
- Every room = the tested single room (make_hospital_world.py *_parts() furniture, room
  coordinates) as ONE static model; south rooms rotated 180 deg about the junction, so
  stop (0.2777 m before the marker), place target (0.20 m left), platform edge (0.17 m
  left) and turn-round circle are identical. Room 2.80 x 2.60 m inside (far wall 12 cm
  closer than in the single room). Junction -> room stop 1.622 m, -> marker 1.900 m.
- Pharmacy: pick counter top 0.10 m, front edge y 0.17 (like the old station), 6 slots
  S1..S6 x 0.022..0.522 (0.10 m apart) at y 0.20; cabinet visual. Turn round there =
  strafe right (south, away from the counter) 0.12, rotate, strafe right: the 0.27 m
  circle is 0.290 m from the counter edge at every slot (need > 0.285).
- Generator tools/make_system_world.py (never edit outputs): hospital_system{,_video,
  _photo}.sdf (459 visuals vs 111), robot_gazebo/worlds/system/ (textures, med_S*.sdf -
  spawned by absolute path), urdf/jetarm_mecanum_system.gazebo.xacro (the robot + one
  DetachableJoint per container: /med_S<k>/attach|detach|attached), config/hospital_map.yaml
  (graph). Colour check: no blue/red, and no green with S >= 0.40 in hue 40-90 (junction
  green) - status/power LEDs are amber in this world. make_hospital_world.py still writes
  hospital*.sdf byte-identical (checked).
- Launch: `ros2 launch robot_gazebo hospital_system.launch.py gui:=false [video|photo:=true]`
  (bridges built from the map; ONE smart_table.py --map for /smart_table_1..4). install.sh
  installs the system files under NEW names only.
- Containers: `scripts/pharmacy.py setup` spawns them one by one AFTER the robot (weld forms
  at first appearance) and releases each weld confirmed; never re-spawned. Inventory
  config/inventory.yaml: qty 1 per slot (0 = empty); Paracetamol in S1 and S6.
- Navigation: scripts/route_planner.py (Dijkstra -> actions pass/straight/left/right/stop,
  turn round at dead ends); scripts/route_follower.py = the tested follower (analyse/steer
  unchanged; steer() is the old PD block moved into a method) + look-ahead band (image rows
  50-95 %, 0.19-0.35 m): a marker must match colour, order and its odometry window (centre
  0.15-0.62 m ahead when seen; red stop 0.15-0.45), else stop with an error. straight /
  pass = BLIND on /odom (edge heading held, sideways correction) until the marker centre is
  0.05 m ahead; left/right = stop, translate onto the junction centre, rotate to the edge
  heading on /odom. --fast-speed V on straights (>= 0.8 m before / 0.3 m after a marker).
- Rendered colours (Gazebo): junction green HSV (68, 203, 255) (detector 45-85, S >= 120),
  line (111, 252, 255) as in the single room; nothing else in the green range.
- Mission scripts/delivery.py (15 steps: start check, arm home, line to P, line up with the
  slot on /odom (goto), [cap check], pick (own weld), arm home, turn round, line to room,
  place (room's table), arm home, lift, turn round, line home, park). Scheduler
  scripts/hospital_scheduler.py (--demo: hospital clock 08:00, Rooms 1+3 due 08:00 -> Room
  1 first, Room 4 08:02, Room 2 08:04; --now ROOM; --list; real time Asia/Dubai). Scoring
  tools/scoring.score_delivery (right container on the right raised platform, others where
  they were). Video: record_video.py captions (hospital clock, room, patient, medicine,
  step) + --wrist-inset. Demo: tools/run_demo.sh -> run_demo.py (launch video world,
  recorder, scheduler --demo, shutdown, summary; STOP file).
- Tests: tools/offline_route_sim.py (synthetic camera, rewritten - the 2026-11-04 one was
  not in the repo: the TESTED follower on the old route stops at (1.6993, 1.4752) in
  38.8-39.0 s, Gazebo (1.6992, 1.4723) in 38.6-39.0 s; 51/51 delivery legs incl. +-12 mm /
  +-1 deg start offsets, stops within 16 mm; 51/51 at 0.12 m/s; 3/3 fault cases stop),
  tools/test_system_offline.py (19 unit tests), tools/system_photos.py (world loads, 4
  tables lowered, 6 containers in their slots, media/system_*.png), test_route_gazebo.py,
  test_pharmacy_pick.py, run_in_sim.py (test d: `-- python3 scripts/hospital_scheduler.py --now 3`).
- evaluate.gazebo_pids now matches the programs (a shell whose command line contained
  "ruby" and "ign gazebo" counted as a running simulation and blocked a launch).
- RTF (tools/measure_rtf.py, interleaved, results/rtf.csv, 2026-11-10): system world + 6
  containers 7.46 / 9.23 % (mean 8.35 %), single room 10.67 / 10.70 % (mean 10.68 %):
  -22 %, above the 7 % budget, no simplification.
- Gazebo route test Room 3 (tools/test_route_gazebo.py): 0.08 m/s 9/9 (stops 7 / 16 / 7 mm, legs
  76.9 + 69.1 sim s, 39 min wall); --fast-speed 0.12 9/9 (stops 7 / 5 / 5 mm, legs 65.6 + 57.9 s,
  33 min wall) -> 0.12 ADOPTED (scheduler default; 0.08 near markers and junctions).
- Pick test c (tools/test_pharmacy_pick.py, 2026-11-10, 107 min wall): 12/12. Pick + replace at
  S1/S3/S6: every attach/detach confirmed 1st try, back 0.4 / 1.3 / 0.7 mm from the slot, z 0.1300,
  tilt <= 0.1 deg, neighbours moved 0.00 mm; parked (0.002, 0, 0). Cap look (calibration only):
  white/grey/black/white OK, yellow (30,200,255) OK, AMBER seen as yellow (30,221,255) - amber
  renders like yellow; optional --cap-check stays off until the amber cap is darkened + rechecked.
- Test d (run_in_sim.py --deadline 23:30 -- hospital_scheduler.py --now 3, 2026-11-10 18:58-20:10,
  72 min wall, 232.3 sim s): SUCCESS. Patient C, Ibuprofen S3: attach/detach confirmed 1st try,
  junction events as in the route test, R3 stop (-3.6508, 1.6273, 89.97), placed 2.8 mm from the
  target, tilt 0.6 deg (single room: 0.2 mm / 0.0 deg - not investigated), lift 18.6 s, container on
  the raised platform z 0.830, the 5 other containers in their slots, parked (-0.002, 0, 0.03 deg).
  Row in results/deliveries.csv; logs results/system/delivery_room3_20261110_185826/.
- Fixes 2026-11-11: (1) the Room 3 placement offset came from the pharmacy line-up: translate()
  stops within 2 mm (robot 2.4 mm short of S3, container 3.2 mm off the gripper centre, kept by
  the weld -> placed 2.7 mm behind the target; 0.6 deg = transient at contact, final 0.0).
  delivery.line_up(): goto + slow fine translate to 0.5 mm (falls back to the coarse pose on a
  timeout). Cap-look run: robot 0.2 mm from every slot. (2) S3 cap amber -> purple (0.38, 0.08,
  0.38): rendered HSV (150, 186, 169); test_pharmacy_pick.py --caps-only 9/9, all 6 caps read
  right -> run_demo.py runs --cap-check by default (--no-cap-check). Delivery: arm home after the
  cap check. (3) STOP file: scheduler --stop-file, ends after the current delivery.
- Demo attempt 1 (2026-11-11 08:43-12:44, results/system/demo_attempt1_20261111_084304/, 4.0 h):
  Room 1 OK 1.6 mm, Room 3 OK 1.5 mm (fine line-up works; was 2.8), Room 4 FAILED at step 7 "arm
  home (carrying)": delivery.py's /joint_states cache froze at 0.040 rad error (both publishers
  showed the arm exactly home; the main thread kept spinning at ~40 % CPU without running the
  callback; root cause in rclpy not found), the 3600 s WALL arm timeout failed the delivery,
  Room 2 not run. RTF meanwhile 2 % (load avg ~16).
- Arm-wait fix (jetarm_pick_place.PickPlace._wait, 2026-11-11): timeout on SIM time (trajectory +
  30 s; wall clock only a 3 h backstop); check_cache(): no /joint_states callback for 15 s wall ->
  read one fresh message via a separate probe node/executor; newer than the cache = frozen ->
  load it, re-create the subscription, log "joint-state cache FROZEN"; arm HOME (target INIT_POSE)
  also accepted within 0.06 rad when not moving ('settled'; other moves still need 0.03).
  tools/test_arm_wait.py (offline, ROS domain 77, fake publisher): 7/7. Gazebo
  tools/test_arm_home_held.py (via run_in_sim, 21 min): PASS - carrying med_S1, arm home reached
  in 3.6 sim s (err 0.029 rad: carried arm home ends close to the 0.03 tol), and with the cache
  frozen on purpose: detected after 15 s, recovered, reached in 3.6 sim s.
- Demo attempt 2 (tools/run_demo.sh, cap check on, 2026-11-11 13:13-18:54, 5.68 h wall, clean
  shutdown, no Gazebo left): 4/4 SUCCESS on the hospital clock. Room 1 (S1 Paracetamol, due 08:00,
  late 0.0 min) placement 2.0 mm at contact / 1.7 final, 185.2 sim s, 67 min wall; Room 3 (S3,
  08:00, late 3.1) 1.4 / 1.4 mm, 237.7 s, 103 min; Room 4 (S4, 08:02, late 5.1) 0.6 / 0.5 mm,
  238.8 s, 93 min; Room 2 (S2, 08:04, late 7.0) 1.7 / 1.7 mm, 181.4 s, 74 min. All caps read
  right, every attach/detach confirmed 1st try, tables raised (lift 18.4-18.7 s, container z
  0.830, tilt 0.0), other containers moved <= 0.14 mm, parked within 2 mm; no cache recovery
  needed. Placement error = robot-at-slot x (+0.8 mm from +x, -0.2 from -x) - container x in
  the slot (-0.8..-1.0 mm). Video media/system_demo.mp4 (1707 frames, 113.8 s, 800x600).
- Status (2026-11-11 19:50): hospital-system DONE; bundle ~/Desktop/sdp_system_bundle.zip.

## Current state (2026-11-05 evening)
GitHub backup: repo https://github.com/Seif-Taha12903478/trackmed-medication-delivery-robot (renamed from sdp_sim on 2026-11-12; the local folder stays ~/sdp_sim) (remote
`origin`). gh 2.103.0 is installed in ~/.local/bin (no sudo), logged in,
`gh auth setup-git` done: `git push` works. Push after each commit.

main (= hospital-room since the fast-forward merge on 2026-11-05): hospital patient
room + smart bedside table that lifts the medicine to the patient (see "Stages" item
7). The beige-table state of the 5-run evaluation (eval_5runs_confirmed, 5/5) is
commit b60e882. Not done on purpose: a 5-run evaluation with the table - ask the user
first (`tools/evaluate.py` already scores the lift; ~50-60 min per run).
Earlier items (finish eval_5runs_confirmed, mission_confirmed.mp4, attach/detach
counts in deliveries.csv) are all DONE (2026-11-05).

## Scenario
At a scheduled time the robot picks a medicine container at the medicine station,
follows a blue floor line to the patient, stops at a red floor marker, places the
container on the bedside table, turns round and follows the line home. Since the
hospital-room merge (2026-11-05) the bedside table is a smart table: once the container rests on it, the
weld is released and the arm is home, it lifts the medicine from 0.10 m to 0.80 m
(bed height) before the robot turns round.

## Machine
- Ubuntu 22.04 VMware VM, ROS 2 Humble, Gazebo Fortress 6.16 (`ign gazebo`, not `gz sim`).
- 2 CPU cores (`nproc` = 2, checked 2026-11-04). Real-time factor is only ~4-6 % without
  the camera: 1 simulated second takes ~20 real seconds. Always wait on feedback (joint
  states, odometry), never on wall-clock sleeps, and use generous timeouts.
- Camera rendering is the big cost. Wrist camera at 10 Hz with the GUI: real_time_factor
  0.02-0.06 (`ign topic -e -n 1 -t /stats`). Since 2026-11-04 the wrist camera runs at
  4 Hz sim (measured 4.03 Hz); headless (`gui:=false`) the RTF is ~0.09 (9 %).
- Use `ros2 launch robot_gazebo hospital.launch.py gui:=false` for test runs: server only
  (`ign gazebo -r -s ...`, one process, no window); camera sensors still render.
- Only ONE simulation at a time. A launch that dies (e.g. on a launch-file exception) can
  leave `ign gazebo server/gui` running orphaned; two simulations then share the CPU and
  topics. Before launching: `pgrep -af "ign gazebo"` must be empty. Kill leftovers by PID
  (`pkill -f` patterns can match the calling shell's own command line).
- The ogre2 renderer shows a blank view in this VM. Always use `--render-engine ogre`
  (already in hospital.launch.py and in the patched worlds.launch.py). Ogre 1 sometimes
  segfaults when models are added; if Gazebo crashes, relaunch.
- `sudo` is passwordless (/etc/sudoers.d/90-ubuntu-nopasswd). Ask the user before any sudo.
- New terminals source ROS via ~/.bashrc. In non-interactive shells run
  `source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash` first.
  `CAMERA_TYPE=GEMINI` must be set (it comes from ~/ros2_ws/.typerc via .bashrc); xacro needs it.

## Code layout
- ~/sdp_sim is a git repository (branches main and hospital-room, see "Current state").
  Commit after every milestone that works.
- Hiwonder package: ~/ros2_ws/src/simulations/{jetarm_6dof_description,robot_gazebo,robot_moveit_config}.
  Do not edit Hiwonder files without a backup next to them. Our additions:
  - robot_gazebo/urdf/jetarm_mecanum.gazebo.xacro - arm on chassis, wrist camera, plugins
  - robot_gazebo/worlds/hospital.sdf - generated by ~/sdp_sim/tools/make_hospital_world.py
    (never edit the .sdf by hand; edit the generator and run it)
  - robot_gazebo/launch/hospital.launch.py
  Edit the copies in ~/sdp_sim/robot_gazebo and run `bash ~/sdp_sim/install.sh` (copies + colcon build).
- ~/sdp_sim/scripts/jetarm_pick_place.py - arm/gripper control with analytic IK (tested)
- ~/sdp_sim/scripts/line_follower.py - line follower (Gazebo-tested, see below)
- ~/sdp_sim/scripts/base_moves.py - closed-loop strafe / forward / rotate / turnaround on
  /odom (CLI and BaseMover class). Gazebo-tested.
- ~/sdp_sim/scripts/mission.py - stage 4 mission node (see Stages).
- ~/sdp_sim/scripts/scheduler.py + ~/sdp_sim/schedule.yaml - stage 5 scheduler.
- ~/sdp_sim/tools/evaluate.py - stage 6 evaluation; tools/record_video.py - videos.
- ~/sdp_sim/results/ (git-ignored) - run outputs; ~/sdp_sim/media/ - kept videos.
- ~/sdp_sim/tools/sim_screenshot.py - GUI-free screenshots (see Facts).
- ~/sdp_sim/scripts/planar_base.py - base driver, STARTED BY hospital.launch.py (see
  "Mobile robot model"). The installed launch file runs it from this path.
- ~/sdp_sim/scripts/medicine_container.sdf - 3 cm x 6 cm, 30 g cylinder
- ~/sdp_sim/scripts/smart_table.py - smart bedside table controller, STARTED BY
  hospital.launch.py; also TableMonitor (used by mission.py) and a CLI
  (`smart_table.py raise|lower|status`). Its constants (N_STAGES, platform centre/size,
  heights, speeds) are imported by tools/make_hospital_world.py and mission.py.
- ~/sdp_sim/tools/test_smart_table.py - stand-alone table test (fresh launch, robot at
  the start, arm NOT homed yet). ~/sdp_sim/tools/measure_rtf.py - RTF of a fresh
  headless launch (appends to results/rtf.csv).
- robot_gazebo/worlds/hospital_video.sdf (video:=true) and hospital_photo.sdf
  (photo:=true: + room cameras /room_cam/{a,b,c}/image, `sim_screenshot.py OUT --room`)
  are generated with hospital.sdf; monitor_screen.png (generated monitor texture) is
  referenced by its absolute path ~/sdp_sim/robot_gazebo/worlds/monitor_screen.png.
- Earlier patches (already applied): worlds.launch.py uses `--render-engine ogre`;
  robot.gazebo.xacro has a DetachableJoint plugin (backup .bak).

## Facts learned the hard way
- Arm joints joint1..joint5, gripper r_joint (1.4 open, ~0.59 touching the container).
  Five more gripper joints are mimics handled by ign_ros2_control.
- Read joint positions from `/joint_states`. `/arm_controller/state` never publishes here.
- Commands: `/arm_controller/joint_trajectory`, `/gripper_controller/joint_trajectory`.
- Kinematics in jetarm_pick_place.py match the URDF exactly (tf2_echo confirmed
  tool point (0.180, 0, 0.045) for the commanded pose). Coordinates are relative to the
  arm base (joint1 axis on the arm plate). Tool-down reach ~0.14-0.24 m at low heights.
- Pure friction grasp works but drops the real-time factor to ~0.3 % (six position-driven
  mimic joints with 2 g finger links pressing on the container). So the grasp is
  "assisted": fingers close to 0.75 without squeezing, then a DetachableJoint weld
  (parent link5, child model `medicine` link `body`) carries it; detach drops it under
  real physics. Topics: `ign topic -t /medicine/attach|/medicine/detach -m ignition.msgs.Empty -p "unused: true"`.
- DetachableJoint (assisted grasp): it welds the container to link5 when the container
  FIRST appears (state "attached"); a container re-spawned in a running simulation is
  NOT welded again. ONE-SHOT `ign topic -p` attach/detach commands GET LOST OR ARRIVE
  LATE: eval_5runs (2026-11-04, old code) had a lost detach (run 3: container carried
  in the gripper for the rest of the mission), a lost attach (run 4: container never
  left the station) and most likely a late detach (run 1: container on the floor).
  Use scripts/medicine_weld.py (Weld): persistent ROS publisher on /medicine/attach |
  /medicine/detach (std_msgs/Empty, bridged in hospital.launch.py), waits on sim time
  for "attached"/"detached" on /medicine/attached (std_msgs/String, published only on
  CHANGE; a repeated command logs "Already ..." and publishes nothing), resends up to
  5 times. A first detach after spawn was seen unconfirmed and the resend worked.
  mission.py also checks physically: container at the station after spawn, risen with
  the gripper after the lift, on the table within 30 mm of the target after the
  retreat and again before the robot moves away (stops the mission otherwise).
- NEVER judge motion on wall-clock time: jetarm_pick_place `_wait` used to call the
  gripper "settled" after 3.5 s WALL without movement; at ~4 % RTF that is 0.14 sim s,
  the start of the closing spline, so the mission saw the gripper still open (1.397).
  Now settling is judged on sim time (/joint_states stamps), only after the trajectory
  duration; mission.py also requires 0.35 < gripper < 1.2 after closing.
- Gripper grasp height: tool point 0.038 m above the surface the container stands on
  (pads at the container middle, claw tips ~6 mm above the surface).
- Humble launch files: a xacro `Command(...)` passed as `robot_description` must be wrapped
  in `ParameterValue(Command([...]), value_type=str)` (launch_ros.parameter_descriptions),
  otherwise launch dies with "Unable to parse the value of parameter robot_description
  as yaml". Fixed in hospital.launch.py on 2026-11-04.
- URDF `<gazebo reference="link"><mu1>/<mu2>` is applied ONLY to collisions without a
  `name` attribute (the URDF->SDF converter matches its default names
  `<link>_collision_N`). Named collisions silently keep mu=1. Check the generated SDF:
  `xacro file.xacro > r.urdf && ign sdf -p r.urdf | grep -A3 '<mu>'`.
- Screenshots: use `python3 ~/sdp_sim/tools/sim_screenshot.py OUT.png [--wrist]`. It saves
  the next frame of the fixed overview camera in the world (model `overview_cam`, topic
  /overview_cam/image, 800x500 @ 0.5 Hz sim, bridged in hospital.launch.py), rendered by
  the server, so it works headless and does not touch the GUI. ~8 s at 9 % RTF.
  `--wrist` also saves the wrist-camera frame. Camera pose: overview_camera() in
  tools/make_hospital_world.py.
- Older GUI-based ways (avoid): gnome-screenshot and ImageMagick are NOT installed.
  Gazebo's GUI service writes to ~/.ignition/gui/pictures/:
  `ign service -s /gui/screenshot --reqtype ignition.msgs.StringMsg --reptype ignition.msgs.Boolean --timeout 80000 --req 'data: ""'`.
  Whole X screen: `python3 -c "from PIL import ImageGrab; ImageGrab.grab(xdisplay=':0').save('/tmp/s.png')"`.
  The service answers `data: true` but only writes the file when the 3D view renders.
  If no file appears, the GUI may be blank (empty Entity Tree, sim time "N/A" - seen
  once after ~25 min, while the server kept running fine). Starting a second GUI client
  (`ign gazebo -g`) at that moment coincided with the whole launch getting SIGINT and
  shutting down; cause unknown, so do that only when the run is finished.
  The GUI screenshot service worked on 2026-11-05 (media/room_gui.png); an X-screen
  grab shows whatever window is in front (the terminal), not Gazebo.
- DART (gz-physics) and velocity-controlled joints: (1) a joint that sits ON its
  position limit ignores its velocity (servo) command - smart-table stages that sagged
  onto the limit before the controllers engaged stayed there (platform at 0.077 m).
  Keep joint limits a few cm beyond the commanded range. (2) The servo of a 4th
  prismatic joint in series never acts (that stage falls onto its limit), reproduced in
  a table-only world independent of joint name, joint/plugin order, inertia, collision
  and sensor; 3 in series or 4 each jointed to the base work. Hence 3 table stages.
- JointPositionController `<use_velocity_commands>true`: velocity = -error/dt clamped
  to cmd_max/cmd_min; holds against gravity exactly. Stream setpoints every step for
  a smooth profile (smart_table.py does, on /smart_table/joint_states).
- Contact sensor (Fortress): world plugin ignition-gazebo-contact-system + `<sensor
  type="contact"><contact><collision>top</collision><topic>/smart_table/contact</topic>`
  in the link. Publishes ignition.msgs.Contacts ONLY while touching (every physics step,
  sim stamps), scoped names `smart_table::platform::top` / `medicine::body::collision`;
  bridged as ros_gz_interfaces/msg/Contacts. "No contact" = no message for GAP_S.
- A container spawned 1 mm above a surface drops onto it in the first physics step,
  BEFORE the DetachableJoint weld forms: it is welded while touching (contact "steady").
- `ros2 run ros_ign_gazebo create -file PATH`: give an ABSOLUTE path. The Gazebo server
  resolves a relative one against ITS working directory (where the launch was started):
  "Error finding file", nothing spawned, yet ros_gz_sim still prints "OK creation of
  entity". mission.py / test_smart_table.py use absolute paths; a hand-typed
  `-file scripts/medicine_container.sdf` cost one stuck photo session (2026-11-05).
- Lighting: scene ambient is 0.8 (keep it - the line follower's HSV ranges were tuned
  under it). Materials with ambient = diffuse render flat/washed out; furniture uses
  ambient 0.6 x colour (FURNITURE_AMB in the generator). Textures work with Ogre 1:
  `<pbr><metal><albedo_map>/abs/path.png</albedo_map>` (monitor screen).
- RTF varies a lot between runs on this VM: the same world gave 6.7-10.3 % headless on
  one afternoon, and full missions 5.1 % and 6.0 % wall-for-sim an hour apart. Compare
  worlds only with interleaved runs of tools/measure_rtf.py and keep the VM idle
  meanwhile (terminal output redraws cost Xorg/gnome-terminal CPU).
- Two simulations at once again (2026-11-05): a test world started with `timeout 70 ign
  gazebo ...` was still alive when the next launch began; both worlds were called
  robot_world, so topics AND services (spawn!) were shared. Check `ps` for ANY `ign
  gazebo` before every launch, also after `timeout`-limited test runs.

## Mobile robot model (jetarm_mecanum.gazebo.xacro)
- Chassis dimensions are ESTIMATES from Hiwonder's installation photos: ~0.32 x 0.26 m,
  0.08 m wheels, arm plate 0.10 m above the floor (`plate_h`).
- Motion: PLANAR BASE (since 2026-11-04). The model is attached to the world:
  `world` -> base_x_joint (prismatic X) -> base_x_link -> base_y_joint (prismatic Y)
  -> base_y_link -> base_yaw_joint (continuous Z) -> chassis_link. The two
  intermediate links have mass 0.001 / inertia 1e-6 (massless links are dropped by the
  URDF->SDF converter). Joint effort limit 1000. Nothing touches the floor (skid spheres
  removed; the chassis body box is 4.5 cm above it), the wheels are visual only.
  Height is locked by the joints: z stays 0.00000.
- Driving: three Gazebo JointController systems (velocity mode) on ign topics
  `/base/vel_x`, `/base/vel_y`, `/base/vel_yaw` (world frame; bridged as
  std_msgs/Float64 by a second parameter_bridge `base_bridge` in hospital.launch.py).
  scripts/planar_base.py subscribes `/controller/cmd_vel` (Twist, ROBOT frame: linear.x,
  linear.y, angular.z, like the mecanum base), rotates it with the current yaw on every
  `/joint_states` message and publishes the joint velocities; it also publishes `/odom`
  (odom -> chassis_link, stamps copied from /joint_states = sim time) from the base joint
  positions/velocities. The last command is held until a new one arrives.
- `/joint_states` has two publishers: Gazebo JointStatePublisher (all joints, incl. the
  base joints) and ros2_control joint_state_broadcaster (arm + gripper only). Filter by
  joint name.
- No TF from /odom any more (the old OdometryPublisher's odom->chassis_link TF is gone);
  robot_state_publisher publishes world -> base_x_link -> base_y_link -> chassis_link
  from the joint states. Hiwonder's bridge still creates an (idle) ROS publisher on /odom
  and bridges /controller/cmd_vel to an ign topic nobody listens to; both harmless.
- Speed check after the switch (odom twist): 0.0800 m/s forward/back for 0.08, 0.0500
  sideways for 0.05, 0.500 rad/s for 0.5; forward at yaw 88.8 deg moved along world +y.
  /odom matches Gazebo ground truth. One physics step (0.02 s) of command latency: a
  timed open-loop 90 deg turn stops at 88.8 deg, so turn closed-loop on /odom yaw.
- Gazebo ground truth now needs composing: in `/world/robot_world/dynamic_pose/info`
  the model `robot` pose follows its first link (base_x_link, x only) and `chassis_link`
  is given relative to the model (y and yaw). World pose = model pose * chassis_link pose.
- History (2026-11-04), why VelocityControl was dropped: (a) the robot moved at ~7 % of
  cmd_vel because of floor friction with the 0.02 s step (mu*g*dt = 0.196 m/s lost per
  step); a frictionless floor fixed the speed. (b) But the model also rose at a constant
  ~0.001 m per sim second from spawn (5 cm by the end of a trip), unaffected by
  cmd_vel linear.z, gravity or contact; cause not found. The floor friction change was
  reverted (no floor contact now). Old files: scratchpad backup only.
- Model bugs fixed on the way: (1) the `jetarm_6dof` macro takes `**origin`, which
  inserts the CHILDREN of the passed block, so a bare `<origin z=plate_h/>` was dropped
  and the arm stood on the FLOOR inside the chassis. Now
  `<origin_block><origin .../></origin_block>`; base_joint is at z=0.10. Check:
  `xacro ... | grep -A4 'name="base_joint"'`. (2) mu on named collisions (see "Facts").
- Wrist camera `sim_cam_link` on link4: 320x200 @ 4 Hz, topic `/depth_cam/depth_cam`
  (bridged as sensor_msgs/Image). Image is upright; image-left = robot-left. In the arm's
  `home`/init pose (0, 0.7309, -1.6127, -1.6591, 0) it looks 56 deg down and the bottom
  strips see the floor ~0.19-0.27 m ahead of joint1. Camera sits 0.396 m above the floor
  at 54.1 deg pitch (tf2_echo chassis_link sim_cam_link, after the arm-height fix); the
  floor is >= 0.38 m from the lens along the view axis.
- Camera near clip is 0.15 m (was 0.03). Despite the 5 cm offset, gripper parts lie
  0.02-0.11 m in front of the lens just below the axis and showed up in the lower image
  as cut-open shells (the line visible "through" them). The clip hides gripper and held
  container without changing the floor view. Verified: frame now shows only the line.
  If the arm pose or camera mount changes, recheck that the floor stays beyond the clip.
- Model frame `chassis_link` is on the floor under joint1; +x = robot front.

## World (hospital.sdf), robot spawned at (0,0) facing +x
- Blue line: x from -0.22 to 1.2, 90-deg left arc (r 0.5), then x=1.7 up to y=1.75.
- Red markers: patient (1.7, 1.75), home (-0.22, 0).
- Medicine station on the robot's left at the start: container at (0, 0.20, 0.131)
  -> arm-relative pick (0, 0.20). Table tops are at 0.10 m = plate height.
- Patient stop (offline test): joint1 at ~(1.70, 1.46) facing +y. Bedside table is on the
  robot's left -> arm-relative place (0, 0.20).
- Home stop (offline test): joint1 at ~(0.07, 0) facing -x. Gazebo: (0.044, -0.001).
- Clearance robot <-> tables is only ~4-5 cm. Turning in place sweeps a 0.27 m radius
  circle, so strafe away from a table before rotating (mecanum: linear.y).
- Patient room (hospital-room, merged into main 2026-11-05) on the robot's left at the patient stop (all
  furniture static, visual-only except box collisions on the bed and the table; 111
  visuals; the generator asserts colours and clearances):
  - Smart bedside table: platform 25 x 25 cm centred (1.405, 1.472), top 0.10 m lowered
    / 0.80 m raised. Its ROBOT-side edge is x 1.53 = the old table's edge (4 cm to the
    wheels, 0.29 m from the turnaround centre); the place target (1.499, 1.472) is 3 cm
    inside that edge. A platform centred on the target would reach x 1.62, into the
    robot's body (collision box from x 1.61) - the user's spec asked for centred.
    Base 0.32 x 0.30 x 0.05 m (x 1.21-1.53, partly under the bed edge); 3 nested tubes
    lift1..3 (0.2333 m each, limits -0.03..+0.03 beyond), JointPositionControllers
    (velocity mode, cmd_max 0.02 m/s per stage), JointStatePublisher
    /smart_table/joint_states, contact sensor on the platform collision `top`.
  - Bed: mattress x 0.34-1.24, y 0.66-2.66 (head toward -y), top 0.58 m, back section
    raised 35 deg, table-side rail head-only and 0.74 m high; IV pole (0.10, 1.45),
    monitor stand (-0.02, 0.85), chair (-0.12, 2.30); walls x = -0.72 and y = 3.15.
  - medicine_station colour is neutral grey now (was pale lavender, hue 120).
  - Overview camera (3.6, -1.2, 2.3) -> (0.5, 1.3, 0.5), hfov 1.1: whole route + room.

## Line follower status
Offline test: a synthetic camera built from the same geometry + kinematic robot
model reached the patient marker (max path deviation 2.8 cm), with a 4 cm / 9 deg start
offset (4.1 cm), and on the return trip. Gazebo (2026-11-04): images are rgb8 (handled);
rendered line is HSV ~(111, 250, 255), inside BLUE_HSV; floor is (0, 0, 255). Gazebo runs
2026-11-04, default parameters, both `Result: arrived`:
  - VelocityControl base (z had drifted to 0.051): 38.4 sim s, stop (1.699, 1.450) yaw 90.0.
  - PLANAR base (current): 38.6 sim s, stop (1.6993, 1.4632) yaw 89.99 deg, z 0.00000,
    /odom identical to ground truth; red fraction 0.21 at the stop. Start offset was
    x +3 mm, yaw -0.57 deg. Within 3 mm of the offline prediction (1.70, 1.46).
  - PLANAR base, wrist camera 4 Hz, headless: 38.7 sim s, stop (1.6992, 1.4723) yaw
    89.99 (9 mm further than at 10 Hz: fewer frames before the 2-frame red check).
Return trip in Gazebo (2026-11-04, 4 Hz, headless), all closed-loop on /odom:
  - `base_moves.py turnaround 0.12`: strafe right 0.12, rotate 180, strafe right 0.12.
    (1.6992, 1.4723, 89.99 deg) -> (1.6993, 1.4729, -90.31 deg) in ~17 sim s; the
    wrist camera then sees the line centred (bedside table corner at the image edge).
  - `line_follower.py` home: `Result: arrived` after 38.2 sim s, stop (0.044, -0.001)
    yaw 180.01 deg (offline prediction ~0.07; red fraction 0.29). Still to tune live if needed:
`--stop-frac`, `--kp/--kd`. The node only logs once the first image arrives (can take
~10 s wall-clock).
Debug image: /tmp/line_follower_debug.jpg (look at it with the Read tool).

## Stages
1. DONE, tested in Gazebo 2026-11-04: world loads, line/markers/tables visible.
2. DONE, tested 2026-11-04: 3 controllers active, arm home OK, camera OK (after near-clip
   fix), drive OK at the commanded speed with the planar base (height locked).
3. DONE, tested in Gazebo 2026-11-04: line follower reaches the patient marker (one run).
4. DONE, tested 2026-11-04: `scripts/mission.py` (11 steps since the hospital-room merge,
   10 before (eval_5runs_confirmed): a new step 8 "table lifts medicine to the patient"
   comes after "arm home" and the later steps move up by one - see item 7; `--from/--to N`,
   `--result-json FILE`, `--label TEXT`): 1 spawn container (removes an old one) +
   detach, 2 arm home, 3 pick at arm-relative (0, 0.20) (open, above, descend, close
   0.75 + check > 0.35, attach, lift), 4 arm home (container held, out of the wrist view),
   5 line_follower.py to the patient, 6 place at (0, 0.20) (above, descend, detach, open,
   retreat), 7 arm home, 8 turnaround right 0.12, 9 line_follower.py home, 10 park at the
   start (turnaround LEFT 0.12 - the station is on the right when facing home - then
   goto (0, 0, yaw 0)) so the next mission can start. Line following runs as a
   subprocess; BaseMover.pose() waits for fresh /odom afterwards (a stale pose once
   nearly sent the turnaround across the room; translate() now also aborts if it moves
   away from its goal). Full run from a fresh launch (headless, video world), triggered
   by the scheduler: success, 161.5 sim s / 31.4 min wall, container at
   (1.4992, 1.4734, 0.1300), 0.1 mm from the target, tilt 0.0 deg; parked at
   (0.002, 0.000, -0.26 deg). Patient stop (1.699, 1.473).
5. DONE, tested 2026-11-04: `scripts/scheduler.py` + `~/sdp_sim/schedule.yaml`
   (deliveries: patient, medicine, time "HH:MM" daily or "YYYY-MM-DD HH:MM" once, bed).
   Wall-clock times; due deliveries are queued and run one after another; each is
   appended to results/deliveries.csv (+ the mission JSON), scored with tools/scoring.py
   (at_target, success = completed AND delivered at the target) and since 2026-11-05 with
   spawn/pick/place attach-detach attempts + unconfirmed commands; an older header is
   upgraded once (old rows kept, new columns empty). Test: `--in-minutes 1
   [--entry N] --once`; `--list` shows the next due times. Only bed 1 exists.
6. Evaluation: `tools/evaluate.py --runs N` (fresh headless launch per run, results in
   results/eval_DATE/: runs.csv, summary.txt, per-run launch/mission logs and JSON).
   success = mission completed AND container on the table within 30 mm of the target
   AND tilt < 10 deg (tools/scoring.py; tools/rescore.py re-scores a results dir from its
   JSON files). CSV/summary also: attach/detach attempts, unconfirmed commands, UTC
   start. Durations are monotonic. eval_5runs (old weld code, re-scored): 2/5 success;
   its `started` column is Hong Kong time (UTC+8; the VM moved to Asia/Dubai mid-run),
   use started_utc.
   eval_5runs_confirmed (confirmed weld + sim-time gripper wait; runs 1-3 2026-11-04,
   runs 4-5 appended 2026-11-05 with --append --total 5): 5/5 SUCCESS, placement error
   0.19 mm mean (max 0.20), tilt 0.0 deg, 160.2 sim s per mission, ~48 min wall
   (mission, n=4), every attach/detach confirmed on the first attempt.
   Run 4's wall time is INVALID: the laptop slept ~58 min during it (VM paused). The
   monotonic clock stops in a paused VM and the wall clock is stepped on resume, so the
   two disagree; evaluate.py (wall clock vs monotonic per run) and rescore.py (JSON
   start stamp + result-file time vs monotonic) now set wall_time_valid=false when they
   differ by > 60 s, and the stats leave such runs out. Manual: rescore.py --invalid-wall N. Result files (runs.csv, summary.txt, run JSON) of
   both evaluations are committed under results/ (force-added; results/ is ignored).
   Video: launch with `video:=true` (hospital_video.sdf: overview camera 2 Hz), then
   `tools/record_video.py OUT.mp4 --caption-log LOG` (H.264, 15 fps = 7.5x real time,
   caption = current mission step from the log, stops after "[mission] done").
   media/mission_overview.mp4: the 2026-11-04 scheduled full mission (23 s, BEFORE the
   confirmed weld). media/mission_confirmed.mp4: 2026-11-05 scheduled full mission with
   the confirmed weld (23 s, 800x500 H.264; success, container 0.2 mm from the target,
   all attach/detach confirmed first try) - use this one for the presentation.
7. DONE 2026-11-05, branch hospital-room, merged into main (fast-forward) the same day: hospital patient room +
   smart bedside table (user spec of 2026-11-05; layout in "World").
   - World: tools/make_hospital_world.py, primitives only (+ one generated texture),
     111 visuals, furniture static; the generator asserts colours (no blue/red hue)
     and clearances (>= 0.165 m from the line, outside both turning circles).
   - scripts/smart_table.py: LIFT when the container contact is steady >= 2 sim s
     (contact that began while lowered) AND /medicine/attached last said "detached" AND
     the arm is home (all joints within 0.06 rad); LOWER when a raised table lost its
     container for >= 3 s; trapezoidal 40 mm/s, 40 mm/s^2 on sim time (0.70 m in 18.6 s);
     manual raise/lower services refused while the gripper is near the table; after a
     manual lower with the container still on, no new lift until a new contact starts.
   - Mission (11 steps): 6 place waits for "lowered" first; the placement pose is read
     the moment contact_state turns "steady" (thread, place motion unchanged); new 8
     "table lifts medicine to the patient" waits for "raised" (--lift-timeout 90 sim s;
     on timeout it goes on with table_raised false); 9 checks the container at the
     platform's current height before turning. JSON 'placed' + 'table'. Scoring: placement
     from 'placed' (else the final pose - both old evaluations re-score identically);
     table runs also need table_raised AND container_on_platform_after_lift (centre over
     the platform, z = top + 0.03 +- 1 cm, tilt < 10 deg). Columns table_raised,
     lift_time_s, container_on_platform_after_lift in runs.csv and deliveries.csv.
   - Table test (tools/test_smart_table.py; results/room_mission/table_test.*): 28/29
     checks - the failed one was a wrong test assumption (container touches while
     welded), fixed afterwards, test not rerun. No lift while welded or with the arm away;
     lift 0.10 -> 0.7997 m in 18.6 sim s, cruise 39.9 mm/s; container on the raised
     platform z 0.830, tilt 0.0 deg, slid 0.08 mm; auto-lower 3.04 s after removal;
     manual raise/lower OK, stays raised without a container.
   - Line follower in the new world: arrived, stop (1.6992, 1.4723, 89.99 deg) = same as
     before; the wrist frame at the stop shows only marker and line.
   - Scheduled missions (results/room_mission/, deliveries.csv): 2/2 success. Normal
     world: placement 0.27 mm at contact confirmation, tilt 0.13 deg, attach/detach
     1/1/1, raised in 18.56 s, container z 0.830 on the platform, 178.3 sim s / 58.5 min
     wall. Video world: 0.25 mm, raised in 18.68 s, 178.4 sim s / 49.8 min wall.
   - RTF (tools/measure_rtf.py, results/rtf.csv), interleaved after/before/after/before:
     8.99 / 10.33 / 8.50 / 8.41 % -> means 8.74 vs 9.37 % (-6.7 %), inside the
     run-to-run spread (before runs that afternoon: 6.75-10.33 %). No simplification.
   - Media: media/mission_room.mp4 (25.5 s, 800x500; second caption line "smart table:
     lifting, platform ... m"); media/room_gui.png (Gazebo GUI 3D view: the world loads
     in the GUI), room_corner.png and room_overview.png (robot at the start, table
     lowered), room_table_lowered.png / room_table_raised.png (close-up with the
     container), room_corner_raised.png (room corner, table raised with the container),
     room_patient_side.png (raised table with the container at bed height, seen from
     the far side of the bed; taken 2026-11-05 after an auto-lift: arm home, container
     spawned on the platform, detach -> raised 0.7996 m). Report photos: launch
     photo:=true, `sim_screenshot.py OUT --room`. The lowered table is hidden behind the
     mattress from the far side.
   - Not done on purpose: a 5-run evaluation with the table (ask the user first).

## First test procedure (stages 1-3)
1. `bash ~/sdp_sim/install.sh`, new terminal.
2. `ros2 launch robot_gazebo hospital.launch.py` (run in background, log to a file).
   Expect robot on chassis at the start, line visible. `ros2 control list_controllers` -> 3 active.
3. `python3 ~/sdp_sim/scripts/jetarm_pick_place.py home` (arm to ready pose).
4. `ros2 topic hz /depth_cam/depth_cam` - images arriving? Save one and look at it.
5. `ros2 topic pub --once /controller/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.05}}"`,
   watch /odom, then publish zeros. Check the robot moves without tipping or sinking.
6. `python3 ~/sdp_sim/scripts/line_follower.py` - should drive to the patient marker and
   print `Result: arrived`.
Screenshots: `python3 ~/sdp_sim/tools/sim_screenshot.py OUT.png --wrist`. A fresh launch starts the arm in its zero pose: always run `home` before
the line follower, or the camera does not look at the floor.
