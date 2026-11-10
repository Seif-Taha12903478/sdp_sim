# Hospital system - PHASE 0 plan (branch hospital-system, 2026-11-10)

Status: PLAN ONLY, waiting for the user's OK. Nothing is built. main is untouched.
Layout: results/system/layout.png, drawn by results/system/plan_layout.py (prints the
route table; the room furniture is imported from tools/make_hospital_world.py).

## 1. Layout (world frame: x east, y north; robot home/spawn (0, 0) facing +x)
- Corridor 1.2 m wide, x -4.25 .. 1.55, blue main line on y = 0.
- 4-way junction J1 at x -0.75 (Room 1 north, Room 2 south), T junction J2 at x -3.65
  (Room 3 north, Room 4 south); green 15 cm squares. Pitch 2.90 m.
- Each room = the tested room (tools/make_hospital_world.py furniture, own coordinates)
  moved to its junction: north rooms unrotated, south rooms rotated 180 deg about the
  junction. So the robot <-> smart-table geometry is identical in all 4 rooms (stop
  0.2777 m before the marker, place target 0.20 m left, platform edge 0.17 m left,
  turn-round circle 0.29 m from the table). Branch: junction -> stop 1.62 m, -> red
  marker 1.90 m. Door 0.8 m beside the bed head, room sign "Room N" next to it.
  Room 2.80 x 2.60 m inside: far wall 0.12 m closer than in the current room (chair
  still 8 cm clear), near wall 0.095 m outside the turn-round circle.
- Pharmacy at the east end, north side: medication cabinet (shelves + boxes, visual)
  behind a pick counter (top 0.10 m, front edge y 0.17, like the old station).
  6 slots S1..S6 at y = 0.20, x 0.022 .. 0.522 (0.10 m apart).
- Red markers: pharmacy P at x 0.55 (end of the line), HOME at x -0.08 (under the
  parked robot), R1..R4 at the branch ends. Marker spacing on the main line >= 0.63 m.
- Longest one-way route 5.54 m (pharmacy -> Room 3/4). Building 7.7 x 6.8 m.

## 2. Routes and times (every delivery: home -> pharmacy slot -> room -> home)
Time model = measured step times of the 2026-11-05 missions: line 13.2 s/m (2.957 m
in 39.0 s), pick 20.0, arm home 3.6, place 16.4, lift 18.3, turn round 16.4; junction
turn +7 s (stop, drive onto the marker, rotate 90); slot alignment 1 s + 20 s/m.
Wall = sim / RTF: 5.5 % = today's missions (5.1-6.0 %), 4.5 % = expected in the bigger world.

| delivery | leg 1 home->P | leg 2 P->room | leg 3 room->home | sim | wall @5.5 % | @4.5 % |
|---|---|---|---|---|---|---|
| Room 1 | 0.27 m: stop at P | 2.64 m: turn round, J1 right, stop R1 | 2.01 m: turn round, J1 left, stop HOME, park +0.36 m | 186 s | 56 min | 69 min |
| Room 2 | 0.27 m | 2.64 m: turn round, J1 left, stop R2 | 2.01 m: J1 right, stop HOME | 186 s | 56 min | 69 min |
| Room 3 | 0.27 m | 5.54 m: turn round, (pass HOME), J1 straight, J2 right, stop R3 | 4.91 m: J2 left, J1 straight, stop HOME | 262 s | 80 min | 97 min |
| Room 4 | 0.27 m | 5.54 m: J1 straight, J2 left, stop R4 | 4.91 m: J2 right, J1 straight | 262 s | 80 min | 97 min |
| demo 1-4 | | | | 897 s | 4.5 h | 5.5 h |
Optional cap-colour check: +7 s sim per delivery. "pass HOME" (blind, 0 s extra) is planned
for slots S2-S6; after picking from S1 the HOME marker is already under the robot.

## 3. RTF risk
- Measured (interleaved, 2026-11-05): 1 room + table 8.74 % vs. 9.37 % without (-6.7 %,
  inside the run-to-run spread 6.7-10.3 %). Missions run at ~0.6-0.7 x the idle RTF.
- New world: ~490 visuals (now 111), 4 smart tables (12 lift joints, 4 contact sensors,
  4 joint-state streams), 6 resting containers + 6 DetachableJoint plugins, ~45 more
  bridged topics. Wrist camera cost unchanged (it only sees the floor near the robot);
  the overview camera renders the whole building.
- Expectation: -15..-30 % -> 6.5-8 % headless idle (budget 7 %: borderline), missions
  ~4-5 %. Then R3/R4 deliveries ~90-110 min wall, demo ~5-6 h.
- Measure first (step b), interleaved system/main/system/main. If < 7 %, simplify in
  this order and report: (1) overview camera only in the photo/video worlds (or 0.2 Hz),
  (2) fewer cylinders (5-leg bases, casters, rail posts: 52 of the 96 room visuals are
  cylinders), (3) one static model per room (fewer entities), (4) collisions only on
  table, counter, bed; (5) only if still short: one generated mesh per room - not
  primitives, so I would ask first.

## 4. Files
New (all names new, so installing them does not touch main's world/launch/xacro):
- tools/system_layout.py - layout constants (this plan's numbers), shared by world, map, planner, layout PNG
- tools/make_system_world.py - writes robot_gazebo/worlds/hospital_system{,_video,_photo}.sdf,
  textures (room signs, container labels, 4 monitor screens), 6 container SDFs, config/hospital_map.yaml;
  colour check (no blue/red, and now no green outside the junction markers) + clearance checks for all rooms
- robot_gazebo/launch/hospital_system.launch.py; robot_gazebo/urdf/jetarm_mecanum_system.gazebo.xacro
  (current robot + one DetachableJoint per container: /med_S1/attach|detach|attached ...)
- config/hospital_map.yaml (graph), config/inventory.yaml, config/doses.yaml
- scripts/route_planner.py (Dijkstra -> actions), scripts/route_follower.py (extends line_follower),
  scripts/delivery.py (one delivery mission), scripts/pharmacy.py (stock, spawn + release, cap check),
  scripts/hospital_scheduler.py
- tools/offline_route_sim.py (synthetic camera + kinematic base), tools/test_system_offline.py
  (planner, scheduler, scoring unit tests), tools/test_pharmacy_pick.py (Gazebo step c)
Changed (backward compatible; main's behaviour unchanged):
- scripts/line_follower.py (PD step as a method, same output), scripts/smart_table.py (several
  tables from a config; default = main's single table), scripts/medicine_weld.py (topic prefix),
  scripts/mission.py (helpers take the container name), tools/make_hospital_world.py (furniture
  functions take names/textures; hospital.sdf must regenerate byte-identical), tools/scoring.py
  (+ score_delivery), tools/record_video.py (delivery captions), tools/measure_rtf.py (--launch),
  install.sh (+ system files), CLAUDE.md.
Note: the launch files run ~/sdp_sim/scripts from the working tree, so while this branch is
checked out main's world runs the branch's smart_table.py/planar_base.py - hence backward compatible.

## 5. Design decisions to confirm
1. Junctions: stop on the green marker, drive onto its centre and rotate +-90 deg on /odom; straight
   and "pass red" = blind drive on /odom with heading hold (~0.45 m) while still watching for markers,
   so a branch can never steer the robot. Markers are identified by colour + order + odometry window
   (+-0.20 m of the planned position); a wrong / missing marker stops the robot with an error.
2. Pharmacy: the counter runs PARALLEL to the line on the robot's left, so "strafe along the counter"
   = closed-loop forward/back on /odom along the line (|dx| <= 0.25 m); the robot never leaves the
   line, no strafe back needed, it turns round at the slot (strafe away from the counter first).
3. Home next to the pharmacy (robot dock): adds only ONE turn round per delivery (at the pharmacy).
4. Containers med_S1..S6 spawned once after the robot, each weld released + confirmed immediately,
   never re-spawned. qty = physical containers per slot (1, or 0 = empty slot); a medicine may be in
   several slots (Paracetamol S1 + S6); out of stock = no filled slot left.
5. Cap colours white / yellow / amber / grey / black (no blue, red or green); the smart table's green
   status LED becomes amber in the system world. The old container's cap is blue - fine before
   (never in the camera strips), not with neighbours in view at the counter.
6. Optional medicine check: cap colour only (joint1 +90 deg in the home pose, cap ~0.26 m from the
   lens); the printed label (name + dose) is too small for the 320 x 200 camera - photos only.

## 6. Tests (headless), estimated wall time
a. offline: planner (all routes), synthetic camera (first the OLD route as regression - that test
   script is not in the repo, I rewrite it), all 8 legs with start offsets, scheduler/scoring unit tests: minutes
b. world loads, RTF interleaved (4 x ~6 min), 2-3 overview screenshots media/system_*.png: ~45 min
c. pick + replace at S1, S3, S6 (no line following), neighbours checked: ~150 sim s, ~45-60 min
d. one delivery to Room 3: ~262 sim s, ~80-110 min
e. --demo, recorded media/system_demo.mp4: ~900 sim s, ~4.5-6 h (keep the laptop awake)
f. no repeated evaluations without asking

## 7. Other risks
- Ogre 1 has crashed occasionally when models were added: 6 spawns at start-up -> relaunch if it happens.
- Gripper fingers open ALONG the counter (toward the neighbours): pads ~+-3.5-4 cm when open, 0.10 m
  slot pitch leaves ~4.5 cm to the neighbour's edge - checked in test c with all neighbours present.
- Never tested so far: several deliveries in one simulation (all missions so far: one per launch).
