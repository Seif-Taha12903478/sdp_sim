# TrackMed - troubleshooting

## The simulation is very slow
Expected on small machines. On the development VM (VMware, 2 CPU cores) the real-time
factor (RTF) is:
- ~9-10 % headless for the single room and ~8 % for the hospital system: 1 sim second
  takes 10-12 real seconds;
- 2-6 % with the Gazebo window open, or when the VM is busy.

What that means in practice:
- one single-room mission takes ~50 min;
- one hospital delivery takes 60-100 min;
- the 4-delivery demo takes ~5.7 h.

What helps:
- Run headless: `gui:=false`. The cameras still render.
- Keep the machine idle. Busy terminals (scrolling output) and browsers cost CPU.
- Give the VM more cores. The camera rendering (wrist camera 4 Hz) is the main cost.

Check the RTF: `ign topic -e -n 1 -t /stats` (`real_time_factor`), or
`python3 tools/measure_rtf.py`.

Everything waits on simulated time, so a slow machine gives the same results, only
later. Never add wall-clock sleeps or wall-clock timeouts: at 4 % RTF, 3.5 s of wall time
is 0.14 s of sim time.

If the laptop sleeps, the VM pauses and the wall clock jumps. `evaluate.py` / `rescore.py`
mark such runs `wall_time_valid=false`.

## The Gazebo window is black / blank
The ogre2 renderer shows a blank view in VMware VMs. Both launch files pass
`--render-engine ogre` (Ogre 1). If you start Gazebo yourself, add that flag too.

Ogre 1 sometimes segfaults when models are added. If Gazebo crashes, check for leftovers
(next section) and relaunch.

## Two simulations are running / topics behave strangely / spawn goes to the wrong world
A launch that dies (for example on an exception in the launch file), or a
`timeout`-limited `ign gazebo`, can leave `ign gazebo` server or GUI processes running.
Both worlds are called `robot_world`, so a second simulation shares topics and services
with the first and both share the CPU.

Before every launch this must print nothing:
```bash
pgrep -af "ign gazebo"
```
Kill leftovers by PID, `kill <pid>` (then `kill -9 <pid>` if needed). Avoid `pkill -f`
with broad patterns: they can match your own shell's command line. The tools that launch
their own simulation (`evaluate.py`, `measure_rtf.py`, `run_in_sim.py`, `run_demo.py`,
`system_photos.py`, `test_route_gazebo.py`, `test_pharmacy_pick.py`) refuse to start while
a simulation is running, and check that nothing is left afterwards.

## "Error finding file" / a model is not spawned but ros_gz_sim says "OK creation of entity"
Give `ros2 run ros_ign_gazebo create -file PATH` an ABSOLUTE path. The Gazebo server
resolves a relative path against its own working directory, not yours.

The generated worlds also reference textures and container models by absolute path.
They are committed for `/home/ubuntu/sdp_sim`. `install.sh` regenerates them automatically
when the repo is elsewhere. By hand:
`python3 tools/make_hospital_world.py && python3 tools/make_system_world.py`.

The launch files start the helper scripts from `~/sdp_sim/scripts/`, so clone the repo to
`~/sdp_sim`.

## Launch fails with "Unable to parse the value of parameter robot_description as yaml"
A xacro `Command(...)` passed as `robot_description` must be wrapped in
`ParameterValue(..., value_type=str)` (Humble). The included launch files already do
this; it matters only if you write your own.

## xacro fails / the robot is missing
`CAMERA_TYPE` must be set: `export CAMERA_TYPE=GEMINI` (put it in `~/.bashrc`). Also
check the Hiwonder packages are built:
`ros2 pkg prefix jetarm_6dof_description robot_gazebo`.

## The robot does not follow the line / the camera sees the gripper
A fresh launch starts the arm in its zero pose. Run
`python3 scripts/jetarm_pick_place.py home` first, so the wrist camera looks at the floor.
`scripts/line_follower.py` writes a debug image to `/tmp/line_follower_debug.jpg`.

The colour thresholds were tuned under the world's ambient light of 0.8. Do not change
the lighting or the tape colours without re-tuning.

## The container is not picked / not released
The grasp uses a DetachableJoint weld. Its state topic (`/med_S<k>/attached`, or
`/medicine/attached` in the single room) publishes only on change. `medicine_weld.py`
resends up to 5 times and the logs show "confirmed after N attempt(s)".

A container spawned into a running simulation is NOT welded again: the weld forms only
when the container first appears. Restart the simulation instead of re-spawning.

## The scheduler exits with code -6 after finishing
A known, harmless abort inside rclpy shutdown, after all results are written. Check
`summary.txt` / `deliveries.csv` for the outcome.

## GitHub Actions / offline tests fail with "missing Python modules"
`tools/run_offline_tests.sh` needs ROS 2 Humble (it sources `/opt/ros/humble/setup.bash`
when it finds it) and `python3-opencv python3-numpy python3-yaml python3-matplotlib`.
