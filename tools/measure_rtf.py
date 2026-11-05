#!/usr/bin/env python3
"""Measure the real-time factor (RTF) of a fresh headless launch, same setup every time.

    python3 ~/sdp_sim/tools/measure_rtf.py [--seconds 180] [--label TEXT] [--launch-arg video:=true]

Steps: no simulation may be running -> `ros2 launch robot_gazebo hospital.launch.py
gui:=false` -> wait for the 3 controllers -> arm home (jetarm_pick_place.py home,
so the wrist camera looks at the floor as in a mission) -> wait --settle wall s ->
record (monotonic wall time, sim time) pairs from /joint_states stamps for --seconds
-> RTF = d(sim) / d(wall), also per 30 s window -> shut the launch down.
The robot stays at the start, so this measures physics + both cameras (wrist 4 Hz,
overview 0.5 Hz) with the whole world loaded. One line per measurement is appended
to --out (default ~/sdp_sim/results/rtf.csv); the launch log goes next to it.
"""
import argparse
import csv
import datetime as dt
import os
import statistics as st
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import evaluate as ev           # noqa: E402  (start/stop helpers of the evaluation)


def measure(seconds, window=30.0):
    import rclpy
    from sensor_msgs.msg import JointState
    rclpy.init()
    node = rclpy.create_node('measure_rtf')
    pts = []

    def on_js(m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if not pts or t > pts[-1][1]:
            pts.append((time.monotonic(), t))

    node.create_subscription(JointState, '/joint_states', on_js, 50)
    t_end = time.monotonic() + seconds
    while time.monotonic() < t_end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()
    if len(pts) < 2:
        return None, [], pts
    rtf = (pts[-1][1] - pts[0][1]) / (pts[-1][0] - pts[0][0])
    wins, i0 = [], 0
    for i, (w, s) in enumerate(pts):
        if w - pts[i0][0] >= window:
            wins.append((s - pts[i0][1]) / (w - pts[i0][0]))
            i0 = i
    return rtf, wins, pts


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--seconds', type=float, default=180.0, help='wall seconds to measure')
    p.add_argument('--settle', type=float, default=20.0, help='wall seconds to wait after arm home')
    p.add_argument('--label', default='')
    p.add_argument('--out', default=os.path.join(ROOT, 'results', 'rtf.csv'))
    p.add_argument('--launch-arg', action='append', default=[], help='extra launch argument, e.g. video:=true')
    a = p.parse_args()
    if 'CAMERA_TYPE' not in os.environ:
        sys.exit('CAMERA_TYPE is not set (source ~/.bashrc or export CAMERA_TYPE=GEMINI)')
    if ev.gazebo_pids():
        sys.exit('a simulation is already running (%s) - stop it first' % ev.gazebo_pids())
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    stamp = dt.datetime.now().strftime('%Y%m%d_%H%M%S')
    launch_log = os.path.join(os.path.dirname(a.out), 'rtf_%s_launch.log' % stamp)
    fh = open(launch_log, 'w')
    proc = subprocess.Popen(['ros2', 'launch', 'robot_gazebo', 'hospital.launch.py', 'gui:=false'] + a.launch_arg,
                            stdout=fh, stderr=subprocess.STDOUT, start_new_session=True)
    rtf, wins, pts = None, [], []
    try:
        err = ev.wait_ready(proc, launch_log, 900)
        if err:
            sys.exit('launch failed: %s' % err)
        ev.log('controllers active; arm home')
        r = subprocess.run([sys.executable, os.path.join(ROOT, 'scripts', 'jetarm_pick_place.py'), 'home'],
                           capture_output=True, text=True, timeout=1800)
        ev.log('arm home: %s' % (r.stdout.strip().splitlines()[-1:] or r.stderr.strip()[-200:]))
        time.sleep(a.settle)
        ev.log('measuring for %.0f wall s' % a.seconds)
        rtf, wins, pts = measure(a.seconds)
    finally:
        left = ev.stop_sim(proc, fh)
        if left:
            ev.log('leftover processes %s' % left)
    if rtf is None:
        sys.exit('no /joint_states received')
    sd = st.stdev(wins) if len(wins) > 1 else 0.0
    ev.log('RTF %.4f (%.2f %%) over %.0f wall s / %.1f sim s; 30 s windows: mean %.4f std %.4f min %.4f max %.4f'
           % (rtf, 100 * rtf, pts[-1][0] - pts[0][0], pts[-1][1] - pts[0][1],
              st.mean(wins) if wins else rtf, sd, min(wins or [rtf]), max(wins or [rtf])))
    new = not os.path.exists(a.out)
    with open(a.out, 'a', newline='') as fo:
        w = csv.writer(fo)
        if new:
            w.writerow(['started', 'label', 'git_commit', 'launch_args', 'rtf', 'wall_s', 'sim_s',
                        'win_mean', 'win_std', 'win_min', 'win_max', 'launch_log'])
        commit = subprocess.run(['git', '-C', ROOT, 'rev-parse', '--short', 'HEAD'],
                                capture_output=True, text=True).stdout.strip()
        w.writerow([stamp, a.label, commit, ' '.join(a.launch_arg), round(rtf, 5),
                    round(pts[-1][0] - pts[0][0], 1), round(pts[-1][1] - pts[0][1], 2),
                    round(st.mean(wins), 5) if wins else '', round(sd, 5), round(min(wins), 5) if wins else '',
                    round(max(wins), 5) if wins else '', launch_log])
    print('appended to', a.out)


if __name__ == '__main__':
    main()
