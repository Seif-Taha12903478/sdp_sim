#!/usr/bin/env python3
"""World-load test + report pictures of the hospital system (fresh headless launch).

    python3 ~/sdp_sim/tools/system_photos.py [--out-dir ~/sdp_sim/media] [--no-setup]

no simulation may run -> `ros2 launch robot_gazebo hospital_system.launch.py gui:=false
photo:=true` -> controllers active -> all 4 smart tables report "lowered" -> arm home ->
scripts/pharmacy.py setup (6 containers spawned, welds released, all in their slots) ->
pictures: system_top.png (straight-down overview), system_building.png (whole building
from the south-east), system_corridor.png (down the corridor), system_pharmacy.png
(counter + containers), system_room1.png (room 1), system_wrist_home.png (wrist camera at
home) -> clean shutdown. Log and a JSON summary in results/system/photos_<date>/.
"""
import argparse
import datetime as dt
import json
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import evaluate as ev           # noqa: E402

PY = sys.executable
SCRIPTS = os.path.join(ROOT, 'scripts')
SHOTS = [('system_top.png', '/overview_cam/image'), ('system_building.png', '/room_cam/a/image'),
         ('system_corridor.png', '/room_cam/b/image'), ('system_pharmacy.png', '/room_cam/c/image'),
         ('system_room1.png', '/room_cam/d/image')]


def run(cmd, timeout, log_fh):
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    log_fh.write('$ %s\n%s%s\n' % (' '.join(cmd), r.stdout, r.stderr))
    log_fh.flush()
    return r


def table_states(timeout_s=300):
    """Latest state of every smart table, from their latched /smart_table_N/state topics."""
    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
    rclpy.init()
    node = rclpy.create_node('system_photos_tables')
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    st = {}
    for k in range(1, 5):
        node.create_subscription(String, '/smart_table_%d/state' % k, lambda m, k=k: st.__setitem__(k, m.data), qos)
    end = time.monotonic() + timeout_s
    while time.monotonic() < end and len(st) < 4:
        rclpy.spin_once(node, timeout_sec=0.5)
    node.destroy_node()
    rclpy.shutdown()
    return st


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--out-dir', default=os.path.join(ROOT, 'media'))
    p.add_argument('--no-setup', action='store_true', help='pictures without the containers')
    a = p.parse_args()
    if ev.gazebo_pids():
        sys.exit('a simulation is already running (%s) - stop it first' % ev.gazebo_pids())
    stamp = dt.datetime.now().strftime('%Y%m%d_%H%M%S')
    out = os.path.join(ROOT, 'results', 'system', 'photos_' + stamp)
    os.makedirs(out, exist_ok=True)
    os.makedirs(a.out_dir, exist_ok=True)
    summary = {'started': stamp, 'checks': {}}
    lf = open(os.path.join(out, 'steps.log'), 'w')
    proc, fh = ev.start_sim(os.path.join(out, 'launch.log'), 'hospital_system.launch.py', ['photo:=true'])
    t0 = time.monotonic()
    try:
        err = ev.wait_ready(proc, os.path.join(out, 'launch.log'), 1200)
        if err:
            raise RuntimeError('launch: ' + err)
        ev.log('controllers active after %.0f s' % (time.monotonic() - t0))
        st = table_states()
        summary['checks']['tables'] = st
        ev.log('smart tables: %s' % st)
        if sorted(st.values()) != ['lowered'] * 4:
            raise RuntimeError('not all 4 smart tables lowered: %s' % st)
        r = run([PY, os.path.join(SCRIPTS, 'jetarm_pick_place.py'), 'home'], 1800, lf)
        ev.log('arm home: exit %d' % r.returncode)
        if not a.no_setup:
            r = run([PY, os.path.join(SCRIPTS, 'pharmacy.py'), 'setup'], 3600, lf)
            summary['checks']['pharmacy_setup'] = r.returncode == 0
            ev.log('pharmacy setup: exit %d\n%s' % (r.returncode, '\n'.join(l for l in r.stdout.splitlines() if 'S' in l[:14] or 'FAIL' in l)))
            if r.returncode != 0:
                raise RuntimeError('pharmacy setup failed (see steps.log)')
        shots = {}
        for name, topic in SHOTS:
            path = os.path.join(a.out_dir, name)
            r = run([PY, os.path.join(HERE, 'sim_screenshot.py'), path, '--topic', topic, '--timeout', '900'] +
                    (['--wrist'] if name == 'system_top.png' else []), 1200, lf)
            shots[name] = r.returncode == 0 and os.path.exists(path)
            ev.log('%s: %s' % (name, 'saved' if shots[name] else 'FAILED'))
        wrist = os.path.join(a.out_dir, 'system_top_wrist.png')
        if os.path.exists(wrist):
            os.replace(wrist, os.path.join(a.out_dir, 'system_wrist_home.png'))
        summary['shots'] = shots
    except (RuntimeError, subprocess.TimeoutExpired) as e:
        ev.log('FAILED: %s' % e)
        summary['error'] = str(e)
    finally:
        left = ev.stop_sim(proc, fh)
        summary['leftover_processes'] = left
        summary['wall_s'] = round(time.monotonic() - t0, 1)
        with open(os.path.join(out, 'summary.json'), 'w') as fo:
            json.dump(summary, fo, indent=2)
        lf.close()
        ev.log('done in %.0f s; results in %s' % (summary['wall_s'], out))
    sys.exit(1 if summary.get('error') else 0)


if __name__ == '__main__':
    main()
