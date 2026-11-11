#!/usr/bin/env python3
"""Test step c of the hospital system: pick and replace at 3 slots (no line following).

    python3 ~/sdp_sim/tools/test_pharmacy_pick.py [--slots S1 S3 S6] [--no-launch]

Fresh headless launch of hospital_system.launch.py (unless --no-launch: use the running
one) -> arm home -> scripts/pharmacy.py setup -> cap look at EVERY slot (calibration of
the optional cap-colour check: arm turned left, wrist image, HSV of the cap region) ->
for each tested slot:
    line up with the slot on /odom (forward/back along the line, as a delivery does),
    pick (delivery.pick: the tested sequence, this container's own weld),
    replace it on the counter (delivery.release: above, descend, detach, open, retreat),
    arm home, then check:
      - the container is back in its slot (<= 10 mm, upright, on the 0.10 m counter)
      - no other container moved (<= 2 mm) - the open gripper spreads along the counter
      - attach and detach confirmed
-> robot back home -> shutdown. Results: results/system/pick_test_<date>/ (log, JSON, cap images).
"""
import argparse
import datetime as dt
import json
import math
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, 'scripts'))
import evaluate as ev           # noqa: E402

NEIGHBOUR_TOL = 0.002


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--slots', nargs='+', default=['S1', 'S3', 'S6'])
    p.add_argument('--no-launch', action='store_true')
    p.add_argument('--no-caps', action='store_true', help='skip the cap look at every slot')
    p.add_argument('--caps-only', action='store_true',
                   help='only the cap look (no picks); every cap read is then a pass/fail check')
    a = p.parse_args()
    stamp = dt.datetime.now().strftime('%Y%m%d_%H%M%S')
    out = os.path.join(ROOT, 'results', 'system', 'pick_test_' + stamp)
    os.makedirs(out, exist_ok=True)
    proc = fh = None
    res = {'started': stamp, 'slots': {}, 'caps': {}, 'checks': []}
    t0 = time.monotonic()
    if not a.no_launch:
        if ev.gazebo_pids():
            sys.exit('a simulation is already running (%s) - stop it first' % ev.gazebo_pids())
        proc, fh = ev.start_sim(os.path.join(out, 'launch.log'), 'hospital_system.launch.py')
        err = ev.wait_ready(proc, os.path.join(out, 'launch.log'), 1200)
        if err:
            ev.stop_sim(proc, fh)
            sys.exit('launch failed: ' + err)
        ev.log('controllers active after %.0f s' % (time.monotonic() - t0))

    def check(name, ok, msg=''):
        res['checks'].append({'check': name, 'ok': bool(ok), 'msg': msg})
        ev.log('%s %s %s' % ('PASS' if ok else 'FAIL', name, msg))
        return ok

    node = None
    try:
        r = subprocess.run([sys.executable, '-u', os.path.join(ROOT, 'scripts', 'pharmacy.py'), 'setup'],
                           capture_output=True, text=True, timeout=3600)
        open(os.path.join(out, 'setup.log'), 'w').write(r.stdout + r.stderr)
        if not check('pharmacy setup', r.returncode == 0, (r.stdout.strip().splitlines() or [''])[-1]):
            raise RuntimeError('setup failed')
        import rclpy
        import cv2
        import numpy as np
        from sensor_msgs.msg import Image
        rclpy.init()
        import jetarm_pick_place as jp
        from base_moves import BaseMover
        from medicine_weld import Weld
        import pharmacy as ph
        import delivery as dv
        node = jp.PickPlace()
        base = BaseMover(node)
        node.wait_ready()
        pharm = ph.Pharmacy()
        check('arm home', node.arm(jp.INIT_POSE, 3.0) is not None)
        frames = {}

        def on_img(m):
            frames['last'] = m
        node.create_subscription(Image, '/depth_cam/depth_cam', on_img, 2)

        def wrist_after(t_min):
            frames.pop('last', None)
            end = time.monotonic() + 600
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.2)
                m = frames.get('last')
                if m and m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 > t_min:
                    arr = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, :m.width * 3]
                    arr = arr.reshape(m.height, m.width, 3)
                    return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR) if m.encoding == 'rgb8' else arr.copy()
            return None

        def at_slot(s):
            sx, sy = pharm.slots[s]
            return dv.line_up(base, sx, sy - pharm.m['pharmacy']['reach_left'], 0.0)

        # ---- cap look at every slot (calibration of the optional cap check)
        if not a.no_caps:
            look = [math.pi / 2] + jp.INIT_POSE[1:]
            for s in sorted(pharm.stocked()):
                x, y, yaw, t = at_slot(s)
                node.arm(look, 3.0)
                img = wrist_after(base.pose()[3] + 0.3)
                if img is None:
                    check('cap image %s' % s, False, 'no wrist frame')
                    continue
                cv2.imwrite(os.path.join(out, 'cap_%s.png' % s), img)
                got, hsv = ph.classify_cap(img, ph.CAP_ROI)
                want = pharm.inv[s]['cap']
                res['caps'][s] = {'expected': want, 'seen': got, 'hsv': hsv, 'match': got == want}
                if a.caps_only:
                    check('cap %s read' % s, got == want, 'expected %s, seen %s, HSV %s, robot x %.4f'
                          % (want, got, hsv, x))
                else:
                    ev.log('cap look %s (calibration, not a pass/fail check): expected %s, seen %s, HSV %s'
                           % (s, want, got, hsv))
                node.arm(jp.INIT_POSE, 3.0)

        # ---- pick and replace
        for s in ([] if a.caps_only else a.slots):
            name = ph.container_name(s)
            weld = Weld(node, '/' + name)
            before = ph.gz_all_poses('med_')
            x, y, yaw, t = at_slot(s)
            info = res['slots'][s] = {'robot_at_slot': [x, y, math.degrees(yaw)], 'before': {
                k: list(v[0]) for k, v in before.items()}}
            ev.log('slot %s: robot at (%.4f, %.4f, %.2f deg)' % (s, x, y, math.degrees(yaw)))
            dv.WELD_LOG.clear()
            try:
                c = dv.pick(node, weld, name)
                info['lifted_z'] = c[0][2]
                node.arm(jp.INIT_POSE, 3.0)             # carry it once, like a delivery
                dv.release(node, weld)
                node.arm(jp.INIT_POSE, 3.0)
            except RuntimeError as e:
                check('pick + replace %s' % s, False, str(e))
                info['error'] = str(e)
                break
            info['weld'] = list(dv.WELD_LOG)
            after = ph.gz_all_poses('med_')
            info['after'] = {k: list(v[0]) for k, v in after.items()}
            p1 = after.get(name)
            sx, sy = pharm.slots[s]
            err = math.hypot(p1[0][0] - sx, p1[0][1] - sy) if p1 else None
            info['replace_error_mm'] = 1000 * err if err is not None else None
            check('weld %s confirmed' % s, all(w['confirmed'] for w in dv.WELD_LOG),
                  ', '.join('%s %d' % (w['kind'], w['attempts']) for w in dv.WELD_LOG))
            check('%s back in its slot' % name, pharm.in_slot(s, p1),
                  '%.1f mm from the slot, z %.4f, tilt %.1f deg' % (1000 * err, p1[0][2], ph.tilt_deg(p1[1])) if p1 else 'missing')
            moved = {k: 1000 * math.hypot(after[k][0][0] - v[0][0], after[k][0][1] - v[0][1])
                     for k, v in before.items() if k != name and k in after}
            info['neighbours_moved_mm'] = moved
            worst = max(moved.values()) if moved else 0.0
            check('neighbours of %s unmoved' % s, worst <= 1000 * NEIGHBOUR_TOL, 'max %.2f mm' % worst)
        base.goto(0.0, 0.0, 0.0)
        x, y, yaw, t = base.pose()
        check('back home', math.hypot(x, y) < 0.01, '(%.4f, %.4f, %.2f deg)' % (x, y, math.degrees(yaw)))
    except Exception as e:      # noqa: BLE001 - report and shut down
        res['error'] = repr(e)
        ev.log('ERROR %r' % e)
    finally:
        if node is not None:
            node.destroy_node()
            import rclpy
            if rclpy.ok():
                rclpy.shutdown()
        if proc is not None:
            res['leftover_processes'] = ev.stop_sim(proc, fh)
        res['wall_s'] = round(time.monotonic() - t0, 1)
        n_bad = sum(1 for c in res['checks'] if not c['ok'])
        res['passed'] = '%d/%d' % (len(res['checks']) - n_bad, len(res['checks']))
        with open(os.path.join(out, 'pick_test.json'), 'w') as fo:
            json.dump(res, fo, indent=2)
        ev.log('%s checks passed; results in %s' % (res['passed'], out))
    sys.exit(0 if not res.get('error') and all(c['ok'] for c in res['checks']) else 1)


if __name__ == '__main__':
    main()
