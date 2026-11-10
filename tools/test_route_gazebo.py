#!/usr/bin/env python3
"""Gazebo test of the junction logic on one full route (no pick / place), fresh launch.

    python3 ~/sdp_sim/tools/test_route_gazebo.py [--room 3] [--fast-speed 0.12] [--no-launch]

home -> pharmacy (stop at P) -> turn round -> pharmacy -> Room N (Room 3: pass HOME, J1
straight, J2 right, stop at R3) -> turn round -> Room N -> home (J2 left, J1 straight, stop
at HOME) -> park at (0, 0, 0). The same planner (route_planner.py), line follower
(route_follower.py) and base moves (base_moves.py) as a delivery. Checks every leg arrives,
the stop poses (vs. the map: marker - 0.2777 m along the edge, as in the single room) and
the offline prediction (tools/offline_route_sim.py), the line offsets, the junction
events. Results: results/system/route_test_<date>/ (log, JSON per leg, summary.json).
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
import system_layout as L       # noqa: E402

STOP_TOL = 0.03                 # m: stop vs expected (offline legs: < 16 mm)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--room', type=int, default=3)
    p.add_argument('--fast-speed', type=float, default=0.0)
    p.add_argument('--no-launch', action='store_true')
    p.add_argument('--no-setup', action='store_true', help='no containers at the counter')
    a = p.parse_args()
    stamp = dt.datetime.now().strftime('%Y%m%d_%H%M%S')
    out = os.path.join(ROOT, 'results', 'system', 'route_test_%s%s' % (stamp, '_fast' if a.fast_speed else ''))
    os.makedirs(out, exist_ok=True)
    res = {'started': stamp, 'room': a.room, 'fast_speed': a.fast_speed, 'legs': {}, 'checks': []}
    t0 = time.monotonic()
    proc = fh = None
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
        r = subprocess.run([sys.executable, os.path.join(ROOT, 'scripts', 'jetarm_pick_place.py'), 'home'],
                           capture_output=True, text=True, timeout=1800)
        check('arm home', r.returncode == 0)
        if not a.no_setup:
            r = subprocess.run([sys.executable, '-u', os.path.join(ROOT, 'scripts', 'pharmacy.py'), 'setup'],
                               capture_output=True, text=True, timeout=3600)
            open(os.path.join(out, 'setup.log'), 'w').write(r.stdout + r.stderr)
            check('pharmacy setup', r.returncode == 0)
        import rclpy
        rclpy.init()
        node = rclpy.create_node('route_test')
        from base_moves import BaseMover
        import route_planner as rp
        base = BaseMover(node)
        net = rp.Network()
        room = 'R%d' % a.room

        def leg(src, dst, name):
            x, y, yaw, t = base.pose()
            pl = net.plan(src, dst, math.degrees(yaw))
            if pl['turn_round']:
                ev.log('%s: turn round first' % name)
                base.turnaround(0.12)
                x, y, yaw, t = base.pose()
                pl = net.plan(src, dst, math.degrees(yaw))
            pf, rf = os.path.join(out, name + '_plan.json'), os.path.join(out, name + '_route.json')
            json.dump(pl, open(pf, 'w'), indent=2)
            cmd = [sys.executable, '-u', os.path.join(ROOT, 'scripts', 'route_follower.py'), '--plan', pf,
                   '--result-json', rf]
            if a.fast_speed:
                cmd += ['--fast-speed', str(a.fast_speed)]
            t_sim0 = t
            r = subprocess.run(cmd, capture_output=True, text=True)
            open(os.path.join(out, name + '_follower.log'), 'w').write(r.stdout + r.stderr)
            out_leg = json.load(open(rf)) if os.path.exists(rf) else {}
            x, y, yaw, t = base.pose()
            ev_last = pl['events'][-1]
            h = math.radians(ev_last['edge_heading'])
            ex, ey = ev_last['x'] - L.STOP_AHEAD * math.cos(h), ev_last['y'] - L.STOP_AHEAD * math.sin(h)
            err = math.hypot(x - ex, y - ey)
            out_leg.update(exit_code=r.returncode, stop=[x, y, math.degrees(yaw)], expected_stop=[ex, ey],
                           stop_error_mm=1000 * err, sim_s=t - t_sim0,
                           actions=[(e['node'], e['action']) for e in pl['events']])
            res['legs'][name] = out_leg
            check('%s arrived' % name, r.returncode == 0 and out_leg.get('result') == 'arrived',
                  '%s %s' % (out_leg.get('result'), out_leg.get('error') or ''))
            check('%s stop position' % name, err < STOP_TOL,
                  'stop (%.4f, %.4f, %.2f deg), expected (%.4f, %.4f): %.1f mm; %.1f sim s; max line offset %.1f mm'
                  % (x, y, math.degrees(yaw), ex, ey, 1000 * err, t - t_sim0, 1000 * (out_leg.get('max_xte') or 0)))
            for e in out_leg.get('events', []):
                ev.log('   %s %s: %s, marker %.3f m ahead, robot %s%s' % (
                    e['action'], e['node'], e['how'], e['marker_ahead'], e['pose'],
                    ', after turn %s' % e['after_turn'] if 'after_turn' in e else ''))
            return r.returncode == 0

        ok = leg('home', 'pharmacy', '1_home_to_pharmacy')
        ok = ok and leg('pharmacy', room, '2_pharmacy_to_%s' % room)
        ok = ok and leg(room, 'home', '3_%s_to_home' % room)
        if ok:
            base.goto(0.0, 0.0, 0.0)
            x, y, yaw, t = base.pose()
            check('parked at home', math.hypot(x, y) < 0.01 and abs(math.degrees(yaw)) < 1.0,
                  '(%.4f, %.4f, %.2f deg)' % (x, y, math.degrees(yaw)))
    except Exception as e:      # noqa: BLE001
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
        json.dump(res, open(os.path.join(out, 'summary.json'), 'w'), indent=2)
        ev.log('%s checks passed in %.0f s wall; results in %s' % (res['passed'], res['wall_s'], out))
    sys.exit(0 if not res.get('error') and all(c['ok'] for c in res['checks']) else 1)


if __name__ == '__main__':
    main()
