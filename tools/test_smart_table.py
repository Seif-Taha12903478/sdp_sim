#!/usr/bin/env python3
"""Stand-alone test of the smart bedside table (no mission, robot stays at the start).

    python3 ~/sdp_sim/tools/test_smart_table.py [--out results/table_test.json] [--photo PREFIX]
    (--photo, photo world: room cameras saved as PREFIX_lowered_{a,b,c}.png at step 5
     and PREFIX_raised_{a,b,c}.png at step 6)

Needs a FRESH simulation (ros2 launch robot_gazebo hospital.launch.py gui:=false
[photo:=true]): the arm must still be in its launch (zero) pose, NOT at home, and no
container may exist yet. Sequence, every wait on sim time:
  1 table lowered at 0.10 m
  2 spawn the container on the platform 1 mm above the target (the DetachableJoint
    welds it at first appearance: "attached"; it has already dropped onto the platform
    by then, so it touches while welded) -> table stays lowered
  3 detach -> it drops onto the platform -> contact "steady" after 2 s; the arm is
    not at home -> table must STAY lowered
  4 attach (arm static, container resting) -> weld "attached" -> must stay lowered
  5 detach -> still lowered (arm not home)
  6 arm home -> the table must lift; check speed, the platform reaching 0.80 m and the
    container standing on it (x, y over the platform, z 0.83, tilt < 10 deg)
    [--photo PREFIX: save the room cameras now (photo world)]
  7 remove the container -> lowering must start >= 3 s later; platform back at 0.10 m
  8 manual raise (no container) -> raised, and stays raised (no container was removed)
  9 manual lower -> lowered
Writes a JSON report; exit code 0 = all checks passed.
"""
import argparse
import json
import math
import os
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'scripts'))
import rclpy                                    # noqa: E402
from std_msgs.msg import Float64                # noqa: E402
from std_srvs.srv import Trigger                # noqa: E402
import jetarm_pick_place as jp                  # noqa: E402
import smart_table as tbl                       # noqa: E402
from medicine_weld import Weld                  # noqa: E402
from mission import (CONTAINER_NAME, CONTAINER_SDF, WORLD, gz_model_pose_full,   # noqa: E402
                     tilt_deg)

CHECKS = []


def log(msg):
    print('[table-test] ' + msg, flush=True)


def check(name, ok, detail=''):
    CHECKS.append({'check': name, 'ok': bool(ok), 'detail': detail})
    log('%s  %s  %s' % ('PASS' if ok else 'FAIL', name, detail))
    return ok


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--out', default=os.path.join(ROOT, 'results', 'table_test.json'))
    p.add_argument('--photo', default=None, help='save the room cameras at the raised state (photo world)')
    p.add_argument('--x', type=float, default=1.4992, help='container x (default: mission place target)')
    p.add_argument('--y', type=float, default=1.4723)
    a = p.parse_args()

    rclpy.init()
    node = jp.PickPlace()
    weld = Weld(node)
    table = tbl.TableMonitor(node)
    heights = []                                       # (sim_t, platform top) during moves
    node.create_subscription(Float64, '/smart_table/height',
                             lambda m: heights.append((table.sim_t, m.data)), 50)
    report = {'checks': CHECKS}

    def stay(state, sim_s, why):
        ok = table.spin_until(lambda: table.state() != state, sim_s, wall_limit=1800.0)
        return check('stays %s for %.0f sim s (%s)' % (state, sim_s, why), not ok,
                     'state %s, platform %.3f m' % (table.state(), table.height or -1))

    def service(name):
        cl = node.create_client(Trigger, '/smart_table/' + name)
        cl.wait_for_service(timeout_sec=60.0)
        fut = cl.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, fut, timeout_sec=120.0)
        r = fut.result()
        return r.success if r else False, r.message if r else 'no answer'

    def photos(out):
        r = subprocess.run([sys.executable, os.path.join(ROOT, 'tools', 'sim_screenshot.py'), out, '--room',
                            '--timeout', '400'], capture_output=True, text=True)
        log('photos: ' + (r.stdout.strip() or r.stderr.strip()[-300:]).replace('\n', '; '))

    def pose():
        c = gz_model_pose_full(CONTAINER_NAME)
        return (list(c[0]), tilt_deg(c[1])) if c else (None, None)

    try:
        node.wait_ready()
        table.spin_until(lambda: table.state() is not None and table.height is not None, 10.0, wall_limit=120.0)
        # 1
        check('1 lowered at start', table.state() == 'lowered' and abs((table.height or 0) - tbl.LOWERED_TOP) < 0.003,
              'state %s, platform %s m' % (table.state(), table.height))
        arm0 = node.get(jp.ARM_JOINTS)
        check('arm not at home (fresh launch)', max(abs(q - h) for q, h in zip(arm0, jp.INIT_POSE)) > tbl.HOME_TOL,
              'arm %s' % [round(v, 3) for v in arm0])
        # 2
        n0 = len(weld.states)
        z0 = tbl.LOWERED_TOP + tbl.CONTAINER_HALF_H + 0.001
        r = subprocess.run(['ros2', 'run', 'ros_ign_gazebo', 'create', '-file', CONTAINER_SDF, '-name', CONTAINER_NAME,
                            '-x', str(a.x), '-y', str(a.y), '-z', str(z0)], capture_output=True, text=True, timeout=300)
        check('2 container spawned', r.returncode == 0, (r.stderr or r.stdout).strip()[-200:])
        check('2 welded at spawn (DetachableJoint)', weld.wait_state('attached', since=n0, sim_seconds=5.0))
        # the container drops the 1 mm onto the platform in its first physics step, before the
        # weld forms (seen: contact "steady" while welded), so this also shows: welded -> no lift
        stay('lowered', 3.0, 'container welded (contact %s)' % table.contact())
        report['contact_while_welded'] = table.contact()
        # 3
        ok, n = weld.command('detach')
        check('3 detach confirmed', ok, '%d attempt(s)' % n)
        t_det = weld.sim_t
        ok = table.spin_until(lambda: table.contact() == 'steady', 6.0, wall_limit=1800.0)
        t_st = [t for t, s in table.contacts if s == 'steady']
        check('3 contact steady after the drop', ok,
              'steady %.2f sim s after the detach' % (t_st[-1] - t_det) if t_st else 'contact %s' % table.contact())
        report['contact_steady_after_detach_s'] = (t_st[-1] - t_det) if t_st else None
        stay('lowered', 4.0, 'arm not at home')
        # 4
        ok, n = weld.command('attach')
        check('4 attach confirmed (container resting, arm static)', ok, '%d attempt(s)' % n)
        stay('lowered', 3.0, 'weld attached and arm not at home')
        # 5
        ok, n = weld.command('detach')
        check('5 detach confirmed', ok, '%d attempt(s)' % n)
        stay('lowered', 3.0, 'weld released, arm still not at home')
        c, tilt = pose()
        report['container_before_lift'] = c
        check('5 container resting on the lowered platform',
              c and tbl.on_platform(c, tilt, tbl.LOWERED_TOP), 'container %s tilt %s' % (c, tilt))
        if a.photo:
            photos(a.photo + '_lowered.png')
        # 6
        log('arm home ...')
        check('6 arm home', node.arm(jp.INIT_POSE, 3.0) is not None)
        n_h = len(heights)
        ok_l = table.wait_state('lifting', 5.0, wall_limit=1800.0) or table.state() == 'raised'
        check('6 lift starts once the arm is home', ok_l, 'state %s' % table.state())
        ok = table.wait_state('raised', 60.0, wall_limit=3600.0)
        lift = [(t, s) for t, s in table.states if s in ('lifting', 'raised')][-2:]
        dur = lift[-1][0] - lift[0][0] if len(lift) == 2 else None
        # cruise speed: platform between 0.2 and 0.7 m (times = arrival sim time, +-0.1 s)
        h = [x for x in heights[n_h:] if x[0] is not None and 0.2 <= x[1] <= 0.7]
        v = (h[-1][1] - h[0][1]) / (h[-1][0] - h[0][0]) if len(h) > 2 and h[-1][0] > h[0][0] else None
        report.update(lift_time_s=dur, lift_cruise_speed=v, lift_mean_speed=(tbl.TRAVEL / dur) if dur else None)
        check('6 raised', ok and abs((table.height or 0) - tbl.RAISED_TOP) < 0.003,
              'platform %.4f m, lifting -> raised %s sim s' % (table.height or -1, '%.1f' % dur if dur else '?'))
        check('6 lift speed 3-5 cm/s', v is not None and 0.03 <= v <= 0.05, 'cruise %.4f m/s' % (v or -1))
        table.spin_until(lambda: False, 1.0, wall_limit=600.0)
        c, tilt = pose()
        report['container_after_lift'] = c
        report['container_tilt_after_lift'] = tilt
        check('6 container on the raised platform', c and tbl.on_platform(c, tilt, table.height),
              'container %s tilt %.2f deg (expected z %.3f)' % (c, tilt or -1, (table.height or 0) + 0.03))
        if c and report.get('container_before_lift'):
            b = report['container_before_lift']
            check('6 container did not slide during the lift', math.hypot(c[0] - b[0], c[1] - b[1]) < 0.002,
                  '%.2f mm' % (1000 * math.hypot(c[0] - b[0], c[1] - b[1])))
        if a.photo:
            photos(a.photo + '_raised.png')
        stay('raised', 4.0, 'container on it')
        # 7
        rm = subprocess.run(['ign', 'service', '-s', '/world/%s/remove' % WORLD, '--reqtype', 'ignition.msgs.Entity',
                             '--reptype', 'ignition.msgs.Boolean', '--timeout', '60000',
                             '--req', 'name: "%s" type: MODEL' % CONTAINER_NAME],
                            capture_output=True, text=True, timeout=120)
        t_rm = table.sim_t
        check('7 container removed', 'true' in rm.stdout, rm.stdout.strip()[-100:])
        ok = table.wait_state('lowering', 10.0, wall_limit=1800.0)
        t_lo = [t for t, s in table.states if s == 'lowering']
        delay = (t_lo[-1] - t_rm) if t_lo else None
        report['lower_delay_after_removal_s'] = delay
        check('7 lowering starts >= 3 s after the removal', ok and delay is not None and delay >= tbl.LOST_S - 0.3,
              '%s sim s (removal request returned at sim %.2f)' % ('%.2f' % delay if delay else '?', t_rm))
        ok = table.wait_state('lowered', 60.0, wall_limit=3600.0)
        check('7 lowered again', ok and abs((table.height or 0) - tbl.LOWERED_TOP) < 0.003,
              'platform %.4f m' % (table.height or -1))
        # 8
        ok, msg = service('raise')
        check('8 manual raise accepted', ok, msg)
        ok = table.wait_state('raised', 60.0, wall_limit=3600.0)
        check('8 raised by hand', ok, 'platform %.4f m' % (table.height or -1))
        stay('raised', 6.0, 'manual raise, no container: no auto-lower')
        # 9
        ok, msg = service('lower')
        check('9 manual lower accepted', ok, msg)
        ok = table.wait_state('lowered', 60.0, wall_limit=3600.0)
        check('9 lowered by hand', ok, 'platform %.4f m' % (table.height or -1))
    except Exception as e:                       # noqa: BLE001  (report, then fail)
        check('no exception', False, repr(e))
    finally:
        report['states'] = [[t, s] for t, s in table.states]
        report['contacts'] = [[t, s] for t, s in table.contacts]
        report['passed'] = all(c['ok'] for c in CHECKS)
        os.makedirs(os.path.dirname(a.out), exist_ok=True)
        with open(a.out, 'w') as fh:
            json.dump(report, fh, indent=2)
        log('%d/%d checks passed; report %s' % (sum(c['ok'] for c in CHECKS), len(CHECKS), a.out))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(0 if report['passed'] else 1)


if __name__ == '__main__':
    main()
