#!/usr/bin/env python3
"""Stage 4: one medicine-delivery mission in the hospital world.

    python3 ~/sdp_sim/scripts/mission.py            # full mission
    python3 ~/sdp_sim/scripts/mission.py --from 5   # resume at step 5 (see STEPS)

Needs a running simulation (ros2 launch robot_gazebo hospital.launch.py [gui:=false])
with the robot at the start (0, 0) facing +x.

Steps
  1 spawn the container on the medicine station and release the automatic weld
  2 arm home
  3 pick at arm-relative (0, 0.20): open, above, descend, close 0.75, attach, lift
  4 arm home (container stays in the gripper, out of the camera view)
  5 follow the line to the patient marker            (line_follower.py)
  6 place on the smart bedside table at arm-relative (0, 0.20): the table must be
    lowered (platform top 0.10 m); above, descend, detach, open, retreat. The
    placement is recorded the moment the platform's contact sensor confirms the
    container (steady for 2 sim s, /smart_table/contact_state), on the lowered table
  7 arm home
  8 the table lifts the medicine to the patient: scripts/smart_table.py lifts by
    itself once the contact is steady, the weld released and the arm home; wait for
    "raised" (--lift-timeout sim s; on timeout the mission goes on and records
    table_raised false), then check the container still stands on the platform
  9 turn round: strafe right, rotate 180 deg on /odom, strafe right (base_moves.py)
 10 follow the line home                             (line_follower.py)
 11 park at the start for the next mission: strafe LEFT (away from the medicine
    station, which is on the robot's right when it faces home), rotate 180 deg,
    strafe left, then go to (0, 0) yaw 0 on /odom

Reuses jetarm_pick_place.py (IK, arm/gripper control, attach/detach) and
base_moves.py. Every wait is on feedback (/joint_states, /odom, camera stamps).
At the end it prints the container's placement error (Gazebo ground truth vs. the
intended spot on the bedside table) and the cycle time in sim seconds.
Exit code 0 = mission complete. --result-json FILE writes all results (used by
scheduler.py and tools/evaluate.py).
"""
import argparse
import json
import math
import os
import re
import subprocess
import sys
import threading
import time

import rclpy

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import jetarm_pick_place as jp          # noqa: E402
from base_moves import BaseMover        # noqa: E402
from medicine_weld import Weld          # noqa: E402
import smart_table as tbl               # noqa: E402  (table geometry + TableMonitor)

CONTAINER_SDF = os.path.join(HERE, 'medicine_container.sdf')
CONTAINER_NAME = 'medicine'              # DetachableJoint child_model
STATION_XYZ = (0.0, 0.20, 0.131)          # container spawn (world), table top 0.10
REACH_XY = (0.0, 0.20)                    # arm-relative pick and place (left of joint1)
TABLE_Z = 0.13                            # container centre z when standing on a 0.10 table
PLACE_TOL = 0.03                          # m: container must stay this close to the target
WORLD = 'robot_world'


def log(msg):
    print('[mission] ' + msg, flush=True)


def gz_model_pose_full(name):
    """Gazebo ground truth of a model: ((x, y, z), (qx, qy, qz, qw)), or None."""
    t = subprocess.run(['ign', 'topic', '-e', '-n', '1', '-t', '/world/%s/pose/info' % WORLD],
                       capture_output=True, text=True, timeout=120).stdout
    m = re.search(r'name: "%s"\s*id: \d+\s*position \{([^}]*)\}(?:\s*orientation \{([^}]*)\})?'
                  % re.escape(name), t)
    if not m:
        return None
    # protobuf text output OMITS fields that are exactly 0, so every missing component
    # is 0 - including w (only a missing orientation block means identity)
    def f(k, s):
        r = re.search(r'\b' + k + r': ([-\d.e+]+)', s or '')
        return float(r.group(1)) if r else 0.0
    pos = (f('x', m.group(1)), f('y', m.group(1)), f('z', m.group(1)))
    if m.group(2) is None:
        return pos, (0.0, 0.0, 0.0, 1.0)
    q = [f(k, m.group(2)) for k in 'xyzw']
    n = math.sqrt(sum(v * v for v in q)) or 1.0
    return pos, tuple(v / n for v in q)


def gz_model_pose(name):
    """Gazebo ground truth (x, y, z) of a model, or None."""
    r = gz_model_pose_full(name)
    return r[0] if r else None


def tilt_deg(q):
    """Angle between the model's z axis and the world z axis (0 = upright)."""
    qx, qy, qz, qw = q
    return math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (qx * qx + qy * qy)))))


def spawn_container(weld):
    if gz_model_pose(CONTAINER_NAME) is not None:
        log('removing the old container')
        subprocess.run(['ign', 'service', '-s', '/world/%s/remove' % WORLD,
                        '--reqtype', 'ignition.msgs.Entity', '--reptype', 'ignition.msgs.Boolean',
                        '--timeout', '60000', '--req', 'name: "%s" type: MODEL' % CONTAINER_NAME],
                       capture_output=True, text=True, timeout=120)
    x, y, z = STATION_XYZ
    n0 = len(weld.states)
    r = subprocess.run(['ros2', 'run', 'ros_ign_gazebo', 'create', '-file', CONTAINER_SDF,
                        '-name', CONTAINER_NAME, '-x', str(x), '-y', str(y), '-z', str(z)],
                       capture_output=True, text=True, timeout=300)
    if r.returncode != 0:
        raise RuntimeError('spawn failed: ' + (r.stderr.strip() or r.stdout.strip()))
    # The DetachableJoint welds the container to the gripper when it first appears
    # (state "attached"); release it. A container re-spawned in a running simulation
    # is NOT welded again (no state message): then there is nothing to release.
    if weld.wait_state('attached', since=n0, sim_seconds=3.0):
        confirm(weld, 'detach', 'spawn')
    else:
        log('no automatic weld at spawn (re-spawned in a running simulation): nothing to release')
        WELD_LOG.append({'where': 'spawn', 'kind': 'detach', 'attempts': 0, 'confirmed': True,
                         'sim_t': weld.sim_t, 'note': 'not welded at spawn'})
    c = gz_model_pose(CONTAINER_NAME)
    if c is None or abs(c[2] - TABLE_Z) > 0.01 or math.hypot(c[0] - x, c[1] - y) > 0.01:
        raise RuntimeError('container not resting at the station after spawn: %s' % (c,))
    return c


WELD_LOG = []       # every attach/detach: where, attempts, confirmed (-> result JSON 'weld')


def confirm(weld, kind, where):
    ok, n = weld.command(kind)
    WELD_LOG.append({'where': where, 'kind': kind, 'attempts': n, 'confirmed': ok, 'sim_t': weld.sim_t})
    if ok:
        log('%s (%s) confirmed by /medicine/attached after %d attempt(s)' % (kind, where, n))
    else:
        # state message missing (e.g. "Already detached" publishes nothing): the pose
        # checks that follow decide
        log('WARNING: %s not confirmed by /medicine/attached after %d attempts '
            '(last state %s) - relying on the pose check' % (kind, n, weld.last_state()))
    return ok


def check_on_table(target, where, z=TABLE_Z):
    """Container must stand on the table (centre at height z) within PLACE_TOL of the target."""
    c = gz_model_pose_full(CONTAINER_NAME)
    if c is None:
        raise RuntimeError('%s: container not found in Gazebo' % where)
    (cx, cy, cz), q = c
    d = math.hypot(cx - target[0], cy - target[1])
    log('%s: container (%.4f, %.4f, %.4f) tilt %.1f deg, %.1f mm from target'
        % (where, cx, cy, cz, tilt_deg(q), 1000 * d))
    if abs(cz - z) > 0.01 or d > PLACE_TOL or tilt_deg(q) > 10:
        raise RuntimeError('%s: container is not standing on the table at the target '
                           '(z %.3f, expected %.3f, %.0f mm away, tilt %.0f deg)'
                           % (where, cz, z, 1000 * d, tilt_deg(q)))
    return c


class ContactPose:
    """Reads the container's Gazebo pose the moment the smart table reports a steady
    contact (TableMonitor callback). The pose is read in a thread, so the arm/gripper
    waits keep spinning: the place motion is not changed or delayed."""

    def __init__(self, table):
        self.result, self.thread, self.armed = None, None, False
        table.on_contact = self._on_contact

    def arm(self):
        self.result, self.thread, self.armed = None, None, True

    def _on_contact(self, state, sim_t):
        if self.armed and state == 'steady':
            self.armed = False
            self.thread = threading.Thread(target=self._grab, args=(sim_t,), daemon=True)
            self.thread.start()

    def _grab(self, sim_t):
        c = gz_model_pose_full(CONTAINER_NAME)
        self.result = {'source': 'contact sensor', 'sim_t': sim_t,
                       'xyz': list(c[0]) if c else None, 'quat_xyzw': list(c[1]) if c else None}

    def get(self, timeout=180.0):
        if self.thread is not None:
            self.thread.join(timeout)
        return self.result


def follow_line(extra=()):
    cmd = [sys.executable, '-u', os.path.join(HERE, 'line_follower.py')] + list(extra)
    log('running ' + ' '.join(os.path.basename(c) for c in cmd[2:]))
    r = subprocess.run(cmd, text=True, capture_output=True)
    tail = [l for l in r.stdout.splitlines() + r.stderr.splitlines() if l.strip()][-3:]
    for l in tail:
        log('  line_follower: ' + re.sub(r'\x1b\[[0-9;]*m', '', l))
    if r.returncode != 0:
        raise RuntimeError('line follower did not arrive (exit %d)' % r.returncode)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--from', dest='first', type=int, default=1, help='first step (1-11)')
    p.add_argument('--to', dest='last', type=int, default=11, help='last step (1-11)')
    p.add_argument('--result-json', default=None, help='write the results to this JSON file')
    p.add_argument('--label', default='', help='free text for the log, e.g. patient/medicine')
    p.add_argument('--offset', type=float, default=0.12, help='strafe distance for the turn (m)')
    p.add_argument('--duration', type=float, default=3.0, help='sim seconds per arm move')
    p.add_argument('--lift-timeout', type=float, default=90.0,
                   help='sim seconds to wait for the smart table to be raised (step 8)')
    p.add_argument('--table-timeout', type=float, default=120.0,
                   help='sim seconds to wait for the smart table to be lowered before placing')
    a = p.parse_args()

    # pick/place plan from jetarm_pick_place (same defaults as its CLI)
    pa = argparse.Namespace(pick=list(REACH_XY), place=list(REACH_XY), grasp_z=0.038,
                            release_z=0.045, above_z=0.14, open=1.4, closed=0.52,
                            carry_grip=0.75)
    plan = jp.build_plan(pa)
    d = a.duration

    rclpy.init()
    node = jp.PickPlace()
    base = BaseMover(node)
    weld = Weld(node)
    table = tbl.TableMonitor(node)
    contact_pose = ContactPose(table)
    result = {}

    def table_top():
        """Current platform top (m) from /smart_table/height."""
        table.spin_until(lambda: table.height is not None, 5.0, wall_limit=120.0)
        return table.height if table.height is not None else tbl.LOWERED_TOP

    def check(res, what):
        if res is None:
            raise RuntimeError(what + ' failed or timed out')
        return res

    def pick():
        check(node.gripper(pa.open, d), 'open gripper')
        check(node.arm(plan['pick_above'], d), 'move above container')
        check(node.arm(plan['pick_descent'], d / 2), 'descend')
        check(node.gripper(pa.carry_grip, d), 'close gripper')
        g = node.get(jp.GRIP_JOINTS)[0]
        log('gripper at %.3f rad' % g)
        if g < 0.35:
            raise RuntimeError('gripper closed almost fully: container missed')
        if g > 1.2:
            raise RuntimeError('gripper did not close (%.3f rad, open = %.2f)' % (g, pa.open))
        confirm(weld, 'attach', 'pick')
        check(node.arm(plan['pick_lift'], d / 2), 'lift')
        c = gz_model_pose(CONTAINER_NAME)        # it must have come up with the gripper
        if c is None or c[2] < TABLE_Z + 0.05:
            raise RuntimeError('container not lifted with the gripper (attach failed?): %s' % (c,))
        log('lifted: container z %.3f' % c[2])

    def place():
        # the smart table lowers by itself once a previous container has been taken off
        if not table.wait_state('lowered', a.table_timeout, wall_limit=7200.0):
            raise RuntimeError('smart table not lowered (state %s, platform %s m) - not placing'
                               % (table.state(), table.height))
        info = result['table'] = {
            'platform': {'center': list(tbl.PLATFORM_CENTER), 'size': list(tbl.PLATFORM_SIZE),
                         'top_lowered': tbl.LOWERED_TOP, 'top_raised': tbl.RAISED_TOP},
            'state_before_place': table.state(), 'height_before_place': table_top()}
        x, y, yaw, _ = base.pose()
        rx, ry = REACH_XY
        result['target'] = (x + math.cos(yaw) * rx - math.sin(yaw) * ry,
                            y + math.sin(yaw) * rx + math.cos(yaw) * ry)
        check(node.arm(plan['place_above'], d), 'move above table')
        check(node.arm(plan['place_descent'], d / 2), 'descend')
        contact_pose.arm()                      # pose read when the contact sensor confirms
        confirm(weld, 'detach', 'place')
        info['detach_sim_t'] = weld.sim_t
        check(node.gripper(pa.open, d), 'open gripper')
        check(node.arm(plan['place_retreat'], d / 2), 'retreat')
        if not table.spin_until(lambda: contact_pose.thread is not None, 10.0, wall_limit=1800.0):
            log('WARNING: the platform contact sensor did not confirm the container within 10 sim s '
                '(contact %s)' % table.contact())
        placed = contact_pose.get()
        c = check_on_table(result['target'], 'after retreat')
        if not placed or not placed.get('xyz'):
            placed = {'source': 'after retreat (no contact confirmation)', 'sim_t': table.sim_t,
                      'xyz': list(c[0]), 'quat_xyzw': list(c[1])}
        placed['table_state'], placed['table_height'] = table.state(), table.height
        result['placed'] = placed
        px, py, pz = placed['xyz']
        log('placed (%s, sim t %.2f): container (%.4f, %.4f, %.4f), %.1f mm from target, tilt %.1f deg, '
            'table %s at %.3f m' % (placed['source'], placed['sim_t'] or -1, px, py, pz,
                                    1000 * math.hypot(px - result['target'][0], py - result['target'][1]),
                                    tilt_deg(placed['quat_xyzw']), placed['table_state'], placed['table_height'] or -1))

    def table_lift():
        info = result.setdefault('table', {})
        t0 = table.sim_t
        ok = table.spin_until(lambda: table.state() == 'raised', a.lift_timeout, wall_limit=7200.0,
                              every=(2.0, lambda: log('table: %s, platform %.3f m'
                                                      % (table.state(), table.height or 0.0))))
        log('table: %s, platform %.3f m' % (table.state(), table.height or 0.0))   # last caption line
        t_place = (result.get('placed') or {}).get('sim_t') or info.get('detach_sim_t') or 0.0
        starts = [t for t, s in table.states if s == 'lifting' and t is not None and t >= t_place - 1.0]
        ends = [t for t, s in table.states if s == 'raised' and starts and t is not None and t >= starts[-1]]
        top = table_top()
        info.update(raised=bool(ok), lift_start_sim_t=starts[-1] if starts else None,
                    raised_sim_t=ends[0] if ends else None, height_after_lift=top,
                    states=[[t, s] for t, s in table.states])
        info['lift_time_s'] = (ends[0] - starts[-1]) if starts and ends else None
        info['wait_s'] = (ends[0] - t0) if ends else None
        c = gz_model_pose_full(CONTAINER_NAME)
        if c is not None:
            info['container_after_lift'] = {'xyz': list(c[0]), 'quat_xyzw': list(c[1])}
            on = tbl.on_platform(c[0], tilt_deg(c[1]), top)
            info['container_on_platform_after_lift'] = on
            log('table %s (platform %.3f m, lift %s s): container (%.4f, %.4f, %.4f) tilt %.1f deg - %s'
                % (table.state(), top, '%.1f' % info['lift_time_s'] if info['lift_time_s'] else '?',
                   c[0][0], c[0][1], c[0][2], tilt_deg(c[1]),
                   'on the platform' if on else 'NOT on the platform'))
        if not ok:
            log('WARNING: smart table not raised within %.0f sim s (state %s, platform %.3f m) - going on'
                % (a.lift_timeout, table.state(), top))

    def turn_round():
        if 'target' in result:      # container must still stand on the platform, at its height now
            check_on_table(result['target'], 'before moving away', z=table_top() + tbl.CONTAINER_HALF_H)
        base.turnaround(a.offset)

    def park():
        base.turnaround(a.offset, side=+1)
        base.goto(0.0, 0.0, 0.0)

    steps = [
        ('spawn container + release weld', lambda: log('container at %s' % (spawn_container(weld),))),
        ('arm home', lambda: check(node.arm(jp.INIT_POSE, d), 'arm home')),
        ('pick', pick),
        ('arm home (carrying)', lambda: check(node.arm(jp.INIT_POSE, d), 'arm home')),
        ('follow line to the patient', lambda: follow_line()),
        ('place on the bedside table', place),
        ('arm home', lambda: check(node.arm(jp.INIT_POSE, d), 'arm home')),
        ('table lifts medicine to the patient', table_lift),
        ('turn round', turn_round),
        ('follow line home', lambda: follow_line()),
        ('park at the start', park),
    ]

    code = 1
    out = {'label': a.label, 'success': False, 'failed_step': None, 'error': None,
           'steps': [], 'wall_start_unix': time.time()}
    t_wall0 = time.monotonic()          # durations: monotonic (immune to clock/time-zone changes)
    current = None
    try:
        node.wait_ready()
        t_start = base.pose()[3]
        out['sim_start'] = t_start
        if a.label:
            log(a.label)
        for i, (label, fn) in enumerate(steps, 1):
            if i < a.first or i > a.last:
                continue
            current = (i, label)
            x, y, yaw, t = base.pose()
            log('step %d/%d %s  (sim t %.1f, base %.3f %.3f %.1f deg)'
                % (i, len(steps), label, t, x, y, math.degrees(yaw)))
            out['steps'].append({'step': i, 'name': label, 'sim_t': t,
                                 'base_before': [x, y, math.degrees(yaw)]})
            if i == 6:
                out['patient_stop'] = [x, y, math.degrees(yaw)]
            fn()
            if i >= 6:      # where is the container after each step from placing on?
                c = gz_model_pose_full(CONTAINER_NAME)
                if c:
                    (cx, cy, cz), q = c
                    out['steps'][-1]['container_after'] = [cx, cy, cz, tilt_deg(q)]
                    out['steps'][-1]['container_after_pose'] = {'xyz': [cx, cy, cz], 'quat_xyzw': list(q)}
                    log('  container after step %d: (%.4f, %.4f, %.4f) tilt %.1f deg'
                        % (i, cx, cy, cz, tilt_deg(q)))
        x, y, yaw, t_end = base.pose()
        out.update(sim_time=t_end - t_start, base_final=[x, y, math.degrees(yaw)])
        log('done: base at %.4f %.4f yaw %.2f deg; cycle time %.1f sim s'
            % (x, y, math.degrees(yaw), t_end - t_start))
        out['success'] = True
        code = 0
    except KeyboardInterrupt:
        log('aborted')
        out['error'] = 'aborted'
    except RuntimeError as e:
        log('FAILED: %s' % e)
        out['error'] = str(e)
    finally:
        base.stop()
        if not out['success'] and current:
            out['failed_step'] = '%d %s' % current
        out['wall_time'] = time.monotonic() - t_wall0
        out['weld'] = WELD_LOG
        for k in ('placed', 'table'):
            if k in result:
                out[k] = result[k]
        # container result (Gazebo ground truth) - also after a failure, if it was placed
        if 'target' in result:
            tx, ty = result['target']
            out['target'] = [tx, ty]
            c = gz_model_pose_full(CONTAINER_NAME)
            if c is None:
                log('container not found in Gazebo')
            else:
                (cx, cy, cz), q = c
                out['container_quat_xyzw'] = list(q)       # raw pose for re-scoring
                err = 1000 * math.hypot(cx - tx, cy - ty)
                tilt = tilt_deg(q)
                top = table.height if table.height is not None else tbl.LOWERED_TOP
                on_table = abs(cz - (top + tbl.CONTAINER_HALF_H)) < 0.01
                out.update(container=[cx, cy, cz], placement_error_mm=err, tilt_deg=tilt,
                           on_table=on_table, platform_top_final=top)
                log('container at (%.4f, %.4f, %.4f); target (%.4f, %.4f); %.1f mm from the target; '
                    'tilt %.1f deg; standing on the platform (top %.3f m): %s'
                    % (cx, cy, cz, tx, ty, err, tilt, top, 'yes' if on_table else 'NO'))
        if a.result_json:
            with open(a.result_json, 'w') as fh:
                json.dump(out, fh, indent=2)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
