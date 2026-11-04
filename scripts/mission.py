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
  6 place at arm-relative (0, 0.20): above, descend, detach, open, retreat
  7 arm home
  8 turn round: strafe right, rotate 180 deg on /odom, strafe right (base_moves.py)
  9 follow the line home                             (line_follower.py)
 10 park at the start for the next mission: strafe LEFT (away from the medicine
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
import time

import rclpy

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import jetarm_pick_place as jp          # noqa: E402
from base_moves import BaseMover        # noqa: E402
from medicine_weld import Weld          # noqa: E402

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


def check_on_table(target, where):
    """Container must stand on the table within PLACE_TOL of the target."""
    c = gz_model_pose_full(CONTAINER_NAME)
    if c is None:
        raise RuntimeError('%s: container not found in Gazebo' % where)
    (cx, cy, cz), q = c
    d = math.hypot(cx - target[0], cy - target[1])
    log('%s: container (%.4f, %.4f, %.4f) tilt %.1f deg, %.1f mm from target'
        % (where, cx, cy, cz, tilt_deg(q), 1000 * d))
    if abs(cz - TABLE_Z) > 0.01 or d > PLACE_TOL or tilt_deg(q) > 10:
        raise RuntimeError('%s: container is not standing on the table at the target '
                           '(z %.3f, %.0f mm away, tilt %.0f deg)' % (where, cz, 1000 * d, tilt_deg(q)))


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
    p.add_argument('--from', dest='first', type=int, default=1, help='first step (1-9)')
    p.add_argument('--to', dest='last', type=int, default=10, help='last step (1-10)')
    p.add_argument('--result-json', default=None, help='write the results to this JSON file')
    p.add_argument('--label', default='', help='free text for the log, e.g. patient/medicine')
    p.add_argument('--offset', type=float, default=0.12, help='strafe distance for the turn (m)')
    p.add_argument('--duration', type=float, default=3.0, help='sim seconds per arm move')
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
    result = {}

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
        x, y, yaw, _ = base.pose()
        rx, ry = REACH_XY
        result['target'] = (x + math.cos(yaw) * rx - math.sin(yaw) * ry,
                            y + math.sin(yaw) * rx + math.cos(yaw) * ry)
        check(node.arm(plan['place_above'], d), 'move above table')
        check(node.arm(plan['place_descent'], d / 2), 'descend')
        confirm(weld, 'detach', 'place')
        check(node.gripper(pa.open, d), 'open gripper')
        check(node.arm(plan['place_retreat'], d / 2), 'retreat')
        check_on_table(result['target'], 'after retreat')

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
        ('turn round', lambda: (check_on_table(result['target'], 'before moving away')
                                if 'target' in result else None, base.turnaround(a.offset))),
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
                on_table = abs(cz - 0.13) < 0.01
                out.update(container=[cx, cy, cz], placement_error_mm=err, tilt_deg=tilt,
                           on_table=on_table)
                log('container at (%.4f, %.4f, %.4f); target (%.4f, %.4f); placement error '
                    '%.1f mm; tilt %.1f deg; standing on the table: %s'
                    % (cx, cy, cz, tx, ty, err, tilt, 'yes' if on_table else 'NO'))
        if a.result_json:
            with open(a.result_json, 'w') as fh:
                json.dump(out, fh, indent=2)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
