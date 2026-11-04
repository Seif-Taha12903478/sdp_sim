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

Reuses jetarm_pick_place.py (IK, arm/gripper control, attach/detach) and
base_moves.py. Every wait is on feedback (/joint_states, /odom, camera stamps).
At the end it prints the container's placement error (Gazebo ground truth vs. the
intended spot on the bedside table) and the cycle time in sim seconds.
Exit code 0 = mission complete.
"""
import argparse
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

CONTAINER_SDF = os.path.join(HERE, 'medicine_container.sdf')
CONTAINER_NAME = 'medicine'              # DetachableJoint child_model
STATION_XYZ = (0.0, 0.20, 0.131)          # container spawn (world), table top 0.10
REACH_XY = (0.0, 0.20)                    # arm-relative pick and place (left of joint1)
WORLD = 'robot_world'


def log(msg):
    print('[mission] ' + msg, flush=True)


def gz_model_pose(name):
    """Gazebo ground truth (x, y, z) of a model, or None."""
    t = subprocess.run(['ign', 'topic', '-e', '-n', '1', '-t', '/world/%s/pose/info' % WORLD],
                       capture_output=True, text=True, timeout=120).stdout
    m = re.search(r'name: "%s"\s*id: \d+\s*position \{([^}]*)\}' % re.escape(name), t)
    if not m:
        return None
    f = lambda k: float(re.search(k + r': ([-\d.e]+)', m.group(1)).group(1)) \
        if re.search(k + r': ([-\d.e]+)', m.group(1)) else 0.0
    return f('x'), f('y'), f('z')


def spawn_container():
    if gz_model_pose(CONTAINER_NAME) is not None:
        log('removing the old container')
        subprocess.run(['ign', 'service', '-s', '/world/%s/remove' % WORLD,
                        '--reqtype', 'ignition.msgs.Entity', '--reptype', 'ignition.msgs.Boolean',
                        '--timeout', '60000', '--req', 'name: "%s" type: MODEL' % CONTAINER_NAME],
                       capture_output=True, text=True, timeout=120)
    x, y, z = STATION_XYZ
    r = subprocess.run(['ros2', 'run', 'ros_ign_gazebo', 'create', '-file', CONTAINER_SDF,
                        '-name', CONTAINER_NAME, '-x', str(x), '-y', str(y), '-z', str(z)],
                       capture_output=True, text=True, timeout=300)
    if r.returncode != 0:
        raise RuntimeError('spawn failed: ' + (r.stderr.strip() or r.stdout.strip()))
    # the DetachableJoint welds it to the gripper as soon as it exists: release it
    if jp.do_attach_detach(jp.DETACH_TOPIC, 'released') is None:
        raise RuntimeError('could not release the container after spawning')
    return gz_model_pose(CONTAINER_NAME)


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
    p.add_argument('--to', dest='last', type=int, default=9, help='last step (1-9)')
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
        check(jp.do_attach_detach(jp.ATTACH_TOPIC, 'attached'), 'attach')
        check(node.arm(plan['pick_lift'], d / 2), 'lift')

    def place():
        x, y, yaw, _ = base.pose()
        rx, ry = REACH_XY
        result['target'] = (x + math.cos(yaw) * rx - math.sin(yaw) * ry,
                            y + math.sin(yaw) * rx + math.cos(yaw) * ry)
        check(node.arm(plan['place_above'], d), 'move above table')
        check(node.arm(plan['place_descent'], d / 2), 'descend')
        check(jp.do_attach_detach(jp.DETACH_TOPIC, 'released'), 'detach')
        check(node.gripper(pa.open, d), 'open gripper')
        check(node.arm(plan['place_retreat'], d / 2), 'retreat')

    steps = [
        ('spawn container + release weld', lambda: log('container at %s' % (spawn_container(),))),
        ('arm home', lambda: check(node.arm(jp.INIT_POSE, d), 'arm home')),
        ('pick', pick),
        ('arm home (carrying)', lambda: check(node.arm(jp.INIT_POSE, d), 'arm home')),
        ('follow line to the patient', lambda: follow_line()),
        ('place on the bedside table', place),
        ('arm home', lambda: check(node.arm(jp.INIT_POSE, d), 'arm home')),
        ('turn round', lambda: base.turnaround(a.offset)),
        ('follow line home', lambda: follow_line()),
    ]

    code = 1
    try:
        node.wait_ready()
        t_start = base.pose()[3]
        for i, (label, fn) in enumerate(steps, 1):
            if i < a.first or i > a.last:
                continue
            x, y, yaw, t = base.pose()
            log('step %d/%d %s  (sim t %.1f, base %.3f %.3f %.1f deg)'
                % (i, len(steps), label, t, x, y, math.degrees(yaw)))
            fn()
        x, y, yaw, t_end = base.pose()
        log('done: base at %.4f %.4f yaw %.2f deg; cycle time %.1f sim s'
            % (x, y, math.degrees(yaw), t_end - t_start))
        if 'target' in result:
            c = gz_model_pose(CONTAINER_NAME)
            tx, ty = result['target']
            if c is None:
                log('container not found in Gazebo')
            else:
                log('container at (%.4f, %.4f, %.4f); target (%.4f, %.4f); '
                    'placement error %.1f mm; standing on the table: %s'
                    % (c[0], c[1], c[2], tx, ty, 1000 * math.hypot(c[0] - tx, c[1] - ty),
                       'yes' if abs(c[2] - 0.13) < 0.01 else 'NO (z %.3f)' % c[2]))
        code = 0
    except KeyboardInterrupt:
        log('aborted')
    except RuntimeError as e:
        log('FAILED: %s' % e)
    finally:
        base.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
