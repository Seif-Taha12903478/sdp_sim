#!/usr/bin/env python3
"""JetArm Pro pick-and-place for the Gazebo simulation (ROS 2 Humble).

Joint angles are computed from the real URDF numbers (link lengths, joint axes,
joint limits from jetarm_6dof_description), so no manual teaching is needed.
The tool always points straight down; only the grasp height and the pick /
place positions are yours to set.

Frames: x is straight ahead of the robot base, y is to its left, z is up
(metres, relative to the robot base on the ground).

Commands
  plan            print the joint angles for every waypoint (no ROS needed)
  state           print current joint positions
  home            move to Hiwonder's 'init_pose'
  grip VALUE      move gripper joint r_joint (1.4 = open, 0 = fully closed)
  goto J1..J5     move the arm to the given joint angles (radians)
  run             full pick-and-place sequence
  attach/detach   attach / release the container (assisted grasp, see below)

Note: Gazebo attaches the container to the gripper as soon as it is spawned.
'run' starting at step 1 releases it first; otherwise run 'detach' right after
spawning.

Assisted grasp (default): the fingers close around the container WITHOUT
squeezing it (--carry-grip), then Gazebo's DetachableJoint plugin welds the
container to the gripper while it is carried. Releasing removes the weld and
the container falls under normal physics. This avoids the very slow contact
between the claws and the container. Use --physical for the old squeeze grasp.

Options (before the command)
  --pick X Y        container position          (default 0.18 0.00)
  --place X Y       drop-off position           (default 0.12 0.14)
  --grasp-z Z       tool-point height at pick   (default 0.038)
  --release-z Z     tool-point height at drop   (default 0.045)
  --above-z Z       travel height               (default 0.14)
  --open / --closed gripper values              (default 1.4 / 0.52)
  --carry-grip V    gripper value for assisted grasp (default 0.75)
  --physical        squeeze grasp (no attach); uses --closed
  --duration S      simulated seconds per move  (default 3.0)
  --only A-B        run only steps A..B of 'run'. Assisted: 1 open, 2 above,
                    3 descend, 4 close, 5 attach, 6 lift, 7 above drop-off,
                    8 descend, 9 detach, 10 open, 11 retreat, 12 home.
                    (--physical has no attach/detach steps: 10 steps.)
"""
import argparse
import math
import subprocess
import sys
import time

try:
    import rclpy
    from builtin_interfaces.msg import Duration
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
except ImportError:          # lets `plan` run on a machine without ROS
    rclpy = None
    Node = object

# ----------------------------------------------------------- arm geometry
# From jetarm_6dof_description/urdf/{connect,arm}.urdf.xacro
BASE_Z = 0.05                       # joint1 height above base origin
H2 = 0.054048012164686              # joint1 -> joint2
L2 = 0.134416446394797              # joint2 -> joint3
L3 = 0.134444631186569              # joint3 -> joint4
D4 = 0.0544833339503674             # joint4 -> servo_link2 (wrist)
D_EE = 0.08                         # link5 -> end_effector_link
Z2 = BASE_Z + H2                    # joint2 pivot height
LT = D4 + D_EE                      # joint4 pivot -> tool point
JOINT_LIMIT = 2.09                  # +/- rad on joint1..joint5
MARGIN = 0.10                       # keep this far from the limits
INIT_POSE = [0.0, 0.7309, -1.6127, -1.6591, 0.0]     # SRDF 'init_pose'

ARM_TOPIC = '/arm_controller/joint_trajectory'
GRIPPER_TOPIC = '/gripper_controller/joint_trajectory'
JOINT_STATES_TOPIC = '/joint_states'
ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
GRIP_JOINTS = ['r_joint']


def fk(q):
    """Tool-point position and tool direction for q = [j1..j5] (all about
    axes (0,-1,0) for j2..j4)."""
    j1, j2, j3, j4, _ = q
    a = j2
    zx = [(H2, a)]
    # planar chain in the arm plane: angle from vertical toward +x is -(sum)
    ang = -j2
    r, z = 0.0, Z2
    r += L2 * math.sin(ang); z += L2 * math.cos(ang)
    ang = -(j2 + j3)
    r += L3 * math.sin(ang); z += L3 * math.cos(ang)
    ang = -(j2 + j3 + j4)
    r += LT * math.sin(ang); z += LT * math.cos(ang)
    return (r * math.cos(j1), r * math.sin(j1), z), ang


def ik_down(x, y, z, max_tilt_deg=60):
    """Joint angles that put the tool point at (x, y, z) with the tool
    pointing down (tilted outward as little as needed)."""
    j1 = math.atan2(y, x)
    r = math.hypot(x, y)
    for tilt in range(0, max_tilt_deg + 1, 5):
        t = math.radians(tilt)
        phit = math.pi - t                       # tool direction from vertical
        wr = r - LT * math.sin(phit)
        wz = z - LT * math.cos(phit)
        dx, dz = wr, wz - Z2
        d = math.hypot(dx, dz)
        if d > L2 + L3 - 1e-6 or d < abs(L2 - L3) + 1e-6:
            continue
        base = math.atan2(dx, dz)
        a = math.acos((d * d + L2 * L2 - L3 * L3) / (2 * d * L2))
        for sgn in (1, -1):
            pa = base + sgn * a
            v = (dx - L2 * math.sin(pa), dz - L2 * math.cos(pa))
            pb = math.atan2(v[0], v[1])
            wrap = lambda u: (u + math.pi) % (2 * math.pi) - math.pi
            q = [j1, wrap(-pa), wrap(-(pb - pa)), wrap(-(phit - pb)), 0.0]
            if all(abs(v_) <= JOINT_LIMIT - MARGIN for v_ in q):
                (ex, ey, ez), _ = fk(q)
                if abs(ex - x) + abs(ey - y) + abs(ez - z) < 1e-3:
                    return q, tilt
    raise ValueError('Position (%.3f, %.3f, %.3f) is out of reach' % (x, y, z))


def build_plan(a):
    """Named waypoints; descents/ascents are several points so the tool goes
    (almost) straight down/up."""
    def pose(xy, z):
        q, tilt = ik_down(xy[0], xy[1], z)
        return q

    def col(xy, z_start, z_end, n=4):
        zs = [z_start + (z_end - z_start) * i / (n - 1) for i in range(n)]
        return [pose(xy, z) for z in zs]

    plan = {
        'pick_above': pose(a.pick, a.above_z),
        'pick_descent': col(a.pick, a.above_z, a.grasp_z)[1:],
        'pick_lift': col(a.pick, a.grasp_z, a.above_z)[1:],
        'place_above': pose(a.place, a.above_z),
        'place_descent': col(a.place, a.above_z, a.release_z)[1:],
        'place_retreat': col(a.place, a.release_z, a.above_z)[1:],
        'home': INIT_POSE,
    }
    return plan


def cmd_plan(a):
    plan = build_plan(a)
    for k, v in plan.items():
        rows = v if isinstance(v[0], list) else [v]
        for i, q in enumerate(rows):
            pos, _ = fk(q)
            print('%-14s #%d  q=%s  tool point=(%.3f, %.3f, %.3f)' % (
                k, i, [round(u, 3) for u in q], *pos))
    print('gripper open=%.2f  closed=%.2f' % (a.open, a.closed))


# ----------------------------------------------------------------- ROS node
class PickPlace(Node):
    def __init__(self):
        super().__init__('jetarm_pick_place')
        self.arm_pub = self.create_publisher(JointTrajectory, ARM_TOPIC, 10)
        self.grip_pub = self.create_publisher(JointTrajectory, GRIPPER_TOPIC, 10)
        self.pos = {}
        self.sim_t = None       # sim time of the newest /joint_states message
        self.create_subscription(JointState, JOINT_STATES_TOPIC, self._on_js, 10)

    def _on_js(self, msg):
        for n, p in zip(msg.name, msg.position):
            self.pos[n] = p
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.sim_t is None or t > self.sim_t:
            self.sim_t = t

    def get(self, names):
        return [self.pos[n] for n in names]

    def wait_ready(self, timeout=30.0):
        end = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if (all(n in self.pos for n in ARM_JOINTS + GRIP_JOINTS) and
                    self.arm_pub.get_subscription_count() > 0 and
                    self.grip_pub.get_subscription_count() > 0):
                return
        missing = [n for n in ARM_JOINTS + GRIP_JOINTS if n not in self.pos]
        raise RuntimeError(
            'Not ready: no %s data for %s; arm topic subscribers=%d, gripper topic '
            'subscribers=%d. Is the simulation running?'
            % (JOINT_STATES_TOPIC, missing or 'none missing',
               self.arm_pub.get_subscription_count(),
               self.grip_pub.get_subscription_count()))

    def _traj(self, names, points, seconds_each):
        msg = JointTrajectory()
        msg.joint_names = names
        for i, p in enumerate(points, 1):
            pt = JointTrajectoryPoint()
            pt.positions = [float(v) for v in p]
            t = seconds_each * i
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            msg.points.append(pt)
        return msg

    def _wait(self, names, target, tol, timeout, settle, min_sim=0.0):
        """Wait until within tol of target ('reached') or, with settle=True, until the
        joints stop moving ('settled', e.g. fingers stopped by the container).
        Settling is judged on SIM time from /joint_states stamps: only after min_sim
        sim seconds (the trajectory duration), and when the joints moved < 0.003 rad
        over the last 0.5 sim s. (It used to be judged on wall-clock time, which at a
        low real-time factor is a tiny sim interval: at ~4 %% RTF the start of the
        closing spline looked 'settled' and the gripper was still open.)
        timeout is a wall-clock safety limit only."""
        start = time.monotonic()
        while self.sim_t is None and time.monotonic() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        sim0 = self.sim_t
        hist = []
        err = 9.9
        last_dot = start
        while rclpy.ok() and time.monotonic() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            pos = self.get(names)
            err = max(abs(p - t) for p, t in zip(pos, target))
            if time.monotonic() - last_dot >= 30:
                last_dot = time.monotonic()
                print('       ... still moving, %.3f rad to go' % err, flush=True)
            if err < tol:
                return 'reached'
            if settle and self.sim_t is not None:
                now = self.sim_t
                if not hist or now > hist[-1][0]:
                    hist.append((now, list(pos)))
                hist = [h for h in hist if now - h[0] <= 0.6]
                if now - sim0 >= min_sim and now - hist[0][0] >= 0.5:
                    if max(abs(x - y) for x, y in zip(hist[0][1], hist[-1][1])) < 0.003:
                        return 'settled'
        self.get_logger().warn('Timed out waiting for %s (error %.3f rad)' % (names, err))
        return None

    def arm(self, points, seconds_each, tol=0.03, timeout=3600.0):
        if not isinstance(points[0], (list, tuple)):
            points = [points]
        self.arm_pub.publish(self._traj(ARM_JOINTS, points, seconds_each))
        return self._wait(ARM_JOINTS, points[-1], tol, timeout, False)

    def gripper(self, value, seconds, tol=0.03, timeout=1800.0):
        self.grip_pub.publish(self._traj(GRIP_JOINTS, [[value]], seconds))
        return self._wait(GRIP_JOINTS, [value], tol, timeout, True, min_sim=seconds + 0.2)


ATTACH_TOPIC = '/medicine/attach'
DETACH_TOPIC = '/medicine/detach'


def ign_publish(topic):
    """Publish an empty message on a Gazebo transport topic."""
    cmd = ['ign', 'topic', '-t', topic, '-m', 'ignition.msgs.Empty', '-p', 'unused: true']
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
    except (OSError, subprocess.TimeoutExpired) as e:
        print('       could not run "ign topic": %s' % e)
        return False
    if r.returncode != 0:
        print('       ign topic failed: %s' % (r.stderr.strip() or r.stdout.strip()))
        return False
    return True


def do_attach_detach(topic, label):
    if not ign_publish(topic):
        return None
    time.sleep(6)           # let the simulation process the message
    return label


def cmd_run(node, a):
    plan = build_plan(a)
    d = a.duration
    close_to = a.closed if a.physical else a.carry_grip
    steps = [
        ('open gripper',         lambda: node.gripper(a.open, d), None),
        ('move above container', lambda: node.arm(plan['pick_above'], d), None),
        ('descend',              lambda: node.arm(plan['pick_descent'], d / 2), None),
        ('close gripper',        lambda: node.gripper(close_to, d), 'grasp'),
    ]
    if not a.physical:
        steps.append(('attach container',
                      lambda: do_attach_detach(ATTACH_TOPIC, 'attached'), None))
    steps += [
        ('lift',                 lambda: node.arm(plan['pick_lift'], d / 2), None),
        ('move above drop-off',  lambda: node.arm(plan['place_above'], d), None),
        ('descend to drop-off',  lambda: node.arm(plan['place_descent'], d / 2), None),
    ]
    if not a.physical:
        steps.append(('release container',
                      lambda: do_attach_detach(DETACH_TOPIC, 'released'), None))
    steps += [
        ('open gripper (drop)',  lambda: node.gripper(a.open, d), None),
        ('retreat up',           lambda: node.arm(plan['place_retreat'], d / 2), None),
        ('return home',          lambda: node.arm(plan['home'], d), None),
    ]
    first, last = 1, len(steps)
    if a.only:
        first, last = [int(v) for v in a.only.split('-')]
    input('Steps %d-%d of %d; the simulation may be slow. Enter to start, Ctrl+C to stop... '
          % (first, last, len(steps)))
    if not a.physical and first == 1:
        # Gazebo's DetachableJoint welds the container as soon as it is spawned;
        # release it before we start so it stays on the ground.
        print('[0] releasing container (Gazebo attaches it automatically at spawn) ...')
        print('       ->', do_attach_detach(DETACH_TOPIC, 'released') or 'failed')
    for i, (label, action, check) in enumerate(steps, 1):
        if i < first or i > last:
            continue
        print('[%d/%d] %s ...' % (i, len(steps), label), flush=True)
        res = action()
        print('       ->', res)
        if res is None:
            sys.exit('Step failed or timed out - stopping.')
        if check == 'grasp':
            g = node.get(GRIP_JOINTS)[0]
            print('       gripper at %.3f rad' % g)
            if g < 0.35:
                sys.exit('Gripper closed almost fully: nothing is between the fingers. '
                         'Container missed - check --pick, --grasp-z and the spawn position.')
        time.sleep(0.5)
    print('Done.')


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--pick', type=float, nargs=2, default=[0.18, 0.0])
    p.add_argument('--place', type=float, nargs=2, default=[0.12, 0.14])
    p.add_argument('--grasp-z', type=float, default=0.038)
    p.add_argument('--release-z', type=float, default=0.045)
    p.add_argument('--above-z', type=float, default=0.14)
    p.add_argument('--open', type=float, default=1.4)
    p.add_argument('--closed', type=float, default=0.52)
    p.add_argument('--carry-grip', type=float, default=0.75)
    p.add_argument('--physical', action='store_true')
    p.add_argument('--duration', type=float, default=3.0)
    p.add_argument('--only', default=None,
                   help="run only these steps of 'run', e.g. 1-4 or 5-5")
    sub = p.add_subparsers(dest='cmd', required=True)
    for n in ('plan', 'state', 'home', 'run', 'attach', 'detach'):
        sub.add_parser(n)
    g = sub.add_parser('grip'); g.add_argument('value', type=float)
    t = sub.add_parser('goto'); t.add_argument('values', type=float, nargs=5)
    a = p.parse_args()

    if a.cmd == 'plan':
        try:
            cmd_plan(a)
        except ValueError as e:
            sys.exit(str(e))
        return
    if a.cmd in ('attach', 'detach'):
        topic = ATTACH_TOPIC if a.cmd == 'attach' else DETACH_TOPIC
        print(do_attach_detach(topic, a.cmd + ' sent') or 'failed')
        return
    if rclpy is None:
        sys.exit('rclpy not found - source ROS 2 first (source ~/.bashrc).')
    try:
        build_plan(a)
    except ValueError as e:
        sys.exit(str(e))

    rclpy.init()
    node = PickPlace()
    try:
        node.wait_ready()
        if a.cmd == 'state':
            print('arm    :', [round(v, 4) for v in node.get(ARM_JOINTS)])
            print('gripper:', round(node.get(GRIP_JOINTS)[0], 4))
        elif a.cmd == 'home':
            print(node.arm(INIT_POSE, a.duration))
        elif a.cmd == 'grip':
            print(node.gripper(a.value, a.duration))
        elif a.cmd == 'goto':
            print(node.arm(a.values, a.duration))
        elif a.cmd == 'run':
            cmd_run(node, a)
    except KeyboardInterrupt:
        print('\nAborted.')
    except RuntimeError as e:
        sys.exit(str(e))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
