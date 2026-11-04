#!/usr/bin/env python3
"""Closed-loop base moves for the planar (mecanum-like) base, using /odom.

    python3 base_moves.py strafe 0.12        # + = robot left, - = robot right (m)
    python3 base_moves.py forward 0.05       # + = forward (m)
    python3 base_moves.py rotate 180         # + = counter-clockwise (deg), relative
    python3 base_moves.py turnaround [0.12]  # strafe right, rotate 180, strafe right

turnaround is the patient-stop manoeuvre: the bedside table is on the robot's LEFT,
so it strafes RIGHT away from it (turning in place sweeps a 0.27 m circle), turns
180 deg, and then the line is on the robot's right again: strafe right by the same
distance to get back onto it.

All moves wait on /odom (sim time) - never on wall-clock time - and stop with a zero
command. Import BaseMover for use in other nodes (mission.py).
"""
import math
import sys

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class BaseMover:
    """Uses an existing rclpy node; spins it while moving."""

    def __init__(self, node: Node):
        self.node = node
        self.pub = node.create_publisher(Twist, '/controller/cmd_vel', 10)
        self.odom = None
        node.create_subscription(Odometry, '/odom', self._on_odom, 20)

    def _on_odom(self, m):
        self.odom = m

    def pose(self):
        """(x, y, yaw, sim_t) of chassis_link; waits for the first /odom."""
        while self.odom is None:
            rclpy.spin_once(self.node, timeout_sec=0.5)
        m = self.odom
        p = m.pose.pose.position
        return p.x, p.y, yaw_of(m.pose.pose.orientation), m.header.stamp.sec + m.header.stamp.nanosec * 1e-9

    def send(self, vx=0.0, vy=0.0, wz=0.0):
        t = Twist()
        t.linear.x, t.linear.y, t.angular.z = float(vx), float(vy), float(wz)
        self.pub.publish(t)

    def stop(self):
        for _ in range(3):
            self.send()
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _next(self, t_prev):
        while True:
            rclpy.spin_once(self.node, timeout_sec=0.5)
            x, y, yaw, t = self.pose()
            if t > t_prev:
                return x, y, yaw, t

    def translate(self, dx_robot, dy_robot, vmax=0.05, tol=0.002, timeout_sim=60.0):
        """Move by (dx, dy) in the robot frame at the start pose, keeping the heading."""
        x0, y0, yaw0, t0 = self.pose()
        c, s = math.cos(yaw0), math.sin(yaw0)
        gx, gy = x0 + c * dx_robot - s * dy_robot, y0 + s * dx_robot + c * dy_robot
        x, y, yaw, t = x0, y0, yaw0, t0
        while True:
            ex_w, ey_w = gx - x, gy - y
            if math.hypot(ex_w, ey_w) < tol:
                break
            if t - t0 > timeout_sim:
                self.stop()
                raise RuntimeError('translate timed out (%.3f m left)' % math.hypot(ex_w, ey_w))
            # error in the CURRENT robot frame
            cy, sy = math.cos(yaw), math.sin(yaw)
            ex, ey = cy * ex_w + sy * ey_w, -sy * ex_w + cy * ey_w
            k = 1.5
            vx, vy = k * ex, k * ey
            n = math.hypot(vx, vy)
            if n > vmax:
                vx, vy = vx * vmax / n, vy * vmax / n
            self.send(vx, vy, 2.0 * wrap(yaw0 - yaw))   # hold heading
            x, y, yaw, t = self._next(t)
        self.stop()
        x, y, yaw, _ = self.pose()
        return x, y, yaw

    def rotate(self, dyaw, wmax=0.4, tol=math.radians(0.3), timeout_sim=60.0):
        """Rotate by dyaw (rad, + = CCW) about chassis_link (= under arm joint1)."""
        x0, y0, yaw0, t0 = self.pose()
        goal = yaw0 + dyaw
        done = 0.0
        yaw_prev = yaw0
        x, y, yaw, t = x0, y0, yaw0, t0
        while True:
            done += wrap(yaw - yaw_prev)      # unwrapped progress
            yaw_prev = yaw
            err = dyaw - done
            if abs(err) < tol:
                break
            if t - t0 > timeout_sim:
                self.stop()
                raise RuntimeError('rotate timed out (%.1f deg left)' % math.degrees(err))
            w = max(-wmax, min(wmax, 2.0 * err))
            if abs(w) < 0.02:
                w = math.copysign(0.02, err)
            self.send(0.0, 0.0, w)
            x, y, yaw, t = self._next(t)
        self.stop()
        x, y, yaw, _ = self.pose()
        return x, y, yaw

    def turnaround(self, offset=0.12):
        self.translate(0.0, -offset)
        self.rotate(math.pi)
        return self.translate(0.0, -offset)


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    cmd = sys.argv[1]
    rclpy.init()
    node = rclpy.create_node('base_moves')
    m = BaseMover(node)
    x, y, yaw, t = m.pose()
    print('start: x %.4f  y %.4f  yaw %.2f deg  (sim t %.1f)' % (x, y, math.degrees(yaw), t))
    try:
        if cmd == 'strafe':
            r = m.translate(0.0, float(sys.argv[2]))
        elif cmd == 'forward':
            r = m.translate(float(sys.argv[2]), 0.0)
        elif cmd == 'rotate':
            r = m.rotate(math.radians(float(sys.argv[2])))
        elif cmd == 'turnaround':
            r = m.turnaround(float(sys.argv[2]) if len(sys.argv) > 2 else 0.12)
        else:
            print('unknown command', cmd)
            sys.exit(1)
        print('end:   x %.4f  y %.4f  yaw %.2f deg' % (r[0], r[1], math.degrees(r[2])))
    except KeyboardInterrupt:
        m.stop()
        print('aborted')
        sys.exit(1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
