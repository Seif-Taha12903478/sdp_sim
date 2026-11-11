#!/usr/bin/env python3
"""Offline test of PickPlace._wait (scripts/jetarm_pick_place.py), no Gazebo:

    python3 ~/sdp_sim/tools/test_arm_wait.py

A fake /joint_states publisher (own ROS domain 77, sim time 4x real time) drives the arm:
reached, arm home settled within 0.06 rad, a non-home move that stops 0.045 rad short (fails on
SIM time), arm home stuck 0.10 rad short (fails), a FROZEN joint-state cache (the subscription
silently stops delivering, as in demo attempt 1) recovered by check_cache, the gripper settling
on the container, and no messages at all (wall-clock backstop).
"""
import os
import sys
import threading
import time
import unittest

os.environ['ROS_DOMAIN_ID'] = '77'          # isolated from any simulation
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), 'scripts'))
import rclpy                                 # noqa: E402
from sensor_msgs.msg import JointState       # noqa: E402
import jetarm_pick_place as jp               # noqa: E402

HOME = jp.INIT_POSE
OTHER = [1.5708, 0.3, -1.2, -1.4, 0.0]


class FakeJoints:
    """Publishes arm + gripper joint states; profile(sim_t) -> (arm list, gripper)."""

    def __init__(self, profile, rate_wall=200.0, sim_dt=0.02):
        self.node = rclpy.create_node('fake_joint_states')
        self.pub = self.node.create_publisher(JointState, jp.JOINT_STATES_TOPIC, 10)
        self.profile, self.dt_wall, self.sim_dt = profile, 1.0 / rate_wall, sim_dt
        self.t, self.run = 100.0, True
        self.th = threading.Thread(target=self._loop, daemon=True)
        self.th.start()

    def _loop(self):
        while self.run:
            arm, g = self.profile(self.t)
            m = JointState()
            m.header.stamp.sec, m.header.stamp.nanosec = int(self.t), int((self.t % 1) * 1e9)
            m.name = list(jp.ARM_JOINTS) + list(jp.GRIP_JOINTS)
            m.position = [float(v) for v in arm] + [float(g)]
            self.pub.publish(m)
            self.t += self.sim_dt
            time.sleep(self.dt_wall)

    def stop(self):
        self.run = False
        self.th.join()
        self.node.destroy_node()


def approach(start, goal, t0, tau=0.5, stop_short=None):
    """Exponential approach from start to goal from sim t0; stop_short = {joint index: offset}."""
    import math

    def f(t):
        k = math.exp(-max(0.0, t - t0) / tau)
        q = [g + (s - g) * k for s, g in zip(start, goal)]
        for i, off in (stop_short or {}).items():
            q[i] = max(q[i], goal[i] + off) if start[i] > goal[i] else min(q[i], goal[i] - off) \
                if start[i] < goal[i] else goal[i] + off
        return q
    return f


class ArmWait(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = jp.PickPlace()
        self.node.STALE_WALL = 1.0
        self.node.PROBE_TIMEOUT = 2.0
        self.fake = None

    def tearDown(self):
        if self.fake:
            self.fake.stop()
        self.node.destroy_node()

    def start(self, arm_profile, grip=lambda t: 1.4):
        self.fake = FakeJoints(lambda t: (arm_profile(t), grip(t)))
        end = time.monotonic() + 10
        while self.node.sim_t is None and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertIsNotNone(self.node.sim_t, 'fake joint states not received')
        return self.node.sim_t

    def test_reached(self):
        t0 = self.start(lambda t: OTHER)
        self.fake.profile = (lambda f: (lambda t: (f(t), 1.4)))(approach(OTHER, HOME, t0))
        self.assertEqual(self.node.arm(HOME, 1.0, sim_margin=10.0), 'reached')

    def test_home_settled_within_006(self):
        t0 = self.start(lambda t: OTHER)
        self.fake.profile = (lambda f: (lambda t: (f(t), 1.4)))(approach(OTHER, HOME, t0, stop_short={1: 0.045}))
        self.assertEqual(self.node.arm(HOME, 1.0, sim_margin=10.0), 'settled')

    def test_non_home_needs_tol(self):
        t0 = self.start(lambda t: HOME)
        self.fake.profile = (lambda f: (lambda t: (f(t), 1.4)))(approach(HOME, OTHER, t0, stop_short={1: 0.045}))
        w0, s0 = time.monotonic(), self.node.sim_t
        self.assertIsNone(self.node.arm(OTHER, 1.0, sim_margin=3.0))
        self.assertGreaterEqual(self.node.sim_t - s0, 4.0)          # failed on SIM time ...
        self.assertLess(time.monotonic() - w0, 30.0)                # ... long before any wall limit

    def test_home_stuck_010_fails(self):
        t0 = self.start(lambda t: OTHER)
        self.fake.profile = (lambda f: (lambda t: (f(t), 1.4)))(approach(OTHER, HOME, t0, stop_short={1: 0.10}))
        self.assertIsNone(self.node.arm(HOME, 1.0, sim_margin=3.0))

    def test_frozen_cache_recovered(self):
        t0 = self.start(lambda t: OTHER)
        self.fake.profile = (lambda f: (lambda t: (f(t), 1.4)))(approach(OTHER, HOME, t0 + 0.5))
        rclpy.spin_once(self.node, timeout_sec=0.1)
        # freeze: the subscription silently stops delivering (cache keeps OTHER = far from home)
        self.node.destroy_subscription(self.node._js_sub)
        self.node._js_sub = self.node.create_subscription(JointState, '/not_joint_states', self.node._on_js, 10)
        self.assertEqual(self.node.arm(HOME, 1.0, sim_margin=30.0), 'reached')
        self.assertGreaterEqual(self.node.cache_recoveries, 1)
        self.assertEqual(self.node._js_sub.topic_name, jp.JOINT_STATES_TOPIC)   # subscription re-created

    def test_gripper_settles_on_container(self):
        t0 = self.start(lambda t: HOME, grip=lambda t: 1.4)
        f = approach([1.4], [0.52], t0, stop_short={0: 0.0})
        self.fake.profile = lambda t: (HOME, max(0.75, f(t)[0]))
        self.assertEqual(self.node.gripper(0.52, 1.0, sim_margin=10.0), 'settled')

    def test_no_messages_wall_backstop(self):
        self.start(lambda t: OTHER)
        self.fake.stop()
        self.fake = None
        w0 = time.monotonic()
        self.assertIsNone(self.node.arm(HOME, 1.0, timeout=8.0))
        self.assertLess(time.monotonic() - w0, 20.0)


if __name__ == '__main__':
    unittest.main(verbosity=2)
