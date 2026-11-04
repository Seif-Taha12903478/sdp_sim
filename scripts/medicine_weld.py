#!/usr/bin/env python3
"""Confirmed attach / detach of the medicine container (Gazebo DetachableJoint).

The old way (jetarm_pick_place.do_attach_detach) ran `ign topic -p` once and then
slept 6 s wall-clock: a one-shot publisher can send before its connection to the
plugin is up, so a command could be lost or late, and nothing checked it.

Here the commands go through ROS with one persistent publisher, bridged by
hospital.launch.py:
    /medicine/attach, /medicine/detach   std_msgs/Empty  -> ignition.msgs.Empty
    /medicine/attached                   ignition.msgs.StringMsg -> std_msgs/String
The plugin publishes "attached" / "detached" on /medicine/attached when the state
CHANGES (and logs "Already attached/detached" for a repeated command, which is
harmless). We subscribe first, send, and wait for the expected state on sim time
(from /joint_states stamps); if it does not come, resend, up to `attempts` times.
Pose checks in mission.py confirm the result physically (container follows the
gripper after attach; stays on the table after detach + retreat).

CLI (needs a running simulation):  python3 medicine_weld.py attach|detach
"""
import sys

import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, String


class Weld:
    def __init__(self, node):
        self.node = node
        self.pub = {'attach': node.create_publisher(Empty, '/medicine/attach', 10),
                    'detach': node.create_publisher(Empty, '/medicine/detach', 10)}
        self.states = []            # (sim_t, 'attached'|'detached') as received
        self.sim_t = None
        node.create_subscription(String, '/medicine/attached', self._on_state, 10)
        node.create_subscription(JointState, '/joint_states', self._on_js, 1)

    def _on_state(self, m):
        self.states.append((self.sim_t, m.data))
        self.node.get_logger().info('DetachableJoint state: %s' % m.data)

    def _on_js(self, m):
        self.sim_t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9

    def _spin_until(self, cond, sim_seconds):
        while self.sim_t is None:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        t_end = self.sim_t + sim_seconds
        while self.sim_t < t_end:
            if cond():
                return True
            rclpy.spin_once(self.node, timeout_sec=0.2)
        return cond()

    def wait_connected(self, sim_seconds=10.0):
        """Wait until the bridge subscribes to both command topics."""
        return self._spin_until(lambda: all(p.get_subscription_count() > 0 for p in self.pub.values()),
                                sim_seconds)

    def command(self, kind, attempts=5, sim_timeout=2.0):
        """kind 'attach' | 'detach'. Returns (confirmed, attempts_used)."""
        want = 'attached' if kind == 'attach' else 'detached'
        if not self.wait_connected():
            raise RuntimeError('/medicine/%s has no subscriber - is the bridge running?' % kind)
        for i in range(1, attempts + 1):
            n0 = len(self.states)
            self.pub[kind].publish(Empty())
            if self._spin_until(lambda: any(s == want for _, s in self.states[n0:]), sim_timeout):
                return True, i
            self.node.get_logger().warn('%s not confirmed within %.1f sim s (attempt %d/%d) - resending'
                                        % (kind, sim_timeout, i, attempts))
        return False, attempts

    def wait_state(self, want, since=0, sim_seconds=3.0):
        """True if state `want` was received (index >= since) within sim_seconds."""
        return self._spin_until(lambda: any(st == want for _, st in self.states[since:]), sim_seconds)

    def last_state(self):
        return self.states[-1][1] if self.states else None


def main():
    if len(sys.argv) != 2 or sys.argv[1] not in ('attach', 'detach'):
        print(__doc__)
        sys.exit(1)
    rclpy.init()
    node = rclpy.create_node('medicine_weld')
    w = Weld(node)
    ok, n = w.command(sys.argv[1])
    print('%s: %s after %d attempt(s); last state %s' % (sys.argv[1], 'CONFIRMED' if ok else 'NOT confirmed',
                                                        n, w.last_state()))
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 2)


if __name__ == '__main__':
    main()
