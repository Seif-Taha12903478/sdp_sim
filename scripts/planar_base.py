#!/usr/bin/env python3
"""Planar base driver for the simulated mecanum chassis.

The robot model (jetarm_mecanum.gazebo.xacro) is attached to the world through
three joints: base_x_joint (prismatic, world X), base_y_joint (prismatic, world Y)
and base_yaw_joint (continuous, Z). Gazebo JointController systems drive them
with velocity commands. This node:

  * subscribes /controller/cmd_vel (geometry_msgs/Twist, ROBOT frame:
    linear.x forward, linear.y left, angular.z yaw rate), like a mecanum base,
  * rotates it into world-frame joint velocities using the current yaw and
    publishes std_msgs/Float64 on /base/vel_x, /base/vel_y, /base/vel_yaw
    (bridged to Gazebo by hospital.launch.py),
  * publishes nav_msgs/Odometry on /odom (frame odom -> chassis_link) from the
    joint positions/velocities in /joint_states. Stamps are copied from
    /joint_states (sim time), so no /clock is needed.

The last command is held until a new one arrives (same as VelocityControl did).
The rotation is redone on every /joint_states message, so driving forward while
turning follows an arc correctly.

Started by hospital.launch.py:  python3 ~/sdp_sim/scripts/planar_base.py
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

JX, JY, JYAW = 'base_x_joint', 'base_y_joint', 'base_yaw_joint'


class PlanarBase(Node):
    def __init__(self):
        super().__init__('planar_base')
        self.cmd = Twist()
        self.yaw = None
        self.pub = {j: self.create_publisher(Float64, t, 10) for j, t in (
            (JX, '/base/vel_x'), (JY, '/base/vel_y'), (JYAW, '/base/vel_yaw'))}
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.create_subscription(Twist, '/controller/cmd_vel', self.on_cmd, 10)
        self.create_subscription(JointState, '/joint_states', self.on_joints, 20)
        self.get_logger().info('planar base ready: /controller/cmd_vel -> /base/vel_*, /odom')

    def on_cmd(self, msg):
        self.cmd = msg
        if self.yaw is not None:
            self.send()

    def send(self):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        vx, vy = self.cmd.linear.x, self.cmd.linear.y
        for j, v in ((JX, c * vx - s * vy), (JY, s * vx + c * vy), (JYAW, self.cmd.angular.z)):
            self.pub[j].publish(Float64(data=float(v)))

    def on_joints(self, msg):
        # /joint_states has two publishers (Gazebo JointStatePublisher with all joints,
        # ros2_control joint_state_broadcaster with arm/gripper only): use the first kind.
        try:
            idx = [msg.name.index(j) for j in (JX, JY, JYAW)]
        except ValueError:
            return
        x, y, yaw = (msg.position[i] for i in idx)
        vel = list(msg.velocity) if len(msg.velocity) == len(msg.name) else None
        self.yaw = yaw
        self.send()

        o = Odometry()
        o.header.stamp = msg.header.stamp
        o.header.frame_id = 'odom'
        o.child_frame_id = 'chassis_link'
        o.pose.pose.position.x = x
        o.pose.pose.position.y = y
        o.pose.pose.orientation.z = math.sin(yaw / 2)
        o.pose.pose.orientation.w = math.cos(yaw / 2)
        if vel is not None:
            xd, yd, yawd = (vel[i] for i in idx)
            c, s = math.cos(yaw), math.sin(yaw)
            o.twist.twist.linear.x = c * xd + s * yd      # robot frame
            o.twist.twist.linear.y = -s * xd + c * yd
            o.twist.twist.angular.z = yawd
        self.odom_pub.publish(o)


def main():
    rclpy.init()
    node = PlanarBase()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
