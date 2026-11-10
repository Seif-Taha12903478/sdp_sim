#!/usr/bin/env python3
"""Blue-line follower for the simulated JetArm mecanum robot (ROS 2 Humble).

Port of Hiwonder's line_following node ("JetArm and Mecanum Chassis
Integration Course", section 6) to the simulation:
  * same camera (arm wrist camera, arm in its ready pose looking down),
  * same three image strips near the bottom of the image with weights
    0.7 / 0.2 / 0.1 (Hiwonder's GEMINI ROIs),
  * deflection angle = -atan((cx - w/2) / (h/2)), PD steering,
  * constant forward speed 0.08 m/s (Hiwonder uses 80 mm/s).
Differences: colour is a fixed HSV range instead of click-to-pick, the
weighted centre is normalised by the strips actually found (Hiwonder divides
by the full weight sum), commands are geometry_msgs/Twist on
/controller/cmd_vel, and the robot stops on a red floor marker.

Usage:
  python3 line_follower.py             # follow until the red marker, then stop
  python3 line_follower.py --show      # also open a camera window
Exit code 0 = arrived at marker, 2 = line lost, 1 = error / aborted.
A debug image is written to /tmp/line_follower_debug.jpg about once a second.
"""
import argparse
import math
import sys
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image

IMAGE_TOPIC = '/depth_cam/depth_cam'
CMD_TOPIC = '/controller/cmd_vel'
# (y_start, y_end, x_start, x_end, weight) as fractions of the image (Hiwonder GEMINI)
ROIS = ((0.90, 0.95, 0.0, 1.0, 0.7), (0.80, 0.85, 0.0, 1.0, 0.2), (0.70, 0.75, 0.0, 1.0, 0.1))
BLUE_HSV = ((100, 120, 60), (130, 255, 255))
RED_HSV = (((0, 120, 60), (10, 255, 255)), ((170, 120, 60), (180, 255, 255)))


def image_to_bgr(msg):
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    if msg.encoding in ('rgb8', 'bgr8'):
        img = buf.reshape(msg.height, msg.step)[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if msg.encoding == 'rgb8' else img.copy()
    raise ValueError('unsupported image encoding %s' % msg.encoding)


class LineFollower(Node):
    def __init__(self, a):
        super().__init__('line_follower')
        self.a = a
        self.pub = self.create_publisher(Twist, CMD_TOPIC, 10)
        self.create_subscription(Image, IMAGE_TOPIC, self.on_image, 5)
        self.result = None          # 'arrived' | 'lost'
        self.prev_t = None
        self.prev_err = 0.0
        self.lost_since = None
        self.red_frames = 0
        self.frames = 0
        self.t_start = None
        self.last_debug = 0.0

    @staticmethod
    def stamp(msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def analyse(self, bgr):
        h, w = bgr.shape[:2]
        hsv = cv2.cvtColor(cv2.GaussianBlur(bgr, (3, 3), 0), cv2.COLOR_BGR2HSV)
        kernel = np.ones((3, 3), np.uint8)
        centre_sum, weight_found, red_px, band_px = 0.0, 0.0, 0, 0
        for (y0, y1, x0, x1, wt) in ROIS:
            ys, ye, xs, xe = int(y0 * h), int(y1 * h), int(x0 * w), int(x1 * w)
            roi = hsv[ys:ye, xs:xe]
            band_px += roi.shape[0] * roi.shape[1]
            red = sum(cv2.inRange(roi, np.array(lo), np.array(hi)) for lo, hi in RED_HSV)
            red_px += int(np.count_nonzero(red))
            mask = cv2.inRange(roi, np.array(BLUE_HSV[0]), np.array(BLUE_HSV[1]))
            mask = cv2.dilate(cv2.erode(mask, kernel), kernel)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contours = [c for c in contours if cv2.contourArea(c) > self.a.min_area]
            if contours:
                c = max(contours, key=cv2.contourArea)
                (cx, cy), _, _ = cv2.minAreaRect(c)
                cx += xs
                centre_sum += cx * wt
                weight_found += wt
                cv2.circle(bgr, (int(cx), int(cy + ys)), 4, (0, 255, 255), -1)
            cv2.rectangle(bgr, (xs, ys), (xe - 1, ye - 1), (0, 255, 0), 1)
        red_frac = red_px / max(band_px, 1)
        if weight_found == 0:
            return None, red_frac
        centre = centre_sum / weight_found
        cv2.line(bgr, (int(centre), 0), (int(centre), h - 1), (255, 0, 255), 1)
        return -math.atan((centre - w / 2.0) / (h / 2.0)), red_frac

    def steer(self, angle, t, speed=None):
        """PD steering on the deflection angle at sim time t -> (vx, wz). speed: forward
        speed on a straight line (default --speed); slower when the line is at an angle.
        (Moved out of on_image unchanged so route_follower.py can reuse it.)"""
        dt = (t - self.prev_t) if self.prev_t is not None else 0.0
        d = (angle - self.prev_err) / dt if dt > 1e-6 else 0.0
        self.prev_t, self.prev_err = t, angle
        wz = max(-self.a.max_turn, min(self.a.max_turn, self.a.kp * angle + self.a.kd * d))
        vx = (self.a.speed if speed is None else speed) * max(0.4, 1.0 - abs(angle) / 0.6)
        return vx, wz

    def send(self, vx=0.0, wz=0.0):
        t = Twist()
        t.linear.x = float(vx)
        t.angular.z = float(wz)
        self.pub.publish(t)

    def on_image(self, msg):
        if self.result:
            return
        try:
            bgr = image_to_bgr(msg)
        except ValueError as e:
            self.get_logger().error(str(e))
            return
        t = self.stamp(msg)
        if self.t_start is None:
            self.t_start = t
        self.frames += 1
        angle, red_frac = self.analyse(bgr)

        if t - self.t_start > self.a.ignore_stop and red_frac > self.a.stop_frac:
            self.red_frames += 1
        else:
            self.red_frames = 0
        if self.red_frames >= 2:
            self.send()
            self.result = 'arrived'
            self.get_logger().info('Red stop marker reached.')
        elif angle is None:
            self.send()
            self.lost_since = self.lost_since if self.lost_since is not None else t
            if t - self.lost_since > self.a.lost_timeout:
                self.result = 'lost'
                self.get_logger().warn('Line lost for %.1f s - stopping.' % self.a.lost_timeout)
        else:
            self.lost_since = None
            self.send(*self.steer(angle, t))

        now = time.monotonic()
        if now - self.last_debug > 1.0:
            self.last_debug = now
            txt = 'angle=%s red=%.2f %s' % ('none' if angle is None else '%.3f' % angle,
                                          red_frac, self.result or '')
            cv2.putText(bgr, txt, (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1)
            cv2.imwrite('/tmp/line_follower_debug.jpg', bgr)
            self.get_logger().info('sim t=%.1f  %s' % (t - self.t_start, txt))
        if self.a.show:
            cv2.imshow('line follower', bgr)
            cv2.waitKey(1)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--speed', type=float, default=0.08, help='forward speed m/s (default 0.08)')
    p.add_argument('--kp', type=float, default=1.2)
    p.add_argument('--kd', type=float, default=0.1)
    p.add_argument('--max-turn', type=float, default=0.6, help='max yaw rate rad/s')
    p.add_argument('--min-area', type=float, default=15, help='min blob area in pixels')
    p.add_argument('--stop-frac', type=float, default=0.15, help='red fraction of the strips that means stop')
    p.add_argument('--ignore-stop', type=float, default=0.0, help='ignore red for the first S sim seconds')
    p.add_argument('--lost-timeout', type=float, default=2.0, help='sim seconds without line before giving up')
    p.add_argument('--show', action='store_true', help='open a camera window')
    a = p.parse_args()

    rclpy.init()
    node = LineFollower(a)
    node.get_logger().info('Waiting for camera images on %s ...' % IMAGE_TOPIC)
    code = 1
    try:
        while rclpy.ok() and node.result is None:
            rclpy.spin_once(node, timeout_sec=0.2)
        code = 0 if node.result == 'arrived' else 2
    except KeyboardInterrupt:
        print('\nAborted.')
    finally:
        for _ in range(3):
            node.send()
            time.sleep(0.05)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print('Result:', node.result or 'aborted')
    sys.exit(code)


if __name__ == '__main__':
    main()
