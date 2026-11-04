#!/usr/bin/env python3
"""Save a picture of the simulation without the Gazebo GUI.

Uses the fixed overview camera in hospital.sdf (/overview_cam/image, 0.5 Hz sim,
rendered by the Gazebo server, so it also works with gui:=false). Waits for the
first frame rendered AFTER this script started (sim time from /joint_states), so
the picture is never stale. At ~5 % real-time factor that takes up to ~40 s.

    python3 ~/sdp_sim/tools/sim_screenshot.py [out.png] [--wrist] [--timeout S]

--wrist also saves the next wrist-camera frame (/depth_cam/depth_cam) next to it
as <out>_wrist.png.
"""
import argparse
import sys
import time

import cv2
import numpy as np
import rclpy
from sensor_msgs.msg import Image, JointState


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def to_bgr(msg):
    a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3]
    a = a.reshape(msg.height, msg.width, 3)
    return cv2.cvtColor(a, cv2.COLOR_RGB2BGR) if msg.encoding == 'rgb8' else a.copy()


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('out', nargs='?', default='/tmp/sim_shot.png')
    p.add_argument('--wrist', action='store_true', help='also save the next wrist-camera frame')
    p.add_argument('--timeout', type=float, default=180.0, help='wall-clock seconds to wait')
    a = p.parse_args()

    rclpy.init()
    node = rclpy.create_node('sim_screenshot')
    state = {'t0': None, 'shot': None, 'wrist': None}

    def on_js(m):
        if state['t0'] is None:
            state['t0'] = stamp(m)

    def on_img(key):
        def cb(m):
            if state['t0'] is not None and state[key] is None and stamp(m) > state['t0']:
                state[key] = m
        return cb

    node.create_subscription(JointState, '/joint_states', on_js, 10)
    node.create_subscription(Image, '/overview_cam/image', on_img('shot'), 2)
    if a.wrist:
        node.create_subscription(Image, '/depth_cam/depth_cam', on_img('wrist'), 2)

    end = time.monotonic() + a.timeout
    while time.monotonic() < end and (state['shot'] is None or (a.wrist and state['wrist'] is None)):
        rclpy.spin_once(node, timeout_sec=0.5)

    ok = True
    if state['shot'] is None:
        print('no overview frame within %.0f s (is the simulation running?)' % a.timeout)
        ok = False
    else:
        cv2.imwrite(a.out, to_bgr(state['shot']))
        print('saved %s (sim t=%.2f)' % (a.out, stamp(state['shot'])))
    if a.wrist:
        if state['wrist'] is None:
            print('no wrist-camera frame')
            ok = False
        else:
            w = a.out.rsplit('.', 1)[0] + '_wrist.png'
            cv2.imwrite(w, to_bgr(state['wrist']))
            print('saved %s (sim t=%.2f)' % (w, stamp(state['wrist'])))
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
