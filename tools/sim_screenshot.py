#!/usr/bin/env python3
"""Save a picture of the simulation without the Gazebo GUI.

Uses the fixed overview camera in hospital.sdf (/overview_cam/image, 0.5 Hz sim,
rendered by the Gazebo server, so it also works with gui:=false). Waits for the
first frame rendered AFTER this script started (sim time from /joint_states), so
the picture is never stale. At ~5 % real-time factor that takes up to ~40 s.

    python3 ~/sdp_sim/tools/sim_screenshot.py [out.png] [--wrist] [--timeout S]
    python3 ~/sdp_sim/tools/sim_screenshot.py out.png --topic /room_cam/a/image
    python3 ~/sdp_sim/tools/sim_screenshot.py ~/sdp_sim/media/room.png --room

--wrist also saves the next wrist-camera frame (/depth_cam/depth_cam) next to it
as <out>_wrist.png. --topic uses another camera. --room (world launched with
photo:=true, hospital_photo.sdf) saves the three 1280x800 room cameras
/room_cam/{a,b,c}/image as <out>_a.png, <out>_b.png, <out>_c.png (0.25 Hz sim:
up to ~45 s wall at 9 % RTF).
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
    p.add_argument('--topic', default='/overview_cam/image', help='camera image topic for OUT')
    p.add_argument('--room', action='store_true', help='save the three room cameras (photo world)')
    a = p.parse_args()
    base = a.out.rsplit('.', 1)[0]
    shots = ({'/room_cam/%s/image' % k: '%s_%s.png' % (base, k) for k in 'abc'} if a.room
             else {a.topic: a.out})
    if a.wrist:
        shots['/depth_cam/depth_cam'] = base + '_wrist.png'

    rclpy.init()
    node = rclpy.create_node('sim_screenshot')
    state = {'t0': None}
    state.update({t: None for t in shots})

    def on_js(m):
        if state['t0'] is None:
            state['t0'] = stamp(m)

    def on_img(key):
        def cb(m):
            if state['t0'] is not None and state[key] is None and stamp(m) > state['t0']:
                state[key] = m
        return cb

    node.create_subscription(JointState, '/joint_states', on_js, 10)
    for t in shots:
        node.create_subscription(Image, t, on_img(t), 2)

    end = time.monotonic() + a.timeout
    while time.monotonic() < end and any(state[t] is None for t in shots):
        rclpy.spin_once(node, timeout_sec=0.5)

    ok = True
    for t, path in shots.items():
        if state[t] is None:
            print('no frame on %s within %.0f s (is the simulation running%s?)'
                  % (t, a.timeout, ' with photo:=true' if a.room else ''))
            ok = False
        else:
            cv2.imwrite(path, to_bgr(state[t]))
            print('saved %s (sim t=%.2f)' % (path, stamp(state[t])))
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
