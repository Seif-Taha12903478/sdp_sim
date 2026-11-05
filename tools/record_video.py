#!/usr/bin/env python3
"""Record a video from the overview camera (/overview_cam/image) - no GUI needed.

    python3 record_video.py OUT.mp4 [--fps 15] [--caption-log mission.log] [--title TEXT]

Launch the simulation with video:=true first (overview camera at 2 Hz sim; the normal
world has it at 0.5 Hz). Every camera frame becomes one video frame, so the video
plays at fps / 2 times real speed (15 fps -> 7.5x). Each frame shows the sim time
and, with --caption-log, the latest "[mission] step ..." line of that log file and,
below it, the latest "[mission] table: ..." line of that step (smart table lifting).
Stops on Ctrl+C / SIGTERM, or after --max-sim seconds of sim time, or when the
caption log contains "--stop-on" (default "[mission] done"), plus --tail seconds.
Writes H.264 (avc1) MP4; falls back to mp4v if avc1 is not available.
"""
import argparse
import re
import signal

import cv2
import numpy as np
import rclpy
from sensor_msgs.msg import Image

STOP = False


def on_signal(*_):
    global STOP
    STOP = True


def to_bgr(msg):
    a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3]
    a = a.reshape(msg.height, msg.width, 3)
    return cv2.cvtColor(a, cv2.COLOR_RGB2BGR) if msg.encoding == 'rgb8' else a.copy()


def last_step(path):
    try:
        with open(path, errors='replace') as fh:
            lines = [l for l in fh if '[mission]' in l]
    except OSError:
        return '', '', ''
    step, sub = '', ''
    for l in lines:
        m = re.search(r'\[mission\] (step \d+/\d+ [^(]*)', l)
        if m:
            step, sub = m.group(1).strip(), ''
            continue
        m = re.search(r'\[mission\] table: (.*)', l)
        if m:
            sub = 'smart table: ' + m.group(1).strip()
    return step, sub, ''.join(lines)


def text(img, s, org, scale=0.6):
    cv2.putText(img, s, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (255, 255, 255), 4, cv2.LINE_AA)
    cv2.putText(img, s, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (30, 30, 30), 1, cv2.LINE_AA)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('out')
    p.add_argument('--fps', type=float, default=15.0)
    p.add_argument('--caption-log', default=None)
    p.add_argument('--title', default='Medicine delivery robot - Gazebo simulation')
    p.add_argument('--max-sim', type=float, default=1e9, help='stop after this many sim seconds')
    p.add_argument('--stop-on', default='[mission] done')
    p.add_argument('--tail', type=float, default=6.0, help='sim seconds to keep recording after stop-on')
    a = p.parse_args()
    signal.signal(signal.SIGTERM, on_signal)
    signal.signal(signal.SIGINT, on_signal)

    rclpy.init()
    node = rclpy.create_node('record_video')
    frames = []
    node.create_subscription(Image, '/overview_cam/image', frames.append, 10)
    writer, t0, n, stop_at = None, None, 0, None
    print('recording %s from /overview_cam/image (Ctrl+C to stop)' % a.out, flush=True)
    while not STOP:
        rclpy.spin_once(node, timeout_sec=0.5)
        while frames:
            m = frames.pop(0)
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            t0 = t if t0 is None else t0
            img = to_bgr(m)
            if writer is None:
                h, w = img.shape[:2]
                for cc in ('avc1', 'mp4v'):
                    writer = cv2.VideoWriter(a.out, cv2.VideoWriter_fourcc(*cc), a.fps, (w, h))
                    if writer.isOpened():
                        print('codec', cc, flush=True)
                        break
            step, sub, alltext = last_step(a.caption_log) if a.caption_log else ('', '', '')
            text(img, a.title, (12, 26), 0.65)
            text(img, 'sim time %6.1f s   (video %.1fx real time)' % (t - t0, a.fps / 2.0),
                 (12, img.shape[0] - 16), 0.55)
            if step:
                text(img, step, (12, 54), 0.6)
            if sub:
                text(img, sub, (12, 80), 0.55)
            writer.write(img)
            n += 1
            if stop_at is None and a.caption_log and a.stop_on in alltext:
                stop_at = t + a.tail
            if t - t0 >= a.max_sim or (stop_at is not None and t >= stop_at):
                on_signal()
    if writer is not None:
        writer.release()
    print('wrote %d frames (%.1f s of video) to %s' % (n, n / a.fps, a.out), flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
