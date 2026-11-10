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

Hospital system (scheduler / delivery.py logs): the caption shows the time, patient, room,
medicine and step - the time is the HOSPITAL CLOCK when the log has a line
"[clock] hospital HH:MM:SS = sim T" (scheduler --demo; frame time = HH:MM:SS + frame sim
time - T), else the wall clock. --wrist-inset adds the wrist camera in a corner.
"""
import datetime as dt
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
    """(step, sub line, all caption-relevant text, header, clock (start, sim0) or None)."""
    try:
        with open(path, errors='replace') as fh:
            lines = [l for l in fh if '[mission]' in l or '[delivery]' in l or '[clock]' in l or '[scheduler' in l]
    except OSError:
        return '', '', '', '', None
    step, sub, header, clock = '', '', '', None
    for l in lines:
        m = re.search(r'\[(?:mission|delivery)\] (step \d+/\d+ [^(]*)', l)
        if m:
            step, sub = m.group(1).strip(), ''
            continue
        m = re.search(r'\[(?:mission|delivery)\] table: (.*)', l)
        if m:
            sub = 'smart table: ' + m.group(1).strip()
            continue
        m = re.search(r'\[delivery\] (Room \d+) \| ([^|]*) \| ([^|]*) \| (slot \S+)', l)
        if m:
            header = '%s | %s | %s | %s' % (m.group(1), m.group(2).strip(), m.group(3).strip(), m.group(4))
            step, sub = '', ''
            continue
        m = re.search(r'\[clock\] hospital (\d+):(\d+):(\d+) = sim ([-\d.]+)', l)
        if m:
            clock = (int(m.group(1)) * 3600 + int(m.group(2)) * 60 + int(m.group(3)), float(m.group(4)))
        if 'OUT OF STOCK' in l:
            sub = 'scheduler: ' + l.split('] ', 1)[-1].strip()
    return step, sub, ''.join(lines), header, clock


def clock_text(t, clock):
    if clock is None:
        return dt.datetime.now().strftime('%H:%M:%S (local)')
    sec = int(round(clock[0] + t - clock[1])) % 86400
    return 'hospital clock %02d:%02d:%02d' % (sec // 3600, sec // 60 % 60, sec % 60)


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
    p.add_argument('--wrist-inset', action='store_true', help='wrist camera picture in the lower right corner')
    a = p.parse_args()
    signal.signal(signal.SIGTERM, on_signal)
    signal.signal(signal.SIGINT, on_signal)

    rclpy.init()
    node = rclpy.create_node('record_video')
    frames = []
    node.create_subscription(Image, '/overview_cam/image', frames.append, 10)
    wrist = {}
    if a.wrist_inset:
        node.create_subscription(Image, '/depth_cam/depth_cam', lambda m: wrist.__setitem__('m', m), 2)
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
            step, sub, alltext, header, clock = last_step(a.caption_log) if a.caption_log else ('', '', '', '', None)
            text(img, a.title, (12, 26), 0.65)
            text(img, 'sim time %6.1f s   (video %.1fx real time)' % (t - t0, a.fps / 2.0),
                 (12, img.shape[0] - 16), 0.55)
            y = 54
            if header or clock is not None:
                text(img, '%s   %s' % (clock_text(t, clock), header), (12, y), 0.58)
                y += 26
            if step:
                text(img, step, (12, y), 0.6)
                y += 26
            if sub:
                text(img, sub, (12, y), 0.55)
            if 'm' in wrist:
                wi = cv2.resize(to_bgr(wrist['m']), (240, 150))
                h, w = img.shape[:2]
                y0, x0 = h - 150 - 34, w - 240 - 12
                img[y0:y0 + 150, x0:x0 + 240] = wi
                cv2.rectangle(img, (x0 - 1, y0 - 1), (x0 + 240, y0 + 150), (40, 40, 40), 1)
                text(img, 'wrist camera', (x0, y0 - 6), 0.45)
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
