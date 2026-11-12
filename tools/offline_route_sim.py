#!/usr/bin/env python3
"""Offline test of the line follower and the route logic - synthetic camera, no Gazebo.

    python3 ~/sdp_sim/tools/offline_route_sim.py            # all tests, summary at the end
    python3 ~/sdp_sim/tools/offline_route_sim.py --only old  # one group: old | legs | fast | faults

Synthetic wrist camera: 320 x 200 px, hfov 1.047, 0.396 m above the floor at 54.1 deg
pitch (tf2_echo, arm in the home pose), CAM_AHEAD in front of joint1 - every pixel is
intersected with the floor and coloured from a 2 mm raster of the tape and markers
(colours as rendered by Gazebo: line HSV (111, 250, 255), red, junction green).
Kinematic base = the planar base: the commanded twist (robot frame) is applied one
0.02 s physics step later; frames every 0.25 s sim (4 Hz). Maneuvers use the same
control laws as base_moves.BaseMover (translate / rotate).

Groups
  old     the TESTED line_follower.LineFollower.on_image, unchanged, on the single-room
          route: must stop where Gazebo did (patient stop (1.6992, 1.4723)); this also
          calibrates the camera model (the old offline test of 2026-11-04 is not in the repo)
  legs    route_follower.RouteLogic on 17 delivery legs (home->pharmacy; pharmacy slot -> room:
          S1-S6 for Room 1, S3 and S6 for Rooms 2-4; room -> home), each from 3 starts (the pose
          the mission drives to, and +-12 mm / +-1 deg off it) = 51 runs
  fast    the same 51 runs at --fast-speed 0.12 on long straights
  faults  a missing junction marker, a wrong-coloured junction marker, a missing room stop marker:
          the robot must stop with an error
"""
import argparse
import math
import os
import sys
import types

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, 'scripts'))
import system_layout as L          # noqa: E402
import line_follower as lf         # noqa: E402
import route_follower as rf        # noqa: E402
import route_planner as rp         # noqa: E402

RES = 0.002                        # m per raster pixel
BGR_FLOOR, BGR_LINE = (255, 255, 255), (255, 80, 5)      # Gazebo: floor (0,0,255), line (111,250,255) HSV
BGR_RED, BGR_GREEN = (20, 20, 255), (75, 255, 40)
DT, FRAME_DT = 0.02, 0.25
CAM_AHEAD = L.CAM_AHEAD


# -------------------------------------------------------------------- camera
def pixel_ground(cam_ahead=CAM_AHEAD):
    """(ahead, left) floor offsets of every pixel, arrays of shape (H, W)."""
    v, u = np.mgrid[0:L.IMG_H, 0:L.IMG_W].astype(np.float64)
    u += 0.5
    v += 0.5
    a = L.CAM_PITCH + np.arctan((v - L.IMG_H / 2) / L.F_PX)
    d = L.CAM_H / np.tan(a)
    zc = np.hypot(L.CAM_H, d) * np.cos(a - L.CAM_PITCH)
    return cam_ahead + d, -(u - L.IMG_W / 2) / L.F_PX * zc


class Floor:
    def __init__(self, x0, x1, y0, y1):
        self.x0, self.y1 = x0, y1
        self.img = np.full((int((y1 - y0) / RES) + 1, int((x1 - x0) / RES) + 1, 3), BGR_FLOOR, np.uint8)

    def px(self, x, y):
        return int(round((x - self.x0) / RES)), int(round((self.y1 - y) / RES))

    def seg(self, a, b, colour=BGR_LINE, w=L.LINE_W):
        cv2.line(self.img, self.px(*a), self.px(*b), colour, max(1, int(round(w / RES))), cv2.LINE_8)

    def square(self, c, colour, s=L.MARKER):
        p0, p1 = self.px(c[0] - s / 2, c[1] + s / 2), self.px(c[0] + s / 2, c[1] - s / 2)
        cv2.rectangle(self.img, p0, p1, colour, -1)

    def render(self, pose, ground):
        x, y, yaw = pose
        ga, gl = ground
        c, s = math.cos(yaw), math.sin(yaw)
        wx, wy = x + c * ga - s * gl, y + s * ga + c * gl
        ix = np.clip(np.rint((wx - self.x0) / RES).astype(int), 0, self.img.shape[1] - 1)
        iy = np.clip(np.rint((self.y1 - wy) / RES).astype(int), 0, self.img.shape[0] - 1)
        return self.img[iy, ix].copy()


def hospital_floor(skip_markers=(), recolour=None):
    nodes, edges = L.graph()
    f = Floor(-4.6, 1.8, -2.3, 2.3)
    # tape: the main line ends at the pharmacy marker's far edge, branches at the room markers' far edges
    f.seg((L.JUNCTIONS['J2'], 0.0), (L.PHARM_MARK + L.MARKER / 2, 0.0))
    for r in L.ROOMS:
        mx, my = L.room_info(r)['marker']
        f.seg((mx, 0.0), (mx, my + math.copysign(L.MARKER / 2, my)))
    for n, (x, y, kind, colour) in nodes.items():
        if n in skip_markers:
            continue
        col = BGR_GREEN if colour == 'green' else BGR_RED
        if recolour and n in recolour:
            col = recolour[n]
        f.square((x, y), col)
    return f


def old_floor():
    """The single-room route (tools/make_hospital_world.py line_model)."""
    import make_hospital_world as W
    f = Floor(-0.6, 2.2, -0.6, 2.2)
    f.seg((W.HOME_X, 0.0), (W.X_TURN, 0.0))
    pts = [(W.X_TURN + W.R_TURN * math.cos(-math.pi / 2 + (math.pi / 2) * i / 60),
            W.R_TURN + W.R_TURN * math.sin(-math.pi / 2 + (math.pi / 2) * i / 60)) for i in range(61)]
    for a, b in zip(pts, pts[1:]):
        f.seg(a, b)
    f.seg((W.XS, W.R_TURN), (W.XS, W.Y_END))
    f.square((W.XS, W.Y_END), BGR_RED)
    f.square((W.HOME_X, 0.0), BGR_RED)
    return f


# -------------------------------------------------------------------- kinematic planar base
class FakeBase:
    """Planar base: twist (robot frame) applied one physics step after it is sent."""

    def __init__(self, sim, pose):
        self.sim = sim
        self.x, self.y, self.yaw = pose
        self.cmd = (0.0, 0.0, 0.0)
        self.next_cmd = (0.0, 0.0, 0.0)

    def step(self):
        vx, vy, wz = self.cmd
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        self.x += (c * vx - s * vy) * DT
        self.y += (s * vx + c * vy) * DT
        self.yaw = rf.wrap(self.yaw + wz * DT)
        self.cmd = self.next_cmd
        self.sim.t += DT
        self.sim.trace.append((self.sim.t, self.x, self.y, self.yaw))

    def pose_now(self):
        return self.x, self.y, self.yaw, self.sim.t

    def pose(self):
        self.step()
        return self.pose_now()

    def send(self, vx=0.0, vy=0.0, wz=0.0):
        self.next_cmd = (float(vx), float(vy), float(wz))

    def stop(self):
        self.send()
        self.step()
        self.step()

    # same control laws as base_moves.BaseMover
    def translate(self, dx_robot, dy_robot, vmax=0.05, tol=0.002, timeout_sim=60.0):
        x0, y0, yaw0, t0 = self.pose()
        c, s = math.cos(yaw0), math.sin(yaw0)
        gx, gy = x0 + c * dx_robot - s * dy_robot, y0 + s * dx_robot + c * dy_robot
        while True:
            ex_w, ey_w = gx - self.x, gy - self.y
            if math.hypot(ex_w, ey_w) < tol:
                break
            if self.sim.t - t0 > timeout_sim:
                raise RuntimeError('translate timed out')
            cy, sy = math.cos(self.yaw), math.sin(self.yaw)
            ex, ey = cy * ex_w + sy * ey_w, -sy * ex_w + cy * ey_w
            vx, vy = 1.5 * ex, 1.5 * ey
            n = math.hypot(vx, vy)
            if n > vmax:
                vx, vy = vx * vmax / n, vy * vmax / n
            self.send(vx, vy, 2.0 * rf.wrap(yaw0 - self.yaw))
            self.step()
        self.stop()

    def rotate(self, dyaw, wmax=0.4, tol=math.radians(0.3), timeout_sim=60.0):
        yaw_prev, done, t0 = self.yaw, 0.0, self.sim.t
        while True:
            done += rf.wrap(self.yaw - yaw_prev)
            yaw_prev = self.yaw
            err = dyaw - done
            if abs(err) < tol:
                break
            if self.sim.t - t0 > timeout_sim:
                raise RuntimeError('rotate timed out')
            w = max(-wmax, min(wmax, 2.0 * err))
            if abs(w) < 0.02:
                w = math.copysign(0.02, err)
            self.send(0.0, 0.0, w)
            self.step()
        self.stop()

    def turnaround(self, offset=0.12, side=-1):
        self.translate(0.0, side * offset)
        self.rotate(math.pi)
        self.translate(0.0, side * offset)


class Sim:
    def __init__(self, floor, pose, phase=0.0, cam_ahead=CAM_AHEAD):
        self.t = 0.0
        self.trace = []
        self.floor = floor
        self.ground = pixel_ground(cam_ahead)
        self.base = FakeBase(self, pose)
        self.next_frame = phase

    def frame(self):
        return self.floor.render((self.base.x, self.base.y, self.base.yaw), self.ground)

    def advance_to_frame(self):
        while self.t + 1e-9 < self.next_frame:
            self.base.step()
        self.next_frame += FRAME_DT


def follower_args(**kw):
    a = rf.build_args([])
    for k, v in kw.items():
        setattr(a, k, v)
    return a


# -------------------------------------------------------------------- runs
def run_old(phase=0.0, cam_ahead=CAM_AHEAD, start=(0.0, 0.0, 0.0), timeout=120.0):
    """The tested LineFollower.on_image, unchanged, with a stand-in for the ROS parts."""
    sim = Sim(old_floor(), start, phase, cam_ahead)
    fake = types.SimpleNamespace()
    lf.LineFollower.__init__.__globals__  # (module import check)
    fake.a = follower_args()
    fake.result, fake.prev_t, fake.prev_err, fake.lost_since = None, None, 0.0, None
    fake.red_frames, fake.frames, fake.t_start, fake.last_debug = 0, 0, None, 1e18
    fake.stamp = lambda m: m.t
    fake.analyse = lambda bgr: lf.LineFollower.analyse(fake, bgr)
    fake.steer = lambda angle, t, speed=None: lf.LineFollower.steer(fake, angle, t, speed)
    fake.send = lambda vx=0.0, wz=0.0: sim.base.send(vx, 0.0, wz)
    fake.get_logger = lambda: types.SimpleNamespace(info=lambda m: None, warn=lambda m: None, error=lambda m: None)
    while fake.result is None and sim.t < timeout:
        sim.advance_to_frame()
        rgb = cv2.cvtColor(sim.frame(), cv2.COLOR_BGR2RGB)
        msg = types.SimpleNamespace(data=rgb.tobytes(), encoding='rgb8', height=L.IMG_H, width=L.IMG_W,
                                    step=L.IMG_W * 3, t=sim.t)
        lf.LineFollower.on_image(fake, msg)
    for _ in range(3):
        sim.base.step()
    return fake.result, (sim.base.x, sim.base.y, math.degrees(sim.base.yaw)), sim.t


def run_leg(plan, start, floor=None, phase=0.0, timeout=300.0, cam_ahead=CAM_AHEAD, **kw):
    sim = Sim(floor or hospital_floor(), start, phase, cam_ahead)
    logs = []
    logic = rf.RouteLogic(follower_args(**kw), plan, sim.base, log=logs.append)
    if plan.get('turn_round'):
        sim.base.turnaround()
    t0 = sim.t
    xte_follow = 0.0
    while logic.result is None and sim.t - t0 < timeout:
        sim.advance_to_frame()
        logic.on_frame(sim.frame(), sim.t)
        if logic.pending:
            logic.run_pending()
    for _ in range(3):
        sim.base.step()
    out = logic.summary()
    out['final'] = (sim.base.x, sim.base.y, math.degrees(sim.base.yaw))
    out['sim_s'] = sim.t - t0
    out['log'] = logs
    return out


def expected_stop(plan):
    ev = plan['events'][-1]
    h = math.radians(ev['edge_heading'])
    return ev['x'] - L.STOP_AHEAD * math.cos(h), ev['y'] - L.STOP_AHEAD * math.sin(h)


# -------------------------------------------------------------------- test groups
RESULTS = []


def report(name, ok, msg):
    RESULTS.append((name, ok, msg))
    print('%-4s %-46s %s' % ('ok' if ok else 'FAIL', name, msg), flush=True)


def group_old(a):
    import make_hospital_world as W
    stops = []
    for ph in (0.0, 0.0625, 0.125, 0.1875):
        res, (x, y, yd), t = run_old(phase=ph, cam_ahead=a.cam_ahead)
        stops.append(y)
        report('old route (tested follower), phase %.3f' % ph,
               res == 'arrived' and abs(x - 1.70) < 0.01 and abs(y - 1.4723) < 0.02,
               'result %s, stop (%.4f, %.4f) yaw %.2f, %.1f sim s; Gazebo (1.6992, 1.4723), 38.6-39.0 s'
               % (res, x, y, yd, t))
    print('     -> offline stop ahead of the marker: %.4f m (Gazebo %.4f)' % (W.Y_END - sum(stops) / len(stops), L.STOP_AHEAD))


def leg_cases(slots):
    net = rp.Network()
    cases = []
    for room in (1, 2, 3, 4):
        leg1, leg2, leg3 = rp.delivery_legs(net, room)
        if room == 1:
            cases.append(('home -> pharmacy', leg1, (0.0, 0.0, 0.0)))
        for s in (slots if room == 1 else ['S3', 'S6']):
            x = L.SLOT_X[s]
            p = dict(leg2, turn_round=False)
            cases.append(('pharmacy %s -> R%d' % (s, room), p, (x, 0.0, math.pi)))
        sx, sy, hd = L.room_info(room)['stop']
        p = dict(leg3, turn_round=False)
        cases.append(('R%d -> home' % room, p, (sx, sy, math.radians(hd + 180.0))))
    return cases


def check_leg(name, plan, start, **kw):
    worst = None
    for k, (dl, dyaw, ph) in enumerate(((0.0, 0.0, 0.0), (0.012, 1.0, 0.11), (-0.012, -1.0, 0.19))):
        x, y, yaw = start
        sx = (x - dl * math.sin(yaw), y + dl * math.cos(yaw), yaw + math.radians(dyaw))
        out = run_leg(plan, sx, phase=ph, **kw)
        ex, ey = expected_stop(plan)
        fx, fy, fyaw = out['final']
        err = math.hypot(fx - ex, fy - ey)
        ok = out['result'] == 'arrived' and err < 0.03 and out['max_xte'] < 0.035
        msg = ('%s, stop %.1f mm from the expected, max line offset %.1f mm, %.1f sim s%s'
               % (out['result'], 1000 * err, 1000 * out['max_xte'], out['sim_s'],
                  ', fast frames %d' % out['fast_frames'] if kw.get('fast_speed') else ''))
        if out['error']:
            msg += ' - ' + out['error']
        report('%s [offset %+.0f mm %+.0f deg]' % (name, 1000 * dl, dyaw), ok, msg)
        if not ok and worst is None:
            worst = out
    return worst


def group_legs(a, **kw):
    bad = None
    for name, plan, start in leg_cases(['S1', 'S2', 'S3', 'S4', 'S5', 'S6']):
        w = check_leg(name + (' fast' if kw else ''), plan, start, cam_ahead=a.cam_ahead, **kw)
        bad = bad or w
    if bad and a.verbose:
        print('\n'.join(bad['log']))


def group_faults(a):
    net = rp.Network()
    _, leg2, leg3 = rp.delivery_legs(net, 3)
    plan = dict(leg2, turn_round=False)
    # J1 marker missing: must stop with "missed", near J1, not wander into a branch
    out = run_leg(plan, (L.SLOT_X['S3'], 0.0, math.pi), floor=hospital_floor(skip_markers=('J1',)), cam_ahead=a.cam_ahead)
    fx, fy, _ = out['final']
    report('fault: J1 marker missing', out['result'] == 'error' and 'missed' in (out['error'] or '') and
           abs(fy) < 0.03 and abs(fx - L.JUNCTIONS['J1']) < 0.25,
           '%s (%s), stopped at (%.3f, %.3f)' % (out['result'], out['error'], fx, fy))
    # J2 marker red instead of green: unexpected colour -> stop
    out = run_leg(plan, (L.SLOT_X['S3'], 0.0, math.pi), floor=hospital_floor(recolour={'J2': BGR_RED}), cam_ahead=a.cam_ahead)
    fx, fy, _ = out['final']
    report('fault: J2 marker red', out['result'] == 'error' and 'unexpected' in (out['error'] or ''),
           '%s (%s), stopped at (%.3f, %.3f)' % (out['result'], out['error'], fx, fy))
    # room marker missing: the robot must not drive into the table
    p3 = dict(rp.Network().plan('pharmacy', 'R1', 180.0), turn_round=False)
    out = run_leg(p3, (L.SLOT_X['S3'], 0.0, math.pi), floor=hospital_floor(skip_markers=('R1',)), cam_ahead=a.cam_ahead)
    fx, fy, _ = out['final']
    report('fault: R1 stop marker missing', out['result'] in ('error', 'lost') and fy < L.room_info(1)['stop'][1] + 0.2,
           '%s (%s), stopped at (%.3f, %.3f)' % (out['result'], out['error'], fx, fy))


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--only', choices=['old', 'legs', 'fast', 'faults'], default=None)
    p.add_argument('--cam-ahead', type=float, default=CAM_AHEAD)
    p.add_argument('--verbose', action='store_true')
    a = p.parse_args()
    groups = {'old': lambda: group_old(a), 'legs': lambda: group_legs(a),
              'fast': lambda: group_legs(a, fast_speed=0.12), 'faults': lambda: group_faults(a)}
    for g, fn in groups.items():
        if a.only in (None, g):
            print('--- %s' % g)
            fn()
    n_bad = sum(1 for _, ok, _ in RESULTS if not ok)
    print('\n%d / %d checks passed' % (len(RESULTS) - n_bad, len(RESULTS)))
    sys.exit(1 if n_bad else 0)


if __name__ == '__main__':
    main()
