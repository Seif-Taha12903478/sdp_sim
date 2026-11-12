#!/usr/bin/env python3
"""One delivery in the hospital system: home -> pharmacy slot -> room -> home.

    python3 ~/sdp_sim/scripts/delivery.py --room 3 --slot S3 [--patient "Patient C"]
        [--result-json FILE] [--label TEXT] [--fast-speed 0.12] [--cap-check]

Needs a running hospital_system.launch.py, the pharmacy set up (scripts/pharmacy.py setup)
and the robot parked at home (0, 0) facing +x. Steps (log lines "[delivery] step i/15 ..."
are the video captions):
  1 start check: robot parked at home, med_<slot> in its slot (all container poses recorded)
  2 arm home
  3 follow the line to the pharmacy             (route_follower.py: stop at the red P marker)
  4 line up with the slot: forward/back along the line on /odom (counter on the robot's left)
  5 (--cap-check) look at the container: the cap colour must be the inventory's
  6 pick at arm-relative (0, 0.20): open, above, descend, close 0.75 + check, attach confirmed
    (/med_<slot>), lift + check that the container came up
  7 arm home (carrying)
  8 turn round at the pharmacy: strafe right 0.12 (away from the counter), rotate 180, strafe right
  9 follow the line to the room                 (junction actions from route_planner.py)
 10 place on the room's smart table: it must be lowered; above, descend, detach confirmed, open,
    retreat; placement read when the table's contact sensor confirms the container
 11 arm home
 12 the table lifts the medicine to the patient: wait for "raised", check it is on the platform
 13 turn round (the container must still be on the platform)
 14 follow the line home                        (stop at the red home marker)
 15 park at home: (0, 0) yaw 0 on /odom
The pick / place / lift sequences and checks are mission.py's (tested), with this
container's own weld and this room's table. Exit 0 = delivery completed. The result JSON
(--result-json) has the same fields as mission.py's plus the delivery, the route legs and
the poses of ALL containers at the start and at the end (tools/scoring.score_delivery).
"""
import argparse
import json
import math
import os
import re
import subprocess
import sys
import tempfile
import threading
import time

import rclpy

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import jetarm_pick_place as jp          # noqa: E402
from base_moves import BaseMover        # noqa: E402
from medicine_weld import Weld          # noqa: E402
import smart_table as tbl               # noqa: E402
import pharmacy as ph                   # noqa: E402
import route_planner as rp              # noqa: E402
from mission import gz_model_pose_full, tilt_deg     # noqa: E402

REACH_XY = (0.0, 0.20)
TABLE_Z = 0.13
PLACE_TOL = 0.03
HOME_TOL = (0.03, 2.0)                  # m, deg: "parked at home"
N_STEPS = 15
WELD_LOG = []


def log(msg):
    print('[delivery] ' + msg, flush=True)


def confirm(weld, kind, where):
    ok, n = weld.command(kind)
    WELD_LOG.append({'where': where, 'kind': kind, 'attempts': n, 'confirmed': ok, 'sim_t': weld.sim_t})
    if ok:
        log('%s (%s) confirmed by %s/attached after %d attempt(s)' % (kind, where, weld.prefix, n))
    else:
        log('WARNING: %s not confirmed after %d attempts (last state %s) - relying on the pose check'
            % (kind, n, weld.last_state()))
    return ok


class ContactPose:
    """Container pose read the moment the table's contact sensor says "steady" (thread)."""

    def __init__(self, table, name):
        self.name, self.result, self.thread, self.armed = name, None, None, False
        table.on_contact = self._on_contact

    def arm(self):
        self.result, self.thread, self.armed = None, None, True

    def _on_contact(self, state, sim_t):
        if self.armed and state == 'steady':
            self.armed = False
            self.thread = threading.Thread(target=self._grab, args=(sim_t,), daemon=True)
            self.thread.start()

    def _grab(self, sim_t):
        c = gz_model_pose_full(self.name)
        self.result = {'source': 'contact sensor', 'sim_t': sim_t,
                       'xyz': list(c[0]) if c else None, 'quat_xyzw': list(c[1]) if c else None}

    def get(self, timeout=180.0):
        if self.thread is not None:
            self.thread.join(timeout)
        return self.result


def arm_plan():
    """Pick / place waypoints of the tested mission (arm-relative (0, 0.20), tool down)."""
    pa = argparse.Namespace(pick=list(REACH_XY), place=list(REACH_XY), grasp_z=0.038, release_z=0.045,
                            above_z=0.14, open=1.4, closed=0.52, carry_grip=0.75)
    return pa, jp.build_plan(pa)


# pharmacy line-up: translate() alone stops within 2 mm and the grasp keeps that offset
# (Room 3 test d 2026-11-10: robot 1.9 mm short of S3 -> container placed 2.7 mm off the target)
FINE_TOL = 0.0005       # m


def line_up(base, gx, gy, gyaw=0.0):
    """goto (2 mm) + a slow fine translate to FINE_TOL; if that times out, keep the coarse pose."""
    base.goto(gx, gy, gyaw)
    x, y, yaw, _ = base.pose()
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = gx - x, gy - y
    try:
        base.translate(c * dx + s * dy, -s * dx + c * dy, vmax=0.01, tol=FINE_TOL, timeout_sim=20.0)
    except RuntimeError as e:
        log('fine line-up: %s - keeping the coarse pose' % e)
    return base.pose()


def _check(res, what):
    if res is None:
        raise RuntimeError(what + ' failed or timed out')
    return res


def pick(node, weld, cname, d=3.0):
    """mission.py's pick: open, above, descend, close 0.75 (+ check), attach confirmed, lift (+ check)."""
    pa, plan = arm_plan()
    _check(node.gripper(pa.open, d), 'open gripper')
    _check(node.arm(plan['pick_above'], d), 'move above container')
    _check(node.arm(plan['pick_descent'], d / 2), 'descend')
    _check(node.gripper(pa.carry_grip, d), 'close gripper')
    g = node.get(jp.GRIP_JOINTS)[0]
    log('gripper at %.3f rad' % g)
    if g < 0.35:
        raise RuntimeError('gripper closed almost fully: container missed')
    if g > 1.2:
        raise RuntimeError('gripper did not close (%.3f rad, open = %.2f)' % (g, pa.open))
    confirm(weld, 'attach', 'pick')
    _check(node.arm(plan['pick_lift'], d / 2), 'lift')
    c = gz_model_pose_full(cname)
    if c is None or c[0][2] < TABLE_Z + 0.05:
        raise RuntimeError('container not lifted with the gripper (attach failed?): %s' % (c,))
    log('lifted: %s z %.3f' % (cname, c[0][2]))
    return c


def release(node, weld, d=3.0, before_detach=None):
    """mission.py's place (arm part): above, descend, detach confirmed, open, retreat."""
    pa, plan = arm_plan()
    _check(node.arm(plan['place_above'], d), 'move above the surface')
    _check(node.arm(plan['place_descent'], d / 2), 'descend')
    if before_detach:
        before_detach()
    confirm(weld, 'detach', 'place')
    t = weld.sim_t
    _check(node.gripper(pa.open, d), 'open gripper')
    _check(node.arm(plan['place_retreat'], d / 2), 'retreat')
    return t


def poses_dict(poses):
    return {n: {'xyz': list(p[0]), 'quat_xyzw': list(p[1])} for n, p in poses.items()}


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--room', type=int, required=True)
    p.add_argument('--slot', required=True, help='pharmacy slot, e.g. S3')
    p.add_argument('--patient', default='')
    p.add_argument('--medicine', default=None, help='default: the inventory entry of the slot')
    p.add_argument('--dose', default=None)
    p.add_argument('--result-json', default=None)
    p.add_argument('--label', default='')
    p.add_argument('--fast-speed', type=float, default=0.0, help='route_follower --fast-speed (0 = off)')
    p.add_argument('--cap-check', action='store_true', help='check the cap colour before picking')
    p.add_argument('--offset', type=float, default=0.12)
    p.add_argument('--duration', type=float, default=3.0)
    p.add_argument('--lift-timeout', type=float, default=90.0)
    p.add_argument('--table-timeout', type=float, default=120.0)
    p.add_argument('--work-dir', default=None, help='route plans and follower results (default: next to the JSON)')
    a = p.parse_args()

    net = rp.Network()
    rooms = net.m['rooms']
    room = rooms.get(a.room) or rooms.get(str(a.room))
    if room is None:
        sys.exit('no room %s in the map' % a.room)
    pharm = ph.Pharmacy()
    if a.slot not in pharm.slots:
        sys.exit('no slot %s on the counter' % a.slot)
    inv = pharm.inv.get(a.slot) or {}
    medicine = a.medicine or inv.get('medicine', '?')
    dose = a.dose or inv.get('dose', '')
    cname = ph.container_name(a.slot)
    work = a.work_dir or (os.path.dirname(os.path.abspath(a.result_json)) if a.result_json else tempfile.mkdtemp())
    os.makedirs(work, exist_ok=True)
    tag = os.path.splitext(os.path.basename(a.result_json))[0] if a.result_json else 'delivery'

    pa, plan = arm_plan()
    d = a.duration

    rclpy.init()
    node = jp.PickPlace()
    base = BaseMover(node)
    weld = Weld(node, '/' + cname)
    table = tbl.TableMonitor(node, room['prefix'])
    contact_pose = ContactPose(table, cname)
    center = tuple(room['platform_center'])
    result = {}
    out = {'label': a.label, 'success': False, 'failed_step': None, 'error': None, 'steps': [],
           'wall_start_unix': time.time(),
           'delivery': {'room': a.room, 'slot': a.slot, 'container': cname, 'patient': a.patient,
                        'medicine': medicine, 'dose': dose, 'table': room['table'], 'table_prefix': room['prefix'],
                        'platform_center': list(center), 'cap': inv.get('cap')},
           'legs': {}}

    def check(res, what):
        if res is None:
            raise RuntimeError(what + ' failed or timed out')
        return res

    def table_top():
        table.spin_until(lambda: table.height is not None, 5.0, wall_limit=120.0)
        return table.height if table.height is not None else tbl.LOWERED_TOP

    def follow(src, dst, name):
        x, y, yaw, _ = base.pose()
        pl = net.plan(src, dst, math.degrees(yaw))
        pl['turn_round'] = False                    # done by the mission (steps 8, 13)
        pf = os.path.join(work, '%s_%s_plan.json' % (tag, name))
        rf = os.path.join(work, '%s_%s_route.json' % (tag, name))
        with open(pf, 'w') as fh:
            json.dump(pl, fh, indent=2)
        cmd = [sys.executable, '-u', os.path.join(HERE, 'route_follower.py'), '--plan', pf, '--result-json', rf]
        if a.fast_speed:
            cmd += ['--fast-speed', str(a.fast_speed)]
        log('route %s -> %s: %s' % (src, dst, ', '.join('%s %s' % (e['action'], e['node']) for e in pl['events'])))
        r = subprocess.run(cmd, text=True, capture_output=True)
        for l in [l for l in (r.stdout + r.stderr).splitlines() if 'ROUTE ERROR' in l or ' at J' in l
                  or 'Result' in l or 'Final' in l or re.search(r'\] (pass|straight|left|right|stop) ', l)]:
            log('  route_follower: ' + re.sub(r'\x1b\[[0-9;]*m', '', l).split('] ')[-1])
        leg = {}
        try:
            with open(rf) as fh:
                leg = json.load(fh)
        except (OSError, ValueError):
            pass
        leg['exit_code'] = r.returncode
        out['legs'][name] = leg
        if r.returncode != 0:
            raise RuntimeError('route %s -> %s: %s (exit %d)' % (src, dst, leg.get('error') or leg.get('result'),
                                                                 r.returncode))
        base.pose()             # fresh odometry after the subprocess

    def start_check():
        x, y, yaw, _ = base.pose()
        hx, hy, hyaw = net.m['home_pose']
        if math.hypot(x - hx, y - hy) > HOME_TOL[0] or abs(math.degrees(math.atan2(
                math.sin(yaw - hyaw), math.cos(yaw - hyaw)))) > HOME_TOL[1]:
            raise RuntimeError('robot not parked at home: (%.3f, %.3f, %.1f deg)' % (x, y, math.degrees(yaw)))
        poses = ph.gz_all_poses('med_')
        out['containers_start'] = poses_dict(poses)
        if not pharm.in_slot(a.slot, poses.get(cname)):
            raise RuntimeError('%s is not in its slot %s: %s' % (cname, a.slot, poses.get(cname)))
        log('start: robot at home, %s in slot %s; %d containers in the world' % (cname, a.slot, len(poses)))

    def align():
        sx, sy = pharm.slots[a.slot]
        x, y, yaw, _ = base.pose()
        log('pharmacy stop at (%.4f, %.4f, %.2f deg); slot %s at x %.3f -> move %+.3f m along the line'
            % (x, y, math.degrees(yaw), a.slot, sx, sx - x))
        out['pharmacy_stop'] = [x, y, math.degrees(yaw)]
        x, y, yaw, _ = line_up(base, sx, sy - pharm.m['pharmacy']['reach_left'], 0.0)
        out['at_slot'] = [x, y, math.degrees(yaw)]
        log('at slot %s: robot (%.4f, %.4f, %.2f deg)' % (a.slot, x, y, math.degrees(yaw)))

    def cap_check():
        from sensor_msgs.msg import Image
        import cv2
        import numpy as np
        look = [math.pi / 2] + jp.INIT_POSE[1:]
        check(node.arm(look, d), 'look at the container')
        t_ok = base.pose()[3]
        box = {}

        def on_img(m):
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            if t > t_ok + 0.3 and 'img' not in box:
                arr = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, :m.width * 3].reshape(m.height, m.width, 3)
                box['img'] = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR) if m.encoding == 'rgb8' else arr.copy()
        sub = node.create_subscription(Image, '/depth_cam/depth_cam', on_img, 2)
        end = time.monotonic() + 600
        while 'img' not in box and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.2)
        node.destroy_subscription(sub)
        if 'img' not in box:
            raise RuntimeError('no wrist image for the cap check')
        img = box['img']
        cv2.imwrite(os.path.join(work, '%s_cap_%s.png' % (tag, a.slot)), img)
        got, hsv = ph.classify_cap(img, ph.CAP_ROI)
        out['cap_check'] = {'expected': inv.get('cap'), 'seen': got, 'hsv': list(hsv)}
        log('cap check: expected %s, seen %s (HSV %s)' % (inv.get('cap'), got, hsv))
        if got != inv.get('cap'):
            raise RuntimeError('wrong medicine? cap %s, inventory says %s for %s' % (got, inv.get('cap'), a.slot))
        check(node.arm(jp.INIT_POSE, d), 'arm home after the cap check')

    def do_pick():
        pick(node, weld, cname, d)

    def check_on_table(target, where, z=TABLE_Z):
        c = gz_model_pose_full(cname)
        if c is None:
            raise RuntimeError('%s: container not found in Gazebo' % where)
        (cx, cy, cz), q = c
        dd = math.hypot(cx - target[0], cy - target[1])
        log('%s: %s (%.4f, %.4f, %.4f) tilt %.1f deg, %.1f mm from target' % (where, cname, cx, cy, cz, tilt_deg(q), 1000 * dd))
        if abs(cz - z) > 0.01 or dd > PLACE_TOL or tilt_deg(q) > 10:
            raise RuntimeError('%s: container is not standing on the table at the target (z %.3f, expected %.3f, '
                               '%.0f mm away, tilt %.0f deg)' % (where, cz, z, 1000 * dd, tilt_deg(q)))
        return c

    def place():
        if not table.wait_state('lowered', a.table_timeout, wall_limit=7200.0):
            raise RuntimeError('smart table %s not lowered (state %s, platform %s m) - not placing'
                               % (room['table'], table.state(), table.height))
        info = result['table'] = {
            'name': room['table'], 'prefix': room['prefix'],
            'platform': {'center': list(center), 'size': list(tbl.PLATFORM_SIZE),
                         'top_lowered': tbl.LOWERED_TOP, 'top_raised': tbl.RAISED_TOP},
            'state_before_place': table.state(), 'height_before_place': table_top()}
        x, y, yaw, _ = base.pose()
        out['room_stop'] = [x, y, math.degrees(yaw)]
        rx, ry = REACH_XY
        result['target'] = (x + math.cos(yaw) * rx - math.sin(yaw) * ry, y + math.sin(yaw) * rx + math.cos(yaw) * ry)
        info['detach_sim_t'] = release(node, weld, d, before_detach=contact_pose.arm)
        if not table.spin_until(lambda: contact_pose.thread is not None, 10.0, wall_limit=1800.0):
            log('WARNING: the platform contact sensor did not confirm the container within 10 sim s (contact %s)'
                % table.contact())
        placed = contact_pose.get()
        c = check_on_table(result['target'], 'after retreat')
        if not placed or not placed.get('xyz'):
            placed = {'source': 'after retreat (no contact confirmation)', 'sim_t': table.sim_t,
                      'xyz': list(c[0]), 'quat_xyzw': list(c[1])}
        placed['table_state'], placed['table_height'] = table.state(), table.height
        result['placed'] = placed
        px, py, pz = placed['xyz']
        log('placed (%s, sim t %.2f): %s (%.4f, %.4f, %.4f), %.1f mm from target, tilt %.1f deg, table %s at %.3f m'
            % (placed['source'], placed['sim_t'] or -1, cname, px, py, pz,
               1000 * math.hypot(px - result['target'][0], py - result['target'][1]),
               tilt_deg(placed['quat_xyzw']), placed['table_state'], placed['table_height'] or -1))

    def table_lift():
        info = result.setdefault('table', {})
        t0 = table.sim_t
        ok = table.spin_until(lambda: table.state() == 'raised', a.lift_timeout, wall_limit=7200.0,
                              every=(2.0, lambda: log('table: %s, platform %.3f m' % (table.state(), table.height or 0.0))))
        log('table: %s, platform %.3f m' % (table.state(), table.height or 0.0))
        t_place = (result.get('placed') or {}).get('sim_t') or info.get('detach_sim_t') or 0.0
        starts = [t for t, s in table.states if s == 'lifting' and t is not None and t >= t_place - 1.0]
        ends = [t for t, s in table.states if s == 'raised' and starts and t is not None and t >= starts[-1]]
        top = table_top()
        info.update(raised=bool(ok), lift_start_sim_t=starts[-1] if starts else None,
                    raised_sim_t=ends[0] if ends else None, height_after_lift=top,
                    states=[[t, s] for t, s in table.states])
        info['lift_time_s'] = (ends[0] - starts[-1]) if starts and ends else None
        info['wait_s'] = (ends[0] - t0) if ends else None
        c = gz_model_pose_full(cname)
        if c is not None:
            info['container_after_lift'] = {'xyz': list(c[0]), 'quat_xyzw': list(c[1])}
            on = tbl.on_platform(c[0], tilt_deg(c[1]), top, center=center)
            info['container_on_platform_after_lift'] = on
            log('table %s (platform %.3f m, lift %s s): %s (%.4f, %.4f, %.4f) tilt %.1f deg - %s'
                % (table.state(), top, '%.1f' % info['lift_time_s'] if info['lift_time_s'] else '?', cname,
                   c[0][0], c[0][1], c[0][2], tilt_deg(c[1]), 'on the platform' if on else 'NOT on the platform'))
        if not ok:
            log('WARNING: smart table not raised within %.0f sim s (state %s, platform %.3f m) - going on'
                % (a.lift_timeout, table.state(), top))

    def turn_round_room():
        if 'target' in result:
            check_on_table(result['target'], 'before moving away', z=table_top() + tbl.CONTAINER_HALF_H)
        base.turnaround(a.offset)

    def park():
        hx, hy, hyaw = net.m['home_pose']
        base.goto(hx, hy, hyaw)

    r = 'R%d' % a.room
    steps = [
        ('start check', start_check),
        ('arm home', lambda: check(node.arm(jp.INIT_POSE, d), 'arm home')),
        ('follow the line to the pharmacy', lambda: follow('home', 'pharmacy', 'to_pharmacy')),
        ('line up with slot %s' % a.slot, align),
        ('cap colour check', cap_check if a.cap_check else (lambda: log('cap check off'))),
        ('pick %s from slot %s' % (medicine, a.slot), do_pick),
        ('arm home (carrying)', lambda: check(node.arm(jp.INIT_POSE, d), 'arm home')),
        ('turn round at the pharmacy', lambda: base.turnaround(a.offset)),
        ('follow the line to Room %d' % a.room, lambda: follow('pharmacy', r, 'to_room')),
        ('place on the bedside table', place),
        ('arm home', lambda: check(node.arm(jp.INIT_POSE, d), 'arm home')),
        ('table lifts medicine to the patient', table_lift),
        ('turn round', turn_round_room),
        ('follow the line home', lambda: follow(r, 'home', 'to_home')),
        ('park at home', park),
    ]
    assert len(steps) == N_STEPS
    code = 1
    t_wall0 = time.monotonic()
    current = None
    try:
        node.wait_ready()
        t_start = base.pose()[3]
        out['sim_start'] = t_start
        log('Room %d | %s | %s %s | slot %s (%s)' % (a.room, a.patient or '-', medicine, dose, a.slot, cname))
        if a.label:
            log(a.label)
        for i, (label, fn) in enumerate(steps, 1):
            current = (i, label)
            x, y, yaw, t = base.pose()
            log('step %d/%d %s  (sim t %.1f, base %.3f %.3f %.1f deg)' % (i, len(steps), label, t, x, y, math.degrees(yaw)))
            out['steps'].append({'step': i, 'name': label, 'sim_t': t, 'base_before': [x, y, math.degrees(yaw)]})
            fn()
        x, y, yaw, t_end = base.pose()
        out.update(sim_time=t_end - t_start, base_final=[x, y, math.degrees(yaw)])
        log('done: base at %.4f %.4f yaw %.2f deg; cycle time %.1f sim s' % (x, y, math.degrees(yaw), t_end - t_start))
        out['success'] = True
        code = 0
    except KeyboardInterrupt:
        log('aborted')
        out['error'] = 'aborted'
    except RuntimeError as e:
        log('FAILED: %s' % e)
        out['error'] = str(e)
    finally:
        base.stop()
        if not out['success'] and current:
            out['failed_step'] = '%d %s' % current
        out['wall_time'] = time.monotonic() - t_wall0
        out['weld'] = WELD_LOG
        for k in ('placed', 'table'):
            if k in result:
                out[k] = result[k]
        try:
            poses = ph.gz_all_poses('med_')
            out['containers_final'] = poses_dict(poses)
        except Exception as e:      # noqa: BLE001 - keep the result
            log('container poses not read: %s' % e)
            poses = {}
        if 'target' in result:
            tx, ty = result['target']
            out['target'] = [tx, ty]
            c = poses.get(cname)
            if c is not None:
                (cx, cy, cz), q = c
                top = table.height if table.height is not None else tbl.LOWERED_TOP
                out.update(container=[cx, cy, cz], container_quat_xyzw=list(q),
                           placement_error_mm=1000 * math.hypot(cx - tx, cy - ty), tilt_deg=tilt_deg(q),
                           on_table=abs(cz - (top + tbl.CONTAINER_HALF_H)) < 0.01, platform_top_final=top)
        if a.result_json:
            with open(a.result_json, 'w') as fh:
                json.dump(out, fh, indent=2)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
