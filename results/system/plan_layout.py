#!/usr/bin/env python3
"""PHASE 0 plan of the hospital system (branch hospital-system) - nothing is built yet.

    python3 ~/sdp_sim/results/system/plan_layout.py

Draws results/system/layout.png (top view) and prints the routes with their
estimated sim / wall times. The room furniture comes straight from
tools/make_hospital_world.py (the tested single room, in its own coordinates);
every room is that room moved to a junction: north rooms unrotated, south rooms
rotated 180 deg about the junction, so the robot <-> smart-table geometry is the
same in all four rooms.

World frame: x east, y north, robot home pose (0, 0) facing +x (= spawn pose,
so /odom = world as now).
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..'))
sys.path.insert(0, os.path.join(ROOT, 'tools'))
import make_hospital_world as W                # noqa: E402  (room furniture, current coordinates)
tbl = W.tbl                                     # scripts/smart_table.py constants

# ------------------------------------------------------------------ layout (metres)
CORR_HALF = 0.60          # corridor clear half width (1.2 m corridor)
WALL_T = 0.10
J = {'J1': -0.75, 'J2': -3.65}                 # junction x on the main line (y = 0)
ROOMS = {'R1': ('J1', 'N'), 'R2': ('J1', 'S'), 'R3': ('J2', 'N'), 'R4': ('J2', 'S')}
HOME_POSE = (0.0, 0.0)                         # joint1, facing +x
HOME_MARK = -0.08                              # red, under the robot body when parked
PHARM_MARK = 0.55                              # red, end of the line
STOP_AHEAD = 0.2777                            # marker centre ahead of joint1 at a red stop (Gazebo)
MARKER = 0.15                                  # marker squares (as now)
N_SLOTS, SLOT_PITCH, REACH = 6, 0.10, W.REACH
P_STOP = PHARM_MARK - STOP_AHEAD
SLOTS = [P_STOP + (k - (N_SLOTS + 1) / 2) * SLOT_PITCH for k in range(1, N_SLOTS + 1)]
COUNTER = (SLOTS[0] - 0.08, SLOTS[-1] + 0.08, REACH - 0.03, REACH + 0.27)   # x0 x1 y0 y1, top 0.10 m
CORR_X0, CORR_X1 = J['J2'] - CORR_HALF, 1.55                                  # corridor inner faces
CAPS = [('white', (0.95, 0.95, 0.93)), ('yellow', (1.0, 0.86, 0.10)), ('amber', (1.0, 0.62, 0.05)),
        ('grey', (0.55, 0.56, 0.58)), ('black', (0.12, 0.12, 0.12)), ('white', (0.95, 0.95, 0.93))]
MEDS = ['Paracetamol 500 mg', 'Amoxicillin 250 mg', 'Ibuprofen 400 mg', 'Metformin 500 mg',
        'Omeprazole 20 mg', 'Paracetamol 500 mg']

# room shell in ROOM coordinates (= the current world's coordinates of the tested room)
R_FAR = -0.60             # inner face of the wall behind the far side of the bed (was -0.72)
R_NEAR = 2.20             # inner face of the wall on the robot's side (line at x = 1.70)
R_HEAD = 0.55             # inner face of the corridor wall (bed head end)
R_FOOT = W.WALL_Y         # 3.15, inner face behind the foot of the bed
DOOR = (W.XS - 0.40, W.XS + 0.40)
Y_OFF = CORR_HALF + WALL_T - R_HEAD            # room y -> distance from the corridor centre line


def room_tf(rid):
    """Returns f(x, y) room coordinates -> world, and the room's yaw."""
    jx = J[ROOMS[rid][0]]
    if ROOMS[rid][1] == 'N':
        return (lambda x, y: (jx + (x - W.XS), y + Y_OFF)), 0.0
    return (lambda x, y: (jx - (x - W.XS), -(y + Y_OFF))), math.pi


# ------------------------------------------------------------------ camera model (wrist camera, home pose)
CAM_H, CAM_PITCH, CAM_AHEAD = 0.396, math.radians(54.1), 0.062   # tf2_echo + strips 0.19-0.27 m
IMG_W, IMG_H, HFOV = 320, 200, 1.047
F_PX = (IMG_W / 2) / math.tan(HFOV / 2)


def ground_point(u, v):
    """Pixel (u, v) -> floor point (ahead, left) of joint1."""
    a = CAM_PITCH + math.atan((v - IMG_H / 2) / F_PX)
    d = CAM_H / math.tan(a)
    zc = math.hypot(CAM_H, d) * math.cos(a - CAM_PITCH)
    return CAM_AHEAD + d, -(u - IMG_W / 2) / F_PX * zc


# ------------------------------------------------------------------ routes + time model
SEC_PER_M = 39.0 / 2.957      # measured: 2.957 m in 39.0 sim s (incl. start and the old arc)
T = {'junction_turn': 7.0, 'pick': 20.0, 'arm_home': 3.6, 'turnaround': 16.4, 'place': 16.4,
     'lift': 18.3, 'cap_check': 7.2}
BRANCH_STOP = W.STOP_Y + Y_OFF           # junction -> joint1 at the room stop
BRANCH_MARK = W.Y_END + Y_OFF            # junction -> room marker
HOME_STOP = HOME_MARK - STOP_AHEAD       # joint1 when the home marker stops the robot (eastbound)


def delivery(rid, slot=None):
    jx = J[ROOMS[rid][0]]
    xs = P_STOP if slot is None else SLOTS[slot - 1]
    side = ROOMS[rid][1]
    out_turn = 'right' if side == 'N' else 'left'      # robot faces west on the corridor
    back_turn = 'left' if side == 'N' else 'right'     # robot faces south / north on the branch
    legs = []
    legs.append(('home -> pharmacy', P_STOP - HOME_POSE[0], 0,
                 ['follow +x', 'stop at red P (%.2f m)' % (P_STOP - HOME_POSE[0])]))
    ev2 = ['turn round (strafe R 0.12, rotate 180, strafe R 0.12)']
    if xs - HOME_MARK > STOP_AHEAD + 0.08:
        ev2.append('pass red HOME (at %.2f m)' % (xs - HOME_MARK - STOP_AHEAD))
    if rid in ('R3', 'R4'):
        ev2.append('J1 straight (%.2f m)' % (xs - J['J1']))
    ev2.append('%s %s 90 deg (%.2f m)' % (ROOMS[rid][0], out_turn, xs - jx))
    ev2.append('stop at red %s (%.2f m)' % (rid, xs - jx + BRANCH_STOP))
    legs.append(('pharmacy -> %s' % rid, xs - jx + BRANCH_STOP, 1, ev2))
    ev3 = ['turn round', '%s %s 90 deg (%.2f m)' % (ROOMS[rid][0], back_turn, BRANCH_STOP)]
    if rid in ('R3', 'R4'):
        ev3.append('J1 straight (%.2f m)' % (BRANCH_STOP + J['J1'] - jx))
    ev3.append('stop at red HOME (%.2f m), park: +%.2f m on /odom' % (BRANCH_STOP + HOME_STOP - jx, -HOME_STOP))
    legs.append(('%s -> home' % rid, BRANCH_STOP + HOME_STOP - jx, 1, ev3))
    align = abs(xs - P_STOP) if slot else 0.15         # average slot offset
    t = {
        'leg1 line': legs[0][1] * SEC_PER_M,
        'align to slot': 1.0 + align / 0.05,
        'pick + arm home': T['pick'] + T['arm_home'],
        'turn round (pharmacy)': T['turnaround'],
        'leg2 line + junction': legs[1][1] * SEC_PER_M + T['junction_turn'],
        'place + arm home + lift': T['place'] + T['arm_home'] + T['lift'],
        'turn round (room)': T['turnaround'],
        'leg3 line + junction': legs[2][1] * SEC_PER_M + T['junction_turn'],
        'park': 1.0 + (-HOME_STOP) / 0.05,
    }
    return legs, t


def report():
    print('slots (x at y=%.2f): %s' % (REACH, ', '.join('S%d %.3f' % (i + 1, x) for i, x in enumerate(SLOTS))))
    print('pharmacy stop %.3f, home stop %.3f, branch: junction->stop %.3f, ->marker %.3f'
          % (P_STOP, HOME_STOP, BRANCH_STOP, BRANCH_MARK))
    tot = 0.0
    rows = []
    for rid in ROOMS:
        legs, t = delivery(rid)
        s = sum(t.values())
        tot += s
        rows.append((rid, legs, t, s))
        print('\n%s  (sim %.0f s; wall %.0f min at 5.5 %% RTF, %.0f min at 4.5 %%)' % (rid, s, s / 0.055 / 60, s / 0.045 / 60))
        for name, dist, nj, ev in legs:
            print('   %-18s %5.2f m  %s' % (name, dist, ' | '.join(ev)))
        print('   ' + ', '.join('%s %.1f' % kv for kv in t.items()))
    print('\ndemo (R1..R4): sim %.0f s = %.1f min; wall %.1f h at 5.5 %%, %.1f h at 4.5 %%'
          % (tot, tot / 60, tot / 0.055 / 3600, tot / 0.045 / 3600))
    longest = max(l[1] for r in rows for l in r[1])
    print('longest one-way line route: %.2f m' % longest)
    return rows, tot


# ------------------------------------------------------------------ drawing
def draw(path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Polygon, Rectangle

    fig = plt.figure(figsize=(17, 12.4), dpi=105)
    ax = fig.add_axes([0.03, 0.215, 0.70, 0.75])
    axz = fig.add_axes([0.745, 0.47, 0.245, 0.49])          # pharmacy zoom
    axt = fig.add_axes([0.03, 0.005, 0.96, 0.155])
    axt.axis('off')

    WALLC, LINEC, REDC, GRNC = '#555a60', '#1a3fe0', '#e01f1f', '#19b33a'

    def rect(a, x0, x1, y0, y1, **kw):
        a.add_patch(Rectangle((min(x0, x1), min(y0, y1)), abs(x1 - x0), abs(y1 - y0), **kw))

    def poly(a, pts, **kw):
        a.add_patch(Polygon(pts, closed=True, **kw))

    def room_rect(a, f, x0, x1, y0, y1, **kw):
        poly(a, [f(x0, y0), f(x1, y0), f(x1, y1), f(x0, y1)], **kw)

    def furniture(a, rid, labels=True):
        f, yaw = room_tf(rid)
        # beige floor patch under the bed area (as in the tested room)
        room_rect(a, f, R_FAR, W.BED_X1 + 0.045, R_HEAD, R_FOOT, fc='#e8dfcc', ec='none', zorder=1)
        # walls: far, foot, near, head with the door
        for x0, x1, y0, y1 in ((R_FAR - WALL_T, R_FAR, R_HEAD - WALL_T, R_FOOT + WALL_T),
                               (R_FAR - WALL_T, R_NEAR + WALL_T, R_FOOT, R_FOOT + WALL_T),
                               (R_NEAR, R_NEAR + WALL_T, R_HEAD - WALL_T, R_FOOT + WALL_T),
                               (R_FAR - WALL_T, DOOR[0], R_HEAD - WALL_T, R_HEAD),
                               (DOOR[1], R_NEAR + WALL_T, R_HEAD - WALL_T, R_HEAD)):
            room_rect(a, f, x0, x1, y0, y1, fc=WALLC, ec='none', zorder=3)
        # bed: frame, mattress, blanket, pillow, headboard, rails
        room_rect(a, f, W.BED_X0 - 0.005, W.BED_X1 + 0.005, W.BED_Y0 - 0.01, W.BED_Y1 + 0.01, fc='#b8bcc2', ec='#6b7078', lw=0.8, zorder=4)
        room_rect(a, f, W.BED_X0, W.BED_X1, W.BED_Y0, W.BED_Y1, fc='#f4f4f2', ec='#8a8f96', lw=0.6, zorder=5)
        room_rect(a, f, W.BED_X0 - 0.02, W.BED_X1 + 0.02, W.HINGE_Y + 0.04, W.BED_Y1 - 0.03, fc='#5c8c80', ec='none', zorder=6)
        room_rect(a, f, W.BED_CX - 0.31, W.BED_CX + 0.31, W.BED_Y0 + 0.08, W.BED_Y0 + 0.36, fc='#dcdcd8', ec='#9a9a96', lw=0.5, zorder=6)
        room_rect(a, f, W.BED_X0 - 0.03, W.BED_X1 + 0.03, W.BED_Y0 - 0.045, W.BED_Y0 - 0.015, fc='#c9ad85', ec='none', zorder=6)
        room_rect(a, f, W.BED_X0 - 0.03, W.BED_X1 + 0.03, W.BED_Y1 + 0.015, W.BED_Y1 + 0.045, fc='#c9ad85', ec='none', zorder=6)
        for x, y0, y1 in ((W.BED_X0 - 0.025, W.BED_Y0 + 0.06, W.BED_Y0 + 0.58), (W.BED_X0 - 0.025, W.BED_Y0 + 0.92, W.BED_Y0 + 1.72),
                          (W.BED_X1 + 0.025, W.BED_Y0 + 0.06, W.BED_Y0 + 0.58)):
            a.plot(*zip(f(x, y0), f(x, y1)), color='#7d838b', lw=1.6, zorder=7, solid_capstyle='butt')
        # smart table: base + platform + place target
        bx1 = tbl.PLATFORM_ROBOT_EDGE_X
        cy = tbl.PLATFORM_CENTER[1]
        room_rect(a, f, bx1 - W.TABLE_BASE[0], bx1, cy - W.TABLE_BASE[1] / 2, cy + W.TABLE_BASE[1] / 2, fc='#efefea', ec='#8c9096', lw=0.7, zorder=8)
        room_rect(a, f, bx1 - tbl.PLATFORM_SIZE[0], bx1, cy - tbl.PLATFORM_SIZE[1] / 2, cy + tbl.PLATFORM_SIZE[1] / 2, fc='#c8cbd0', ec='#3d4248', lw=1.0, zorder=9)
        tx, ty = f(W.XS - REACH, W.STOP_Y)
        a.plot(tx, ty, marker='x', color='#222', ms=4, mew=1.0, zorder=10)
        # IV pole, monitor, chair
        for (x, y), r, c in ((W.IV_XY, 0.25, '#9aa0a8'), (W.MONITOR_XY, 0.24, '#9aa0a8')):
            a.add_patch(Circle(f(x, y), r, fc='none', ec=c, lw=0.8, zorder=6))
            a.add_patch(Circle(f(x, y), 0.03, fc='#6b7078', ec='none', zorder=6))
        mx, my = W.MONITOR_XY
        room_rect(a, f, mx + 0.03, mx + 0.115, my - 0.19, my + 0.19, fc='#25272a', ec='none', zorder=7)
        cx, cy2 = W.CHAIR_XY
        cw = W.CHAIR_YAW
        pts = []
        for lx, ly in ((-0.28, -0.30), (0.24, -0.30), (0.24, 0.30), (-0.28, 0.30)):
            pts.append(f(cx + lx * math.cos(cw) - ly * math.sin(cw), cy2 + lx * math.sin(cw) + ly * math.cos(cw)))
        poly(a, pts, fc='#6e8a88', ec='#3f5250', lw=0.7, zorder=6)
        # window (foot wall) and room sign next to the door (corridor side)
        room_rect(a, f, 1.70 - 0.475, 1.70 + 0.475, R_FOOT + 0.02, R_FOOT + 0.08, fc='#cfe3df', ec='none', zorder=4)
        room_rect(a, f, DOOR[1] + 0.02, DOOR[1] + 0.07, R_HEAD - WALL_T - 0.03, R_HEAD - WALL_T, fc='#f2f2f2', ec='#333', lw=0.6, zorder=5)
        if labels:
            lx, ly = f((R_FAR + R_NEAR) / 2 - 0.35, (R_HEAD + R_FOOT) / 2 + 0.95 * (1 if yaw == 0 else 1))
            a.text(*f(0.45, 2.95 if yaw == 0 else 2.95), 'Room %s' % rid[1], fontsize=13, weight='bold', ha='center', va='center', zorder=20)
        return f

    def robot(a, x, y, yaw, **kw):
        c, s = math.cos(yaw), math.sin(yaw)
        pts = [(x + c * px - s * py, y + s * px + c * py) for px, py in ((-0.237, -0.13), (0.073, -0.13), (0.073, 0.13), (-0.237, 0.13))]
        poly(a, pts, fc=kw.get('fc', '#ffb347'), ec='#a0522d', lw=1.0, zorder=30, alpha=kw.get('alpha', 1.0))
        a.annotate('', xy=(x + 0.22 * c, y + 0.22 * s), xytext=(x, y), arrowprops=dict(arrowstyle='-|>', color='#8b3a00', lw=1.4), zorder=31)
        a.plot(x, y, 'o', color='#8b3a00', ms=3, zorder=32)

    def lines(a):
        lw = 2.6 if a is ax else 6
        a.plot([PHARM_MARK + MARKER / 2, J['J2']], [0, 0], color=LINEC, lw=lw, solid_capstyle='butt', zorder=12)
        for rid, (jn, side) in ROOMS.items():
            sgn = 1 if side == 'N' else -1
            a.plot([J[jn], J[jn]], [0, sgn * (BRANCH_MARK + MARKER / 2)], color=LINEC, lw=lw, solid_capstyle='butt', zorder=12)
            rect(a, J[jn] - MARKER / 2, J[jn] + MARKER / 2, sgn * BRANCH_MARK - MARKER / 2, sgn * BRANCH_MARK + MARKER / 2, fc=REDC, ec='none', zorder=13)
        for jn, x in J.items():
            rect(a, x - MARKER / 2, x + MARKER / 2, -MARKER / 2, MARKER / 2, fc=GRNC, ec='none', zorder=13)
        for x in (HOME_MARK, PHARM_MARK):
            rect(a, x - MARKER / 2, x + MARKER / 2, -MARKER / 2, MARKER / 2, fc=REDC, ec='none', zorder=13)

    def pharmacy(a, zoom=False):
        # pharmacy room (north side, east end) - walls; its corridor side is the cabinet unit
        x0, x1 = J['J1'] + CORR_HALF - 0.05 + WALL_T / 2, CORR_X1   # = R1 near wall outer face
        x0 = J['J1'] + 0.50 + WALL_T
        y0, y1 = CORR_HALF + WALL_T, R_FOOT + Y_OFF
        rect(a, x0, x1 + WALL_T, y1, y1 + WALL_T, fc=WALLC, ec='none', zorder=3)
        rect(a, x1, x1 + WALL_T, y0 - WALL_T, y1 + WALL_T, fc=WALLC, ec='none', zorder=3)
        rect(a, COUNTER[1], x1, y0 - WALL_T, y0, fc=WALLC, ec='none', zorder=3)
        rect(a, x0, COUNTER[0], y0 - WALL_T, y0, fc=WALLC, ec='none', zorder=3)
        # tall medication cabinet (shelves with boxes, visual only) behind the 0.10 m pick counter
        rect(a, COUNTER[0], COUNTER[1], COUNTER[3], COUNTER[3] + 0.33, fc='#d9dde3', ec='#6b7078', lw=0.8, hatch='////', zorder=8)
        rect(a, COUNTER[0], COUNTER[1], COUNTER[2], COUNTER[3], fc='#e6e6e2', ec='#5a5f66', lw=0.9, zorder=9)
        for i, x in enumerate(SLOTS):
            name, rgb = CAPS[i]
            a.add_patch(Circle((x, REACH), 0.016, fc=rgb, ec='#222', lw=0.6, zorder=15))
            if zoom:
                a.text(x, REACH + 0.03, 'S%d' % (i + 1), fontsize=6.5, ha='center', va='bottom', zorder=16)
        if zoom:
            a.text((COUNTER[0] + COUNTER[1]) / 2, COUNTER[3] + 0.165, 'medication cabinet\n(shelves + boxes, visual)', fontsize=6.5,
                   ha='center', va='center', zorder=16, bbox=dict(fc='white', ec='none', alpha=0.85, pad=0.6))
            a.text((COUNTER[0] + COUNTER[1]) / 2, COUNTER[2] + 0.20, 'pick counter, top 0.10 m', fontsize=6.3, ha='center', va='center', zorder=16)
            a.text(0.80, 0.93, 'PHARMACY', fontsize=10, weight='bold', ha='center', zorder=20)
        else:
            a.text((x0 + x1) / 2 + 0.1, y0 + 1.0, 'PHARMACY', fontsize=13, weight='bold', ha='center', zorder=20)
        if not zoom:
            a.text((x0 + x1) / 2 + 0.1, y0 + 0.75, 'cabinet + pick counter\n(6 slots, top 0.10 m)', fontsize=7.5, ha='center', zorder=20)

    def corridor(a):
        # corridor walls where no room / pharmacy forms them: west end, east end
        rect(a, CORR_X0 - WALL_T, CORR_X0, -CORR_HALF - WALL_T, CORR_HALF + WALL_T, fc=WALLC, ec='none', zorder=3)
        rect(a, CORR_X1, CORR_X1 + WALL_T, -CORR_HALF - WALL_T, CORR_HALF, fc=WALLC, ec='none', zorder=3)
        # south side east of R2: R2 reaches x = J1 + 2.40 -> exterior wall piece to the corridor end
        r2_x1 = J['J1'] + (W.XS - R_FAR) + WALL_T
        if CORR_X1 > r2_x1:
            rect(a, r2_x1, CORR_X1, -CORR_HALF - WALL_T, -CORR_HALF, fc=WALLC, ec='none', zorder=3)

    for a in (ax, axz):
        a.set_facecolor('#d9d9d6')
        rect(a, CORR_X0, CORR_X1, -CORR_HALF, CORR_HALF, fc='white', ec='none', zorder=0)
        for rid in ROOMS:
            f, _ = room_tf(rid)
            room_rect(a, f, R_FAR, R_NEAR, R_HEAD, R_FOOT, fc='white', ec='none', zorder=0)
            room_rect(a, f, DOOR[0], DOOR[1], R_HEAD - WALL_T, R_HEAD, fc='white', ec='none', zorder=0)
        rect(a, J['J1'] + 0.60, CORR_X1, CORR_HALF, R_FOOT + Y_OFF, fc='white', ec='none', zorder=0)
        corridor(a)
        for rid in ROOMS:
            furniture(a, rid, labels=(a is ax))
        pharmacy(a, zoom=(a is axz))
        lines(a)

    # robot at home (both panels); zoom: camera view, P stop, slot range, turn-round circles
    robot(ax, 0, 0, 0)
    robot(axz, 0, 0, 0)
    fp = [ground_point(u, v) for u, v in ((0, IMG_H), (IMG_W, IMG_H), (IMG_W, 0), (0, 0))]
    poly(axz, fp, fc='#7ab8ff', ec='#2a6fd6', alpha=0.15, lw=0.8, zorder=25)
    for y0f, y1f in ((0.90, 0.95), (0.80, 0.85), (0.70, 0.75)):
        st = [ground_point(u, v) for u, v in ((0, y1f * IMG_H), (IMG_W, y1f * IMG_H), (IMG_W, y0f * IMG_H), (0, y0f * IMG_H))]
        poly(axz, st, fc='#2a6fd6', ec='none', alpha=0.35, zorder=26)
    axz.annotate('wrist-camera view from home\n(dark bands = steering strips)', xy=ground_point(IMG_W * 0.75, 20),
                 xytext=(0.55, 0.80), fontsize=6.5, color='#1d4f9c', ha='center',
                 arrowprops=dict(arrowstyle='->', color='#1d4f9c', lw=0.7), zorder=40)
    # pharmacy stop: outline only
    c = [(P_STOP + px, py) for px, py in ((-0.237, -0.13), (0.073, -0.13), (0.073, 0.13), (-0.237, 0.13))]
    poly(axz, c, fc='none', ec='#a0522d', ls='--', lw=1.0, zorder=33)
    axz.plot(P_STOP, 0, 'x', color='#a0522d', ms=6, zorder=33)
    axz.annotate('pharmacy stop (joint1 %.2f m\nbefore the P marker)' % STOP_AHEAD, xy=(P_STOP, -0.02),
                 xytext=(0.74, -0.45), fontsize=6.5, color='#8b3a00', ha='center',
                 arrowprops=dict(arrowstyle='->', color='#8b3a00', lw=0.7), zorder=40)
    axz.plot([SLOTS[0], SLOTS[-1]], [-0.20, -0.20], color='#8b3a00', lw=1.0, zorder=24)
    for x in SLOTS:
        axz.plot([x, x], [-0.215, -0.185], color='#8b3a00', lw=0.8, zorder=24)
    axz.text((SLOTS[0] + SLOTS[-1]) / 2, -0.255, 'joint1 at slot k: forward/back on /odom, pick at (0, 0.20)',
             fontsize=6, ha='center', va='top', color='#8b3a00', zorder=40)
    for x in (SLOTS[0], SLOTS[-1]):
        axz.add_patch(Circle((x, -W.TURN_OFFSET), W.TURN_R, fc='none', ec='#888', ls='--', lw=0.7, zorder=24))
    axz.text(-0.05, -0.56, 'dashed: turn-round circles at S1 / S6', fontsize=6, color='#555', ha='center')
    for x, lab in ((HOME_MARK, 'HOME'), (PHARM_MARK, 'P')):
        axz.text(x, 0.085, lab, fontsize=7, ha='center', va='bottom', color=REDC, weight='bold', zorder=40)
    axz.annotate('robot at home = start\n(0, 0) facing +x', xy=(-0.12, 0.0), xytext=(-0.62, 0.42), fontsize=6.5,
                 color='#8b3a00', ha='center', arrowprops=dict(arrowstyle='->', color='#8b3a00', lw=0.7), zorder=40)
    axz.text(J['J1'] + 0.09, -0.10, 'J1', fontsize=8, ha='left', va='top', color='#0d7a26', weight='bold', zorder=40)
    axz.set_xlim(-1.0, 1.05)
    axz.set_ylim(-0.62, 1.05)
    axz.set_aspect('equal')
    axz.set_title('pharmacy end (zoom)', fontsize=10)
    axz.tick_params(labelsize=7)
    # inventory table under the zoom
    axi = fig.add_axes([0.745, 0.215, 0.245, 0.22])
    axi.axis('off')
    inv = [['S%d' % (i + 1), MEDS[i], CAPS[i][0], '1', '%.3f' % x] for i, x in enumerate(SLOTS)]
    ti = axi.table(cellText=inv, colLabels=['slot', 'medicine + dose', 'cap', 'qty', 'x (m)'], loc='upper center',
                   cellLoc='left', colWidths=[0.11, 0.47, 0.15, 0.09, 0.15])
    ti.auto_set_font_size(False)
    ti.set_fontsize(7.5)
    ti.scale(1, 1.3)
    for i in range(N_SLOTS):
        ti[i + 1, 2].set_facecolor(CAPS[i][1] if CAPS[i][0] not in ('black', 'grey') else '#cfcfcf')
    axi.set_title('inventory (plan): 6 slots, 0.10 m apart, y = 0.20, counter top 0.10 m\n'
                  'containers med_S1..med_S6: same 3 x 6 cm, 30 g; cap colour + label texture', fontsize=7.5)

    for jn, x in J.items():
        ax.add_patch(Circle((x, 0), W.TURN_R, fc='none', ec='#888', ls='--', lw=0.7, zorder=24))
        ax.text(x + 0.10, -0.12, jn, fontsize=11, color='#0d7a26', weight='bold', va='top', zorder=40)
    for rid, (jn, side) in ROOMS.items():
        sgn = 1 if side == 'N' else -1
        x = J[jn]
        heading = math.pi / 2 if side == 'N' else -math.pi / 2
        robot(ax, x, sgn * BRANCH_STOP, heading, fc='#ffe0b0', alpha=0.8)
        ax.add_patch(Circle((x + sgn * W.TURN_OFFSET, sgn * BRANCH_STOP), W.TURN_R, fc='none', ec='#888', ls='--', lw=0.7, zorder=24))
        ax.text(x - sgn * 0.08, sgn * (BRANCH_MARK + 0.15), rid, fontsize=10, color=REDC, weight='bold', ha='center', va='center', zorder=40)
    ax.text(HOME_MARK, -0.16, 'HOME', fontsize=8, color=REDC, weight='bold', ha='center', va='top', zorder=40)
    ax.text(PHARM_MARK + 0.02, -0.16, 'P', fontsize=9, color=REDC, weight='bold', ha='center', va='top', zorder=40)

    # dimensions
    def dim(a, p0, p1, txt, off=(0, 0), **kw):
        a.annotate('', xy=p1, xytext=p0, arrowprops=dict(arrowstyle='<->', color='#444', lw=0.8), zorder=41)
        a.text((p0[0] + p1[0]) / 2 + off[0], (p0[1] + p1[1]) / 2 + off[1], txt, fontsize=8, ha='center', va='center',
               color='#222', zorder=41, bbox=dict(fc='white', ec='none', alpha=0.8, pad=0.5), **kw)
    dim(ax, (J['J2'], -0.42), (J['J1'], -0.42), '%.2f m' % (J['J1'] - J['J2']))
    dim(ax, (J['J1'], -0.42), (HOME_MARK, -0.42), '%.2f' % (HOME_MARK - J['J1']))
    dim(ax, (HOME_MARK, -0.42), (PHARM_MARK, -0.42), '%.2f' % (PHARM_MARK - HOME_MARK))
    dim(ax, (J['J2'] + 0.42, 0), (J['J2'] + 0.42, BRANCH_MARK), '%.2f m\nto marker' % BRANCH_MARK, off=(0.33, 0.25))
    dim(ax, (CORR_X1 + 0.25, -CORR_HALF), (CORR_X1 + 0.25, CORR_HALF), '1.2 m\ncorridor', off=(0.27, 0))
    x_w = J['J1'] - (W.XS - R_FAR) - WALL_T
    dim(ax, (x_w, 3.55), (J['J1'] + R_NEAR - W.XS + WALL_T, 3.55), 'room %.2f x %.2f m (inside)' % (R_NEAR - R_FAR, R_FOOT - R_HEAD))
    ax.text(J['J1'] - 2.2, 0.32, 'corridor (blue main line, y = 0)', fontsize=9, color='#333', zorder=40)
    ax.annotate('robot start / home\n(0, 0) facing +x', xy=(0.0, 0.0), xytext=(0.25, -1.25), fontsize=9, color='#8b3a00',
                arrowprops=dict(arrowstyle='->', color='#8b3a00'), zorder=42)
    ax.set_xlim(-6.35, 2.15)
    ax.set_ylim(-3.75, 3.75)
    ax.set_aspect('equal')
    ax.grid(True, color='#bbb', lw=0.3, zorder=-1)
    ax.tick_params(labelsize=8)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title('Hospital system - top view (plan, branch hospital-system): corridor, pharmacy, Rooms 1-4, line network', fontsize=12)

    # legend
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch
    h = [Line2D([], [], color=LINEC, lw=2.6, label='blue line (3 cm tape)'),
         Patch(fc=REDC, label='red stop marker 15 cm (rooms, pharmacy, home)'),
         Patch(fc=GRNC, label='green junction marker 15 cm (J1, J2)'),
         Patch(fc='#c8cbd0', ec='#3d4248', label='smart table platform 25x25 cm (x = place target)'),
         Patch(fc='#ffb347', ec='#a0522d', label='robot at home (start)'),
         Patch(fc='#ffe0b0', ec='#a0522d', label='robot at a stop (room / pharmacy)'),
         Line2D([], [], color='#888', ls='--', lw=0.8, label='0.27 m turning circles (junction, turn round)'),
         Patch(fc='#f2f2f2', ec='#333', label='room sign "Room N" (texture) next to each door')]
    ax.legend(handles=h, loc='lower left', fontsize=7.5, framealpha=0.92)

    # route table
    rows, tot = report_rows()
    col = ['delivery', 'leg 1 home->P', 'leg 2 P->room (actions)', 'leg 3 room->home (actions)', 'sim s', 'wall @5.5%', 'wall @4.5%']
    cells = []
    for rid, legs, t, s in rows:
        cells.append(['Room %s' % rid[1], '%.2f m' % legs[0][1],
                      '%.2f m: %s' % (legs[1][1], ', '.join(e.split(' (')[0] for e in legs[1][3][1:])),
                      '%.2f m: %s' % (legs[2][1], ', '.join(e.split(' (')[0].split(',')[0] for e in legs[2][3][1:])),
                      '%.0f' % s, '%.0f min' % (s / 0.055 / 60), '%.0f min' % (s / 0.045 / 60)])
    cells.append(['demo R1-R4', '', '', '', '%.0f' % tot, '%.1f h' % (tot / 0.055 / 3600), '%.1f h' % (tot / 0.045 / 3600)])
    tb = axt.table(cellText=cells, colLabels=col, loc='center', cellLoc='left',
                   colWidths=[0.07, 0.07, 0.36, 0.30, 0.05, 0.07, 0.07])
    tb.auto_set_font_size(False)
    tb.set_fontsize(7.6)
    tb.scale(1, 1.25)
    axt.set_title('Routes (every delivery: home -> pharmacy slot -> room -> home; pick at arm-relative (0, 0.20) as tested) - '
                  'times = measured step times of the 2026-11-05 missions', fontsize=8.5, loc='left')
    fig.savefig(path)
    print('wrote', path)


def report_rows():
    rows, tot = [], 0.0
    for rid in ROOMS:
        legs, t = delivery(rid)
        s = sum(t.values())
        tot += s
        rows.append((rid, legs, t, s))
    return rows, tot


def check():
    """Plan-level checks (the generator will repeat them on the real geometry)."""
    probs = []
    # longest one-way line route
    rows, _ = report_rows()
    longest = max(l[1] for r in rows for l in r[1])
    if longest > 6.0:
        probs.append('longest route %.2f m > 6 m' % longest)
    # turning circles vs walls / tables (room coordinates; same in every room)
    tc = (W.XS + W.TURN_OFFSET, W.STOP_Y)
    if R_NEAR - tc[0] < W.TURN_R + 0.015:
        probs.append('room turn-round circle hits the near wall')
    if tc[0] - tbl.PLATFORM_ROBOT_EDGE_X < W.TURN_R + 0.015 - 1e-9:
        probs.append('room turn-round circle hits the table')
    # chair stays inside the moved far wall
    cw = W.CHAIR_YAW
    xmin = min(W.CHAIR_XY[0] + lx * math.cos(cw) - ly * math.sin(cw) for lx, ly in ((-0.28, -0.30), (0.24, -0.30), (0.24, 0.30), (-0.28, 0.30)))
    if xmin - R_FAR < 0.05:
        probs.append('chair %.3f m from the far wall' % (xmin - R_FAR))
    # pharmacy turn-round circle vs counter edge, corridor wall
    if COUNTER[2] - (-W.TURN_OFFSET) < W.TURN_R + 0.015 - 1e-9:
        probs.append('pharmacy turn-round circle hits the counter')
    if CORR_HALF - W.TURN_OFFSET < W.TURN_R + 0.015:
        probs.append('pharmacy turn-round circle hits the south wall')
    # junction circles inside the corridor
    if CORR_HALF < W.TURN_R + 0.015:
        probs.append('junction circle hits the corridor walls')
    # marker spacing along every route >= 0.6 m
    xs = sorted([HOME_MARK, PHARM_MARK] + list(J.values()))
    if min(b - a for a, b in zip(xs, xs[1:])) < 0.6:
        probs.append('markers on the main line closer than 0.6 m')
    print('marker spacing on the main line: %s' % ', '.join('%.2f' % (b - a) for a, b in zip(xs, xs[1:])))
    # rooms do not overlap (north: R3 | R1 | pharmacy, south: R4 | R2)
    n_r1 = (J['J1'] - (W.XS - R_FAR) - WALL_T, J['J1'] + R_NEAR - W.XS + WALL_T)
    n_r3 = (J['J2'] - (W.XS - R_FAR) - WALL_T, J['J2'] + R_NEAR - W.XS + WALL_T)
    if n_r3[1] > n_r1[0] + WALL_T + 1e-9:
        probs.append('R3 overlaps R1')
    print('plan checks:', 'OK' if not probs else probs)
    print('rooms north R3 %.2f..%.2f, R1 %.2f..%.2f (outer faces); camera footprint at home: %s'
          % (n_r3 + n_r1 + (', '.join('(%.2f, %.2f)' % ground_point(u, v) for u, v in ((0, IMG_H), (IMG_W, 0))),)))
    return probs


if __name__ == '__main__':
    report()
    check()
    draw(os.path.join(HERE, 'layout.png'))
