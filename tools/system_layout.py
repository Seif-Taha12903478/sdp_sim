#!/usr/bin/env python3
"""Layout of the hospital system (branch hospital-system): ONE source for the numbers.

    python3 ~/sdp_sim/tools/system_layout.py      # checks + route table + results/system/layout.png

Used by tools/make_system_world.py (world, map YAML, container models), the planner
(through config/hospital_map.yaml) and the offline line simulation.

World frame: x east, y north. The robot's home = spawn pose (0, 0) facing +x, so /odom
equals the world frame (as in the single-room world).

  - corridor 1.2 m wide along y = 0 with the blue main line
  - junction J1 (4-way: Room 1 north, Room 2 south) and J2 (T: Room 3 north, Room 4 south)
  - every room is the TESTED room of tools/make_hospital_world.py, in its own ("room")
    coordinates, moved to its junction: north rooms unrotated, south rooms rotated
    180 deg about the junction. So the robot <-> smart-table geometry is the same in all
    four rooms (red stop marker, stop 0.2777 m before it, place target 0.20 m left,
    platform edge 0.17 m left, turn-round circle 0.29 m from the table).
  - pharmacy at the east end, north side: cabinet behind a pick counter (top 0.10 m,
    front edge 0.17 m left of the line, like the old medicine station), 6 slots 0.10 m
    apart at y = 0.20 -> arm-relative pick (0, 0.20) as tested
  - red markers: pharmacy P (end of the line), HOME (under the parked robot), one per room;
    green markers on the junctions
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import make_hospital_world as W                # noqa: E402  (tested room, room coordinates)
tbl = W.tbl                                     # scripts/smart_table.py constants

# ------------------------------------------------------------------ main numbers (metres)
CORR_HALF = 0.60            # corridor clear half width
WALL_T = 0.10               # wall thickness where two spaces meet (two 0.05 halves)
WALL_H = W.WALL_H           # 2.0
JUNCTIONS = {'J1': -0.75, 'J2': -3.65}          # x on the main line (y = 0)
ROOMS = {1: ('J1', 'N'), 2: ('J1', 'S'), 3: ('J2', 'N'), 4: ('J2', 'S')}
HOME_POSE = (0.0, 0.0, 0.0)                     # joint1 x, y, yaw (spawn pose)
HOME_MARK = -0.08                               # red marker under the parked robot's body
PHARM_MARK = 0.55                               # red marker, end of the line
STOP_AHEAD = 0.2777                             # marker centre ahead of joint1 at a red stop (Gazebo)
MARKER = 0.15                                   # marker squares
LINE_W = W.LINE_W                               # 0.03 tape
N_SLOTS, SLOT_PITCH = 6, 0.10
REACH = W.REACH                                 # 0.20: pick / place left of joint1
PLATE_H = W.PLATE_H                             # 0.10 counter top = arm plate height
P_STOP = PHARM_MARK - STOP_AHEAD                # expected pharmacy stop (joint1 x)
SLOT_X = {'S%d' % k: round(P_STOP + (k - (N_SLOTS + 1) / 2) * SLOT_PITCH, 4) for k in range(1, N_SLOTS + 1)}
CONTAINER_Z = PLATE_H + 0.031                   # spawn height (centre), drops 1 mm onto the counter
COUNTER = (min(SLOT_X.values()) - 0.08, max(SLOT_X.values()) + 0.08, REACH - 0.03, REACH + 0.27)  # x0 x1 y0 y1
CABINET_D = 0.33                                # medication cabinet depth behind the counter
CORR_X0 = JUNCTIONS['J2'] - CORR_HALF           # corridor west end (inner face)
CORR_X1 = 1.55                                  # corridor east end (inner face)
TURN_R, TURN_OFFSET = W.TURN_R, W.TURN_OFFSET   # 0.27 m swept circle, 0.12 m strafe first
ROBOT_HALF_W = W.ROBOT_HALF_W

# room shell in ROOM coordinates (= the single-room world's coordinates; line at x = XS)
XS = W.XS                   # 1.70
R_FAR = -0.60               # inner face behind the far side of the bed (single room: -0.72)
R_NEAR = 2.20               # inner face on the robot's side
R_HEAD = 0.55               # inner face of the corridor wall (bed head end)
R_FOOT = W.WALL_Y           # 3.15
DOOR = (XS - 0.40, XS + 0.40)
Y_OFF = CORR_HALF + WALL_T - R_HEAD             # room y + Y_OFF = distance from the corridor centre line
BRANCH_STOP = W.STOP_Y + Y_OFF                  # junction -> joint1 at the room stop (1.622)
BRANCH_MARK = W.Y_END + Y_OFF                   # junction -> room marker (1.900)
HOME_STOP = HOME_MARK - STOP_AHEAD              # joint1 when HOME stops the robot going east

PHARM_X0 = JUNCTIONS['J1'] + (R_NEAR - XS) + WALL_T   # pharmacy room west inner face (= R1 near wall outer)
PHARM_Y1 = R_FOOT + Y_OFF                              # pharmacy back wall inner face (as deep as the rooms)


def room_pose(room):
    """Pose (x, y, yaw) of a room's frame in the world: world = pose * room coordinates."""
    jx = JUNCTIONS[ROOMS[room][0]]
    if ROOMS[room][1] == 'N':
        return (jx - XS, Y_OFF, 0.0)
    return (jx + XS, -Y_OFF, math.pi)


def room_tf(room):
    """f(x, y) room coordinates -> world (x, y)."""
    px, py, yaw = room_pose(room)
    c, s = math.cos(yaw), math.sin(yaw)
    return lambda x, y: (px + c * x - s * y, py + s * x + c * y)


def room_info(room):
    """World geometry of a room's stop and smart table."""
    f = room_tf(room)
    jn, side = ROOMS[room]
    jx = JUNCTIONS[jn]
    sgn = 1 if side == 'N' else -1
    heading = 90.0 if side == 'N' else -90.0
    return {
        'node': 'R%d' % room, 'junction': jn, 'side': side,
        'marker': (jx, sgn * BRANCH_MARK), 'stop': (jx, sgn * BRANCH_STOP, heading),
        'table': 'smart_table_%d' % room, 'prefix': '/smart_table_%d' % room,
        'platform_center': f(*tbl.PLATFORM_CENTER), 'platform_size': tbl.PLATFORM_SIZE,
        'place_target': f(XS - REACH, W.STOP_Y), 'yaw_deg': math.degrees(room_pose(room)[2]),
        'turn_centre': (jx + sgn * TURN_OFFSET, sgn * BRANCH_STOP),
    }


def graph():
    """Nodes (marker positions) and edges of the line network."""
    nodes = {'pharmacy': (PHARM_MARK, 0.0, 'pharmacy', 'red'),
             'home': (HOME_MARK, 0.0, 'home', 'red')}
    for jn, x in JUNCTIONS.items():
        nodes[jn] = (x, 0.0, 'junction', 'green')
    for r in ROOMS:
        nodes['R%d' % r] = room_info(r)['marker'] + ('room', 'red')
    edges = [('pharmacy', 'home'), ('home', 'J1'), ('J1', 'J2')] + \
            [(ROOMS[r][0], 'R%d' % r) for r in ROOMS]
    return nodes, edges


# ------------------------------------------------------------------ wrist camera model (arm home pose)
CAM_H, CAM_PITCH, CAM_AHEAD = 0.396, math.radians(54.1), 0.062   # tf2_echo; ahead fitted to the strips
IMG_W, IMG_H, HFOV = 320, 200, 1.047
F_PX = (IMG_W / 2) / math.tan(HFOV / 2)


def ground_point(u, v, cam_ahead=CAM_AHEAD, cam_h=CAM_H, pitch=CAM_PITCH):
    """Pixel (u, v) -> floor point (ahead, left) of joint1, or None above the horizon."""
    a = pitch + math.atan((v - IMG_H / 2) / F_PX)
    if a <= 0.01:
        return None
    d = cam_h / math.tan(a)
    zc = math.hypot(cam_h, d) * math.cos(a - pitch)
    return cam_ahead + d, -(u - IMG_W / 2) / F_PX * zc


# ------------------------------------------------------------------ route / time estimates (plan)
SEC_PER_M = 39.0 / 2.957      # measured: 2.957 m in 39.0 sim s (incl. start and the old arc)
T_STEP = {'junction_turn': 7.0, 'pick': 20.0, 'arm_home': 3.6, 'turnaround': 16.4, 'place': 16.4,
          'lift': 18.3, 'cap_check': 7.2}


def estimate(room, slot=None):
    jx = JUNCTIONS[ROOMS[room][0]]
    xs = P_STOP if slot is None else SLOT_X[slot]
    leg1 = P_STOP - HOME_POSE[0]
    leg2 = xs - jx + BRANCH_STOP
    leg3 = BRANCH_STOP + HOME_STOP - jx
    align = abs(xs - P_STOP) if slot else 0.15
    t = {'leg1': leg1 * SEC_PER_M, 'align': 1.0 + align / 0.05, 'pick': T_STEP['pick'] + T_STEP['arm_home'],
         'turn_pharmacy': T_STEP['turnaround'], 'leg2': leg2 * SEC_PER_M + T_STEP['junction_turn'],
         'place_lift': T_STEP['place'] + T_STEP['arm_home'] + T_STEP['lift'], 'turn_room': T_STEP['turnaround'],
         'leg3': leg3 * SEC_PER_M + T_STEP['junction_turn'], 'park': 1.0 + (-HOME_STOP) / 0.05}
    return (leg1, leg2, leg3), t


# ------------------------------------------------------------------ clearance checks
def _rect_pts(x0, x1, y0, y1, n=12):
    return ([(x0 + (x1 - x0) * i / n, y) for i in range(n + 1) for y in (y0, y1)] +
            [(x, y0 + (y1 - y0) * i / n) for i in range(n + 1) for x in (x0, x1)])


def _seg_dist(p, a, b):
    ax, ay = b[0] - a[0], b[1] - a[1]
    t = max(0.0, min(1.0, ((p[0] - a[0]) * ax + (p[1] - a[1]) * ay) / (ax * ax + ay * ay)))
    return math.hypot(p[0] - a[0] - t * ax, p[1] - a[1] - t * ay)


def obstacles():
    """Floor outlines (sampled points, world) of everything near the robot's paths."""
    obs = {}
    for r in ROOMS:
        f = room_tf(r)
        fp = W.footprints()
        for name in ('smart table base', 'smart table platform', 'bed', 'iv pole', 'monitor', 'chair'):
            obs['R%d %s' % (r, name)] = [f(*p) for p in fp[name]]
        for nm, (x0, x1, y0, y1) in (('near wall', (R_NEAR, R_NEAR + 0.05, R_HEAD - WALL_T, R_FOOT + 0.05)),
                                     ('far wall', (R_FAR - 0.05, R_FAR, R_HEAD - WALL_T, R_FOOT + 0.05)),
                                     ('door jamb W', (R_FAR - 0.05, DOOR[0], R_HEAD - WALL_T, R_HEAD)),
                                     ('door jamb E', (DOOR[1], R_NEAR + 0.05, R_HEAD - WALL_T, R_HEAD))):
            obs['R%d %s' % (r, nm)] = [f(*p) for p in _rect_pts(x0, x1, y0, y1)]
    obs['pharmacy counter'] = _rect_pts(*COUNTER)
    # the corridor's side walls are the rooms' head walls (door jambs above) and the
    # pharmacy's corridor wall either side of the cabinet opening
    obs['pharmacy corridor wall W'] = _rect_pts(PHARM_X0, COUNTER[0], CORR_HALF, CORR_HALF + WALL_T)
    obs['pharmacy corridor wall E'] = _rect_pts(COUNTER[1], CORR_X1, CORR_HALF, CORR_HALF + WALL_T)
    obs['corridor west end'] = _rect_pts(CORR_X0 - 0.10, CORR_X0, -CORR_HALF, CORR_HALF)
    obs['corridor east end'] = _rect_pts(CORR_X1, CORR_X1 + 0.10, -CORR_HALF, CORR_HALF)
    return obs


def route_segments():
    nodes, edges = graph()
    return [((nodes[a][0], nodes[a][1]), (nodes[b][0], nodes[b][1])) for a, b in edges]


def turn_circles():
    """Every place where the robot rotates in place: (name, centre)."""
    c = {('junction ' + jn): (x, 0.0) for jn, x in JUNCTIONS.items()}
    for r in ROOMS:
        c['R%d turn round' % r] = room_info(r)['turn_centre']
    for s, x in SLOT_X.items():          # pharmacy: strafe RIGHT (south, away from the counter) first
        c['pharmacy turn round at %s' % s] = (x, -TURN_OFFSET)
    return c


def check(verbose=True):
    """Clearances: everything >= robot half width + 3.5 cm from the line, outside every
    turning circle (+1.5 cm); routes < 6 m; marker spacing; room overlap."""
    need = ROBOT_HALF_W + 0.035
    segs = route_segments()
    circles = turn_circles()
    bad, report = [], []
    for name, pts in obstacles().items():
        d = min(_seg_dist(p, a, b) for p in pts for a, b in segs)
        dc = {k: min(math.hypot(p[0] - c[0], p[1] - c[1]) for p in pts) for k, c in circles.items()}
        k_min = min(dc, key=dc.get)
        report.append('  %-26s line %.3f m   nearest turn circle %.3f m (%s)' % (name, d, dc[k_min], k_min))
        if d < need - 1e-6:
            bad.append('%s only %.3f m from the line (need %.3f)' % (name, d, need))
        for k, v in dc.items():
            if v < TURN_R + 0.015 - 1e-9:
                bad.append('%s %.3f m from the %s centre (need > %.3f)' % (name, v, k, TURN_R + 0.015))
    # the pharmacy turn round explicitly (user request): swept circle vs the counter edge
    for s, x in SLOT_X.items():
        c = (x, -TURN_OFFSET)
        d = min(math.hypot(p[0] - c[0], p[1] - c[1]) for p in _rect_pts(*COUNTER, n=40))
        report.append('  pharmacy turn round at %s: centre (%.3f, %.3f), counter edge %.3f m from it '
                      '(swept circle %.2f m + 1.5 cm)' % (s, c[0], c[1], d, TURN_R))
    # robot body on the line clears the counter and the containers
    if COUNTER[2] - ROBOT_HALF_W < 0.035:
        bad.append('counter edge %.3f m from the robot side' % (COUNTER[2] - ROBOT_HALF_W))
    # routes, markers
    legs = [estimate(r)[0] for r in ROOMS]
    longest = max(max(l) for l in legs)
    if longest > 6.0:
        bad.append('longest route %.2f m > 6 m' % longest)
    xs = sorted([HOME_MARK, PHARM_MARK] + list(JUNCTIONS.values()))
    spacing = min(b - a for a, b in zip(xs, xs[1:]))
    if spacing < 0.6:
        bad.append('markers on the main line %.2f m apart (< 0.6)' % spacing)
    # neighbouring rooms must not overlap (north: R3 | R1 | pharmacy, south: R4 | R2)
    for a, b in ((3, 1), (4, 2)):
        fa, fb = room_tf(a), room_tf(b)
        xa = sorted([fa(R_FAR, R_HEAD)[0], fa(R_NEAR, R_HEAD)[0]])
        xb = sorted([fb(R_FAR, R_HEAD)[0], fb(R_NEAR, R_HEAD)[0]])
        if xa[1] > xb[0] - WALL_T + 1e-9:
            bad.append('room %d overlaps room %d' % (a, b))
    if PHARM_X0 - 1e-9 > COUNTER[0]:
        bad.append('counter starts west of the pharmacy')
    if verbose:
        print('clearances (need line >= %.3f m, turn circles > %.3f m):' % (need, TURN_R + 0.015))
        print('\n'.join(report))
        print('longest one-way route %.2f m; marker spacing on the main line >= %.2f m' % (longest, spacing))
    assert not bad, '\n'.join(bad)
    return report


# ------------------------------------------------------------------ layout drawing
def draw(path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Polygon, Rectangle
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch

    fig = plt.figure(figsize=(17, 12.4), dpi=105)
    ax = fig.add_axes([0.03, 0.215, 0.70, 0.75])
    axz = fig.add_axes([0.745, 0.47, 0.245, 0.49])
    axt = fig.add_axes([0.03, 0.005, 0.96, 0.155])
    axt.axis('off')
    WALLC, LINEC, REDC, GRNC = '#555a60', '#1a3fe0', '#e01f1f', '#19b33a'
    caps = load_inventory_caps()

    def rect(a, x0, x1, y0, y1, **kw):
        a.add_patch(Rectangle((min(x0, x1), min(y0, y1)), abs(x1 - x0), abs(y1 - y0), **kw))

    def poly(a, pts, **kw):
        a.add_patch(Polygon(pts, closed=True, **kw))

    def room_rect(a, f, x0, x1, y0, y1, **kw):
        poly(a, [f(x0, y0), f(x1, y0), f(x1, y1), f(x0, y1)], **kw)

    def furniture(a, r, labels=True):
        f = room_tf(r)
        room_rect(a, f, R_FAR, W.BED_X1 + 0.045, R_HEAD, R_FOOT, fc='#e8dfcc', ec='none', zorder=1)
        for x0, x1, y0, y1 in ((R_FAR - 0.05, R_FAR, R_HEAD - WALL_T, R_FOOT + 0.05),
                               (R_FAR - 0.05, R_NEAR + 0.05, R_FOOT, R_FOOT + 0.05),
                               (R_NEAR, R_NEAR + 0.05, R_HEAD - WALL_T, R_FOOT + 0.05),
                               (R_FAR - 0.05, DOOR[0], R_HEAD - WALL_T, R_HEAD),
                               (DOOR[1], R_NEAR + 0.05, R_HEAD - WALL_T, R_HEAD)):
            room_rect(a, f, x0, x1, y0, y1, fc=WALLC, ec='none', zorder=3)
        room_rect(a, f, W.BED_X0 - 0.005, W.BED_X1 + 0.005, W.BED_Y0 - 0.01, W.BED_Y1 + 0.01, fc='#b8bcc2', ec='#6b7078', lw=0.8, zorder=4)
        room_rect(a, f, W.BED_X0, W.BED_X1, W.BED_Y0, W.BED_Y1, fc='#f4f4f2', ec='#8a8f96', lw=0.6, zorder=5)
        room_rect(a, f, W.BED_X0 - 0.02, W.BED_X1 + 0.02, W.HINGE_Y + 0.04, W.BED_Y1 - 0.03, fc='#5c8c80', ec='none', zorder=6)
        room_rect(a, f, W.BED_CX - 0.31, W.BED_CX + 0.31, W.BED_Y0 + 0.08, W.BED_Y0 + 0.36, fc='#dcdcd8', ec='#9a9a96', lw=0.5, zorder=6)
        for y0, y1 in ((W.BED_Y0 - 0.045, W.BED_Y0 - 0.015), (W.BED_Y1 + 0.015, W.BED_Y1 + 0.045)):
            room_rect(a, f, W.BED_X0 - 0.03, W.BED_X1 + 0.03, y0, y1, fc='#c9ad85', ec='none', zorder=6)
        for x, y0, y1 in ((W.BED_X0 - 0.025, W.BED_Y0 + 0.06, W.BED_Y0 + 0.58), (W.BED_X0 - 0.025, W.BED_Y0 + 0.92, W.BED_Y0 + 1.72),
                          (W.BED_X1 + 0.025, W.BED_Y0 + 0.06, W.BED_Y0 + 0.58)):
            a.plot(*zip(f(x, y0), f(x, y1)), color='#7d838b', lw=1.6, zorder=7, solid_capstyle='butt')
        bx1, cy = tbl.PLATFORM_ROBOT_EDGE_X, tbl.PLATFORM_CENTER[1]
        room_rect(a, f, bx1 - W.TABLE_BASE[0], bx1, cy - W.TABLE_BASE[1] / 2, cy + W.TABLE_BASE[1] / 2, fc='#efefea', ec='#8c9096', lw=0.7, zorder=8)
        room_rect(a, f, bx1 - tbl.PLATFORM_SIZE[0], bx1, cy - tbl.PLATFORM_SIZE[1] / 2, cy + tbl.PLATFORM_SIZE[1] / 2, fc='#c8cbd0', ec='#3d4248', lw=1.0, zorder=9)
        a.plot(*f(XS - REACH, W.STOP_Y), marker='x', color='#222', ms=4, mew=1.0, zorder=10)
        for (x, y), rr in ((W.IV_XY, 0.25), (W.MONITOR_XY, 0.24)):
            a.add_patch(Circle(f(x, y), rr, fc='none', ec='#9aa0a8', lw=0.8, zorder=6))
            a.add_patch(Circle(f(x, y), 0.03, fc='#6b7078', ec='none', zorder=6))
        mx, my = W.MONITOR_XY
        room_rect(a, f, mx + 0.03, mx + 0.115, my - 0.19, my + 0.19, fc='#25272a', ec='none', zorder=7)
        cx, cy2, cw = W.CHAIR_XY[0], W.CHAIR_XY[1], W.CHAIR_YAW
        pts = [f(cx + lx * math.cos(cw) - ly * math.sin(cw), cy2 + lx * math.sin(cw) + ly * math.cos(cw))
               for lx, ly in ((-0.28, -0.30), (0.24, -0.30), (0.24, 0.30), (-0.28, 0.30))]
        poly(a, pts, fc='#6e8a88', ec='#3f5250', lw=0.7, zorder=6)
        room_rect(a, f, XS - 0.475, XS + 0.475, R_FOOT + 0.01, R_FOOT + 0.04, fc='#cfe3df', ec='none', zorder=4)
        room_rect(a, f, DOOR[1] + 0.03, DOOR[1] + 0.33, R_HEAD - WALL_T - 0.02, R_HEAD - WALL_T, fc='#f2f2f2', ec='#333', lw=0.6, zorder=5)
        if labels:
            a.text(*f(0.45, 2.95), 'Room %d' % r, fontsize=13, weight='bold', ha='center', va='center', zorder=20)

    def robot(a, x, y, yaw, fc='#ffb347', alpha=1.0, ls='-'):
        c, s = math.cos(yaw), math.sin(yaw)
        pts = [(x + c * px - s * py, y + s * px + c * py) for px, py in ((-0.237, -0.13), (0.073, -0.13), (0.073, 0.13), (-0.237, 0.13))]
        poly(a, pts, fc=fc, ec='#a0522d', lw=1.0, zorder=30, alpha=alpha, ls=ls)
        if fc != 'none':
            a.annotate('', xy=(x + 0.22 * c, y + 0.22 * s), xytext=(x, y), arrowprops=dict(arrowstyle='-|>', color='#8b3a00', lw=1.4), zorder=31)
        a.plot(x, y, 'o' if fc != 'none' else 'x', color='#8b3a00', ms=3 if fc != 'none' else 6, zorder=32)

    def lines(a):
        lw = 2.6 if a is ax else 6
        a.plot([PHARM_MARK + MARKER / 2, JUNCTIONS['J2']], [0, 0], color=LINEC, lw=lw, solid_capstyle='butt', zorder=12)
        for r in ROOMS:
            inf = room_info(r)
            jx = JUNCTIONS[inf['junction']]
            mx, my = inf['marker']
            a.plot([jx, jx], [0, my + math.copysign(MARKER / 2, my)], color=LINEC, lw=lw, solid_capstyle='butt', zorder=12)
            rect(a, mx - MARKER / 2, mx + MARKER / 2, my - MARKER / 2, my + MARKER / 2, fc=REDC, ec='none', zorder=13)
        for x in JUNCTIONS.values():
            rect(a, x - MARKER / 2, x + MARKER / 2, -MARKER / 2, MARKER / 2, fc=GRNC, ec='none', zorder=13)
        for x in (HOME_MARK, PHARM_MARK):
            rect(a, x - MARKER / 2, x + MARKER / 2, -MARKER / 2, MARKER / 2, fc=REDC, ec='none', zorder=13)

    def pharmacy(a, zoom=False):
        x0, x1, y0, y1 = PHARM_X0, CORR_X1, CORR_HALF + WALL_T, PHARM_Y1
        rect(a, x0, x1 + 0.05, y1, y1 + 0.05, fc=WALLC, ec='none', zorder=3)
        rect(a, x1, x1 + 0.05, y0 - WALL_T, y1 + 0.05, fc=WALLC, ec='none', zorder=3)
        rect(a, COUNTER[1], x1, y0 - WALL_T, y0, fc=WALLC, ec='none', zorder=3)
        rect(a, x0, COUNTER[0], y0 - WALL_T, y0, fc=WALLC, ec='none', zorder=3)
        rect(a, COUNTER[0], COUNTER[1], COUNTER[3], COUNTER[3] + CABINET_D, fc='#d9dde3', ec='#6b7078', lw=0.8, hatch='////', zorder=8)
        rect(a, COUNTER[0], COUNTER[1], COUNTER[2], COUNTER[3], fc='#e6e6e2', ec='#5a5f66', lw=0.9, zorder=9)
        for s, x in SLOT_X.items():
            rgb = caps.get(s, (0.8, 0.8, 0.8))
            a.add_patch(Circle((x, REACH), 0.016, fc=rgb, ec='#222', lw=0.6, zorder=15))
            if zoom:
                a.text(x, REACH + 0.03, s, fontsize=6.5, ha='center', va='bottom', zorder=16)
        if zoom:
            a.text((COUNTER[0] + COUNTER[1]) / 2, COUNTER[3] + 0.165, 'medication cabinet\n(shelves + boxes, visual)', fontsize=6.5,
                   ha='center', va='center', zorder=16, bbox=dict(fc='white', ec='none', alpha=0.85, pad=0.6))
            a.text((COUNTER[0] + COUNTER[1]) / 2, COUNTER[2] + 0.20, 'pick counter, top 0.10 m', fontsize=6.3, ha='center', va='center', zorder=16)
            a.text(0.80, 0.93, 'PHARMACY', fontsize=10, weight='bold', ha='center', zorder=20)
        else:
            a.text((x0 + x1) / 2 + 0.1, y0 + 1.0, 'PHARMACY', fontsize=13, weight='bold', ha='center', zorder=20)
            a.text((x0 + x1) / 2 + 0.1, y0 + 0.75, 'cabinet + pick counter\n(6 slots, top 0.10 m)', fontsize=7.5, ha='center', zorder=20)

    for a in (ax, axz):
        a.set_facecolor('#d9d9d6')
        rect(a, CORR_X0, CORR_X1, -CORR_HALF, CORR_HALF, fc='white', ec='none', zorder=0)
        for r in ROOMS:
            f = room_tf(r)
            room_rect(a, f, R_FAR, R_NEAR, R_HEAD, R_FOOT, fc='white', ec='none', zorder=0)
            room_rect(a, f, DOOR[0], DOOR[1], R_HEAD - WALL_T, R_HEAD, fc='white', ec='none', zorder=0)
        rect(a, PHARM_X0, CORR_X1, CORR_HALF, PHARM_Y1, fc='white', ec='none', zorder=0)
        rect(a, CORR_X0 - 0.10, CORR_X0, -CORR_HALF - WALL_T, CORR_HALF + WALL_T, fc=WALLC, ec='none', zorder=3)
        rect(a, CORR_X1, CORR_X1 + 0.05, -CORR_HALF - WALL_T, CORR_HALF, fc=WALLC, ec='none', zorder=3)
        r2_x1 = room_tf(2)(R_FAR - 0.05, 0)[0]
        if CORR_X1 > r2_x1:
            rect(a, r2_x1, CORR_X1, -CORR_HALF - WALL_T, -CORR_HALF, fc=WALLC, ec='none', zorder=3)
        for r in ROOMS:
            furniture(a, r, labels=(a is ax))
        pharmacy(a, zoom=(a is axz))
        lines(a)

    robot(ax, 0, 0, 0)
    robot(axz, 0, 0, 0)
    # zoom: camera view, pharmacy stop, slot range, turn-round circles at every slot vs the counter
    fp = [ground_point(u, v) for u, v in ((0, IMG_H), (IMG_W, IMG_H), (IMG_W, 0), (0, 0))]
    poly(axz, fp, fc='#7ab8ff', ec='#2a6fd6', alpha=0.15, lw=0.8, zorder=25)
    for y0f, y1f in ((0.90, 0.95), (0.80, 0.85), (0.70, 0.75)):
        st = [ground_point(u, v) for u, v in ((0, y1f * IMG_H), (IMG_W, y1f * IMG_H), (IMG_W, y0f * IMG_H), (0, y0f * IMG_H))]
        poly(axz, st, fc='#2a6fd6', ec='none', alpha=0.35, zorder=26)
    axz.annotate('wrist-camera view from home\n(dark bands = steering strips)', xy=ground_point(IMG_W * 0.75, 20),
                 xytext=(0.55, 0.80), fontsize=6.5, color='#1d4f9c', ha='center',
                 arrowprops=dict(arrowstyle='->', color='#1d4f9c', lw=0.7), zorder=40)
    robot(axz, P_STOP, 0, 0, fc='none', ls='--')
    axz.annotate('pharmacy stop (joint1 %.2f m\nbefore the P marker)' % STOP_AHEAD, xy=(P_STOP, -0.02),
                 xytext=(0.74, -0.45), fontsize=6.5, color='#8b3a00', ha='center',
                 arrowprops=dict(arrowstyle='->', color='#8b3a00', lw=0.7), zorder=40)
    xs = list(SLOT_X.values())
    axz.plot([xs[0], xs[-1]], [-0.20, -0.20], color='#8b3a00', lw=1.0, zorder=24)
    for x in xs:
        axz.plot([x, x], [-0.215, -0.185], color='#8b3a00', lw=0.8, zorder=24)
    axz.text((xs[0] + xs[-1]) / 2, -0.255, 'joint1 at slot k: forward/back on /odom, pick at (0, 0.20)',
             fontsize=6, ha='center', va='top', color='#8b3a00', zorder=40)
    for x in xs:
        axz.add_patch(Circle((x, -TURN_OFFSET), TURN_R, fc='none', ec='#999', ls=':', lw=0.6, zorder=24))
    for x in (xs[0], xs[-1]):
        axz.add_patch(Circle((x, -TURN_OFFSET), TURN_R, fc='none', ec='#555', ls='--', lw=0.8, zorder=24))
        axz.plot(x, -TURN_OFFSET, '+', color='#555', ms=5, zorder=24)
    gap = COUNTER[2] - (-TURN_OFFSET) - TURN_R
    axz.text(-0.05, -0.565, 'turn round: strafe 0.12 m away from the counter, then rotate:\n'
             'swept circle (0.27 m) stays %.0f mm clear of the counter at every slot' % (1000 * gap),
             fontsize=5.8, color='#333', ha='center')
    for x, lab in ((HOME_MARK, 'HOME'), (PHARM_MARK, 'P')):
        axz.text(x, 0.085, lab, fontsize=7, ha='center', va='bottom', color=REDC, weight='bold', zorder=40)
    axz.annotate('robot at home = start\n(0, 0) facing +x', xy=(-0.12, 0.0), xytext=(-0.62, 0.42), fontsize=6.5,
                 color='#8b3a00', ha='center', arrowprops=dict(arrowstyle='->', color='#8b3a00', lw=0.7), zorder=40)
    axz.text(JUNCTIONS['J1'] + 0.09, -0.10, 'J1', fontsize=8, ha='left', va='top', color='#0d7a26', weight='bold', zorder=40)
    axz.set_xlim(-1.0, 1.05)
    axz.set_ylim(-0.62, 1.05)
    axz.set_aspect('equal')
    axz.set_title('pharmacy end (zoom)', fontsize=10)
    axz.tick_params(labelsize=7)
    # inventory table
    axi = fig.add_axes([0.745, 0.215, 0.245, 0.22])
    axi.axis('off')
    inv = load_inventory()
    cells = [[s, '%s %s' % (inv[s]['medicine'], inv[s]['dose']), inv[s]['cap'], str(inv[s]['quantity']), '%.3f' % SLOT_X[s]]
             for s in SLOT_X if s in inv]
    if cells:
        ti = axi.table(cellText=cells, colLabels=['slot', 'medicine + dose', 'cap', 'qty', 'x (m)'], loc='upper center',
                       cellLoc='left', colWidths=[0.11, 0.47, 0.15, 0.09, 0.15])
        ti.auto_set_font_size(False)
        ti.set_fontsize(7.5)
        ti.scale(1, 1.3)
        for i, row in enumerate(cells):
            rgb = caps.get(row[0], (0.8, 0.8, 0.8))
            ti[i + 1, 2].set_facecolor(rgb if sum(rgb) > 1.2 else '#cfcfcf')
    axi.set_title('inventory (config/inventory.yaml): 6 slots, 0.10 m apart, y = 0.20, counter top 0.10 m\n'
                  'containers med_S1..med_S6: same 3 x 6 cm, 30 g; cap colour + label texture', fontsize=7.5)

    for jn, x in JUNCTIONS.items():
        ax.add_patch(Circle((x, 0), TURN_R, fc='none', ec='#888', ls='--', lw=0.7, zorder=24))
        ax.text(x + 0.10, -0.12, jn, fontsize=11, color='#0d7a26', weight='bold', va='top', zorder=40)
    for r in ROOMS:
        inf = room_info(r)
        sx, sy, hd = inf['stop']
        robot(ax, sx, sy, math.radians(hd), fc='#ffe0b0', alpha=0.8)
        ax.add_patch(Circle(inf['turn_centre'], TURN_R, fc='none', ec='#888', ls='--', lw=0.7, zorder=24))
        mx, my = inf['marker']
        ax.text(mx - math.copysign(0.08, my), my + math.copysign(0.15, my), inf['node'], fontsize=10, color=REDC,
                weight='bold', ha='center', va='center', zorder=40)
    ax.text(HOME_MARK, -0.16, 'HOME', fontsize=8, color=REDC, weight='bold', ha='center', va='top', zorder=40)
    ax.text(PHARM_MARK + 0.02, -0.16, 'P', fontsize=9, color=REDC, weight='bold', ha='center', va='top', zorder=40)

    def dim(a, p0, p1, txt, off=(0, 0)):
        a.annotate('', xy=p1, xytext=p0, arrowprops=dict(arrowstyle='<->', color='#444', lw=0.8), zorder=41)
        a.text((p0[0] + p1[0]) / 2 + off[0], (p0[1] + p1[1]) / 2 + off[1], txt, fontsize=8, ha='center', va='center',
               color='#222', zorder=41, bbox=dict(fc='white', ec='none', alpha=0.8, pad=0.5))
    j1, j2 = JUNCTIONS['J1'], JUNCTIONS['J2']
    dim(ax, (j2, -0.42), (j1, -0.42), '%.2f m' % (j1 - j2))
    dim(ax, (j1, -0.42), (HOME_MARK, -0.42), '%.2f' % (HOME_MARK - j1))
    dim(ax, (HOME_MARK, -0.42), (PHARM_MARK, -0.42), '%.2f' % (PHARM_MARK - HOME_MARK))
    dim(ax, (j2 + 0.42, 0), (j2 + 0.42, BRANCH_MARK), '%.2f m\nto marker' % BRANCH_MARK, off=(0.33, 0.25))
    dim(ax, (CORR_X1 + 0.25, -CORR_HALF), (CORR_X1 + 0.25, CORR_HALF), '1.2 m\ncorridor', off=(0.27, 0))
    f1 = room_tf(1)
    dim(ax, (f1(R_FAR - 0.05, 0)[0], 3.55), (f1(R_NEAR + 0.05, 0)[0], 3.55),
        'room %.2f x %.2f m (inside)' % (R_NEAR - R_FAR, R_FOOT - R_HEAD))
    ax.text(j1 - 2.2, 0.32, 'corridor (blue main line, y = 0)', fontsize=9, color='#333', zorder=40)
    ax.annotate('robot start / home\n(0, 0) facing +x', xy=(0.0, 0.0), xytext=(0.25, -1.25), fontsize=9, color='#8b3a00',
                arrowprops=dict(arrowstyle='->', color='#8b3a00'), zorder=42)
    ax.set_xlim(-6.35, 2.15)
    ax.set_ylim(-3.75, 3.75)
    ax.set_aspect('equal')
    ax.grid(True, color='#bbb', lw=0.3, zorder=-1)
    ax.tick_params(labelsize=8)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title('Hospital system - top view (branch hospital-system): corridor, pharmacy, Rooms 1-4, line network', fontsize=12)
    h = [Line2D([], [], color=LINEC, lw=2.6, label='blue line (3 cm tape)'),
         Patch(fc=REDC, label='red stop marker 15 cm (rooms, pharmacy, home)'),
         Patch(fc=GRNC, label='green junction marker 15 cm (J1, J2)'),
         Patch(fc='#c8cbd0', ec='#3d4248', label='smart table platform 25x25 cm (x = place target)'),
         Patch(fc='#ffb347', ec='#a0522d', label='robot at home (start)'),
         Patch(fc='#ffe0b0', ec='#a0522d', label='robot at a room stop'),
         Line2D([], [], color='#888', ls='--', lw=0.8, label='0.27 m turning circles (junction, turn round)'),
         Patch(fc='#f2f2f2', ec='#333', label='room sign "Room N" (texture) next to each door')]
    ax.legend(handles=h, loc='lower left', fontsize=7.5, framealpha=0.92)
    col = ['delivery', 'leg 1 home->P', 'leg 2 P->room (actions)', 'leg 3 room->home (actions)', 'sim s',
           'wall @5.5%', 'wall @4.5%']
    cells, tot = [], 0.0
    acts = {1: ('J1 right 90 deg, stop at red R1', 'J1 left 90 deg, stop at red HOME'),
            2: ('J1 left 90 deg, stop at red R2', 'J1 right 90 deg, stop at red HOME'),
            3: ('J1 straight, J2 right 90 deg, stop at red R3', 'J2 left 90 deg, J1 straight, stop at red HOME'),
            4: ('J1 straight, J2 left 90 deg, stop at red R4', 'J2 right 90 deg, J1 straight, stop at red HOME')}
    for r in ROOMS:
        legs, t = estimate(r)
        s = sum(t.values())
        tot += s
        cells.append(['Room %d' % r, '%.2f m' % legs[0], '%.2f m: turn round, %s' % (legs[1], acts[r][0]),
                      '%.2f m: turn round, %s' % (legs[2], acts[r][1]), '%.0f' % s,
                      '%.0f min' % (s / 0.055 / 60), '%.0f min' % (s / 0.045 / 60)])
    cells.append(['demo R1-R4', '', '', '', '%.0f' % tot, '%.1f h' % (tot / 0.055 / 3600), '%.1f h' % (tot / 0.045 / 3600)])
    tb = axt.table(cellText=cells, colLabels=col, loc='center', cellLoc='left', colWidths=[0.07, 0.07, 0.36, 0.30, 0.05, 0.07, 0.07])
    tb.auto_set_font_size(False)
    tb.set_fontsize(7.6)
    tb.scale(1, 1.25)
    axt.set_title('Routes (every delivery: home -> pharmacy slot -> room -> home; pick at arm-relative (0, 0.20) as tested) - '
                  'times = measured step times of the 2026-11-05 missions', fontsize=8.5, loc='left')
    fig.savefig(path)
    plt.close(fig)
    return path


# ------------------------------------------------------------------ inventory (config/inventory.yaml)
INVENTORY = os.path.join(ROOT, 'config', 'inventory.yaml')
CAP_RGB = {'white': (0.95, 0.95, 0.93), 'yellow': (1.0, 0.86, 0.10), 'amber': (1.0, 0.62, 0.05),
           'grey': (0.36, 0.37, 0.39), 'black': (0.10, 0.10, 0.10)}


def load_inventory(path=INVENTORY):
    try:
        import yaml
        with open(path) as fh:
            data = yaml.safe_load(fh) or {}
    except (OSError, ImportError):
        return {}
    out = {}
    for s, d in (data.get('slots') or {}).items():
        out[str(s)] = {'medicine': str(d['medicine']), 'dose': str(d['dose']), 'cap': str(d['cap']),
                       'quantity': int(d.get('quantity', 1))}
    return out


def load_inventory_caps(path=INVENTORY):
    return {s: CAP_RGB.get(d['cap'], (0.8, 0.8, 0.8)) for s, d in load_inventory(path).items()}


if __name__ == '__main__':
    check()
    print('\nslots: %s' % ', '.join('%s %.3f' % kv for kv in SLOT_X.items()))
    print('pharmacy stop %.4f, home stop %.4f, junction -> room stop %.4f, -> marker %.4f'
          % (P_STOP, HOME_STOP, BRANCH_STOP, BRANCH_MARK))
    tot = 0.0
    for r in ROOMS:
        legs, t = estimate(r)
        s = sum(t.values())
        tot += s
        print('Room %d: legs %.2f / %.2f / %.2f m, sim %.0f s, wall %.0f min @5.5 %%' % ((r,) + legs + (s, s / 0.055 / 60)))
    print('demo: sim %.0f s' % tot)
    out = os.path.join(ROOT, 'results', 'system', 'layout.png')
    os.makedirs(os.path.dirname(out), exist_ok=True)
    print('wrote', draw(out))
