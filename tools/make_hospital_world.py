#!/usr/bin/env python3
"""Generates robot_gazebo/worlds/hospital.sdf (run again after editing the layout).

    python3 ~/sdp_sim/tools/make_hospital_world.py      then  bash ~/sdp_sim/install.sh

Also writes hospital_video.sdf (overview camera at 2 Hz, for videos),
hospital_photo.sdf (plus three high-resolution room cameras, for report pictures;
launch photo:=true) and the patient-monitor screen texture monitor_screen.png.

Layout (metres, robot starts at the origin facing +x):
  - medicine station: low table on the robot's LEFT at the start
  - blue line: straight along +x, 90-degree left turn, straight along +y
  - red stop marker at the patient end, and one behind the start (used when
    the robot turns around and follows the line home)
  - patient room on the robot's LEFT at the patient stop: hospital bed (long side
    parallel to the line, head toward -y), the SMART BEDSIDE TABLE between the
    robot's stop and the mattress edge (scripts/smart_table.py has its geometry and
    controller), IV pole, patient monitor and visitor chair on the far side of the
    bed, two back walls with a window, whiteboard and clock.
Table tops are at the arm-plate height (0.10 m) so the arm reaches them with
the same joint angles as the tested pick-and-place; the smart table's platform is at
0.10 m when lowered and lifts the container to 0.80 m.

Only the smart table moves; all furniture is static, visual-only except box
collisions for the bed and the table. No extra lights. Colours are checked against
the line follower's HSV ranges (no blue / red outside the floor line), and every
footprint against the robot's route and its turning circles.
"""
import colorsys
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'scripts'))
import smart_table as tbl          # noqa: E402  (platform geometry, shared with the controller)

PLATE_H   = 0.10    # chassis plate height = table top height
LINE_W    = 0.03    # tape width
LINE_T    = 0.001   # tape thickness (visual only)
R_TURN    = 0.50    # turn radius
X_TURN    = 1.20    # where the turn starts
Y_END     = 1.75    # patient stop marker (centre)
REACH     = 0.20    # sideways reach used for pick / place (m from arm joint1)
ROBOT_HALF_W = 0.13
HOME_X    = -0.22   # home stop marker (centre)
STOP_AHEAD = 0.29   # robot stops when the marker is this far ahead of joint1 (offline test)
XS        = X_TURN + R_TURN           # x of the straight towards the patient
STOP_Y    = 1.4723  # patient stop y measured in Gazebo (offline estimate Y_END - STOP_AHEAD = 1.46)
TURN_R    = 0.27    # circle swept when the robot rotates in place
TURN_OFFSET = 0.12  # strafe before rotating (mission.py --offset)

# ---------------------------------------------------------------- room layout
BED_W, BED_L = 0.90, 2.00             # mattress (real scale)
BED_X1 = 1.24                         # mattress edge on the table / robot side
BED_X0 = BED_X1 - BED_W
BED_CX = (BED_X0 + BED_X1) / 2
BED_Y0 = 0.66                         # head end of the mattress (head toward -y)
BED_Y1 = BED_Y0 + BED_L
HINGE_Y = BED_Y0 + 0.70               # back section hinge (patient's hips)
BACK_TILT = math.radians(35)          # raised back section
MATT_TOP, MATT_T = 0.58, 0.14
DECK_Z0 = MATT_TOP - MATT_T - 0.05    # deck frame under the mattress
RAIL_TABLE_SIDE_TOP = 0.74            # < 0.75: the patient reaches over it to the table
IV_XY = (0.10, 1.45)
MONITOR_XY = (-0.02, 0.85)
CHAIR_XY, CHAIR_YAW = (-0.12, 2.30), math.radians(-35)
WALL_X = -0.72                        # inner face of the wall behind the far side of the bed
WALL_Y = 3.15                         # inner face of the wall behind the foot of the bed
WALL_H = 2.0
TABLE_BASE = (0.32, 0.30, 0.05)       # smart table base (x, y, height), robot-side edge = platform's
LIMIT_MARGIN = 0.03                   # lift joint limits this far beyond the travel (see smart_table_model)

SCREEN_TEX = os.path.abspath(os.path.join(HERE, '..', 'robot_gazebo', 'worlds', 'monitor_screen.png'))

# line follower colour ranges (OpenCV HSV, scripts/line_follower.py) - keep in sync
BLUE_HUE = (100, 130)
RED_HUE = ((0, 10), (170, 180))

COLOURS = []        # (model, visual, rgb) of every material, for check_colours()
# The scene ambient is 0.8 (kept: the line follower's colour ranges were tuned under it),
# which washes out anything with ambient = diffuse. Furniture gets ambient = 0.6 x colour
# so its faces are shaded; the track, the station and the monitor screen keep 1.0.
FURNITURE_AMB = 0.6


# ---------------------------------------------------------------- SDF helpers
def _pose(p):
    p = tuple(p) + (0.0,) * (6 - len(p))
    return '<pose>%s</pose>' % ' '.join('%.4f' % v for v in p)


def _material(rgb, emissive=None, spec=0.1, texture=None, amb=1.0):
    e = emissive or (0, 0, 0)
    pbr = ('<pbr><metal><albedo_map>%s</albedo_map><metalness>0</metalness>'
           '<roughness>1</roughness></metal></pbr>' % texture) if texture else ''
    a = [round(amb * c, 4) for c in rgb[:3]]
    return ('<material><ambient>{8} {9} {10} 1</ambient><diffuse>{0} {1} {2} 1</diffuse>'
            '<specular>{3} {3} {3} 1</specular><emissive>{4} {5} {6} 1</emissive>{7}</material>'
            .format(rgb[0], rgb[1], rgb[2], spec, e[0], e[1], e[2], pbr, *a))


def box_g(sx, sy, sz):
    return '<box><size>%.4f %.4f %.4f</size></box>' % (sx, sy, sz)


def cyl_g(r, length):
    return '<cylinder><radius>%.4f</radius><length>%.4f</length></cylinder>' % (r, length)


def sph_g(r):
    return '<sphere><radius>%.4f</radius></sphere>' % r


class Parts:
    """Visuals (and collisions) of one link; records colours for the HSV check."""

    def __init__(self, model):
        self.model, self.xml = model, []

    def visual(self, name, pose, geom, rgb, emissive=None, spec=0.1, transparency=None, texture=None,
               amb=FURNITURE_AMB):
        COLOURS.append((self.model, name, rgb))
        if emissive and max(emissive) > 0:
            COLOURS.append((self.model, name + ' (emissive)', emissive))
        t = '<transparency>%.2f</transparency>' % transparency if transparency else ''
        self.xml.append('\n        <visual name="%s">%s<geometry>%s</geometry>%s%s</visual>'
                        % (name, _pose(pose), geom, _material(rgb, emissive, spec, texture, amb), t))

    def box(self, name, pose, size, rgb, **kw):
        self.visual(name, pose, box_g(*size), rgb, **kw)

    def cyl(self, name, pose, r, length, rgb, **kw):
        self.visual(name, pose, cyl_g(r, length), rgb, **kw)

    def sph(self, name, pose, r, rgb, **kw):
        self.visual(name, pose, sph_g(r), rgb, **kw)

    def cyl_between(self, name, p0, p1, r, rgb, **kw):
        d = [b - a for a, b in zip(p0, p1)]
        n = math.sqrt(sum(v * v for v in d))
        mid = [(a + b) / 2 for a, b in zip(p0, p1)]
        pitch = math.acos(max(-1.0, min(1.0, d[2] / n)))
        yaw = math.atan2(d[1], d[0])
        self.cyl(name, (mid[0], mid[1], mid[2], 0, pitch, yaw), r, n, rgb, **kw)

    def collision(self, name, pose, size, mu=1.0):
        self.xml.append('\n        <collision name="%s">%s<geometry>%s</geometry><surface><friction><ode>'
                        '<mu>%s</mu><mu2>%s</mu2></ode></friction></surface></collision>'
                        % (name, _pose(pose), box_g(*size), mu, mu))

    def __str__(self):
        return ''.join(self.xml)


def static_model(name, parts, pose=(0, 0, 0)):
    return """
    <model name="%s">
      <static>true</static>
      %s
      <link name="link">%s
      </link>
    </model>""" % (name, _pose(pose), parts)


def box_visual(name, x, y, z, yaw, sx, sy, sz, rgba, emissive=None):
    p = Parts('floor_line')
    p.box(name, (x, y, z, 0, 0, yaw), (sx, sy, sz), rgba, emissive=emissive, amb=1.0)
    return str(p)


def static_box_model(name, x, y, sx, sy, sz, rgba):
    p = Parts(name)
    p.collision('c', (0, 0, 0), (sx, sy, sz))
    p.box('v', (0, 0, 0), (sx, sy, sz), rgba, amb=1.0)
    return static_model(name, p, (x, y, sz / 2))


# ---------------------------------------------------------------- track
def line_model():
    blue, blue_e = (0.0, 0.15, 0.9), (0.0, 0.05, 0.35)
    red, red_e = (0.9, 0.05, 0.05), (0.35, 0.0, 0.0)
    z = LINE_T / 2 + 0.0005
    parts = []
    # straight 1: from the home marker to the turn
    x0, x1 = HOME_X, X_TURN
    parts.append(box_visual('s1', (x0 + x1) / 2, 0, z, 0, x1 - x0, LINE_W, LINE_T, blue, blue_e))
    # 90-degree left arc, centre (X_TURN, R_TURN)
    n = 9
    for i in range(n):
        a0 = -math.pi / 2 + (math.pi / 2) * i / n
        a1 = -math.pi / 2 + (math.pi / 2) * (i + 1) / n
        am = (a0 + a1) / 2
        cx = X_TURN + R_TURN * math.cos(am)
        cy = R_TURN + R_TURN * math.sin(am)
        chord = 2 * R_TURN * math.sin((a1 - a0) / 2) + 0.012
        parts.append(box_visual(f'arc{i}', cx, cy, z, am + math.pi / 2, chord, LINE_W, LINE_T, blue, blue_e))
    # straight 2: up to the patient marker
    y0, y1 = R_TURN, Y_END
    parts.append(box_visual('s2', XS, (y0 + y1) / 2, z, math.pi / 2, y1 - y0, LINE_W, LINE_T, blue, blue_e))
    # stop markers
    parts.append(box_visual('stop_patient', XS, Y_END, z + 0.0002, 0, 0.15, 0.15, LINE_T, red, red_e))
    parts.append(box_visual('stop_home', HOME_X, 0, z + 0.0002, 0, 0.15, 0.15, LINE_T, red, red_e))
    return f"""
    <model name="floor_line">
      <static>true</static>
      <link name="link">{''.join(parts)}
      </link>
    </model>"""


# ---------------------------------------------------------------- colours
WHITE = (0.94, 0.94, 0.93)
MATTE_WHITE = (0.92, 0.92, 0.90)
STEEL = (0.78, 0.79, 0.81)
STEEL_DARK = (0.55, 0.56, 0.58)
FRAME_GREY = (0.70, 0.71, 0.73)
DARK = (0.16, 0.16, 0.17)
WOOD = (0.80, 0.68, 0.52)           # light wood laminate (head/footboard)
CHAIR_WOOD = (0.72, 0.60, 0.45)
BLANKET = (0.36, 0.55, 0.50)        # soft teal-green (hue ~82 in OpenCV: not blue)
UPHOLSTERY = (0.36, 0.47, 0.46)
WALL = (0.90, 0.92, 0.88)
SKIRTING = (0.55, 0.58, 0.55)
ROOM_FLOOR = (0.58, 0.55, 0.49)     # renders light beige (the corridor floor renders white)
GLASS = (0.80, 0.86, 0.84)
LED_GREEN = (0.10, 0.75, 0.25)


def five_leg_base(p, name, cx, cy, leg, z=0.07, caster_r=0.022):
    """Wheeled 5-star base (IV pole, monitor stand): hub, 5 legs, 5 casters."""
    p.cyl(name + '_hub', (cx, cy, z), 0.04, 0.05, STEEL_DARK)
    for k in range(5):
        a = math.pi / 2 + k * 2 * math.pi / 5
        ex, ey = cx + leg * math.cos(a), cy + leg * math.sin(a)
        p.cyl_between('%s_leg%d' % (name, k), (cx, cy, z), (ex, ey, z - 0.025), 0.012, STEEL)
        p.cyl('%s_caster%d' % (name, k), (ex, ey, caster_r, math.pi / 2, 0, a), caster_r, 0.02, DARK)


# ---------------------------------------------------------------- bed
def bed_model():
    p = Parts('hospital_bed')
    cx, w = BED_CX, BED_W
    # wheeled base frame + two telescopic lift columns (height-adjustable look)
    base_y0, base_y1 = BED_Y0 + 0.12, BED_Y1 - 0.12
    for i, (wx, wy) in enumerate(((BED_X0 + 0.13, base_y0 + 0.02), (BED_X1 - 0.13, base_y0 + 0.02),
                                  (BED_X0 + 0.13, base_y1 - 0.02), (BED_X1 - 0.13, base_y1 - 0.02))):
        p.cyl('caster%d' % i, (wx, wy, 0.05, 0, math.pi / 2, 0), 0.05, 0.035, DARK)
    p.box('base_frame', (cx, (base_y0 + base_y1) / 2, 0.125), (w - 0.20, base_y1 - base_y0, 0.05), FRAME_GREY)
    col_h0, col_h1 = 0.15, DECK_Z0
    for i, cy in enumerate((BED_Y0 + 0.45, BED_Y1 - 0.45)):
        p.box('lift_lower%d' % i, (cx, cy, col_h0 + 0.06), (0.18, 0.12, 0.12), STEEL_DARK)
        p.box('lift_upper%d' % i, (cx, cy, (col_h0 + 0.12 + col_h1) / 2), (0.13, 0.09, col_h1 - col_h0 - 0.12), STEEL)
    # deck frame and mattress: flat leg section + raised back section (35 deg)
    p.box('deck', (cx, (BED_Y0 + BED_Y1) / 2, DECK_Z0 + 0.025), (w + 0.01, BED_L + 0.02, 0.05), FRAME_GREY)
    zm = MATT_TOP - MATT_T / 2
    p.box('mattress_legs', (cx, (HINGE_Y + BED_Y1) / 2, zm), (w, BED_Y1 - HINGE_Y, MATT_T), WHITE)
    back_len = HINGE_Y - BED_Y0
    c, s = math.cos(BACK_TILT), math.sin(BACK_TILT)
    p.box('mattress_back', (cx, HINGE_Y - back_len / 2 * c, zm + back_len / 2 * s, -BACK_TILT, 0, 0),
          (w, back_len, MATT_T), WHITE)
    # pillow on the raised back section, near its head end
    d = back_len - 0.20
    p.box('pillow', (cx, HINGE_Y - d * c + 0.05 * s, MATT_TOP + d * s + 0.05 * c, -BACK_TILT, 0, 0),
          (0.62, 0.36, 0.10), (0.84, 0.84, 0.82))
    # blanket over the leg section (draped over the sides) and the turned-down sheet
    by0, by1 = HINGE_Y + 0.04, BED_Y1 - 0.03
    p.box('blanket', (cx, (by0 + by1) / 2, MATT_TOP - 0.035), (w + 0.04, by1 - by0, 0.11), BLANKET)
    p.box('sheet_fold', (cx, by0 + 0.06, MATT_TOP - 0.032), (w + 0.05, 0.12, 0.118), (0.96, 0.96, 0.95))
    # head- and footboard (wood laminate with a grey top rail), patient chart on the footboard
    p.box('headboard', (cx, BED_Y0 - 0.03, 0.71), (w + 0.06, 0.03, 0.58), WOOD)
    p.cyl('headboard_rail', (cx, BED_Y0 - 0.03, 1.01, 0, math.pi / 2, 0), 0.02, w + 0.08, FRAME_GREY)
    p.box('footboard', (cx, BED_Y1 + 0.03, 0.64), (w + 0.06, 0.03, 0.42), WOOD)
    p.cyl('footboard_rail', (cx, BED_Y1 + 0.03, 0.86, 0, math.pi / 2, 0), 0.02, w + 0.08, FRAME_GREY)
    p.box('chart', (cx + 0.18, BED_Y1 + 0.052, 0.62), (0.22, 0.012, 0.28), (0.95, 0.95, 0.92))
    p.box('chart_clip', (cx + 0.18, BED_Y1 + 0.06, 0.755), (0.08, 0.008, 0.025), DARK)

    # split side rails (U frames): far side head + foot; table side HEAD ONLY and low,
    # so the patient reaches the raised table over / past it
    def rail(name, x, y0, y1, top):
        bottom = MATT_TOP - 0.10
        p.cyl(name + '_top', (x, (y0 + y1) / 2, top, -math.pi / 2, 0, 0), 0.013, y1 - y0, STEEL)
        for k, y in enumerate((y0, y1)):
            p.cyl('%s_post%d' % (name, k), (x, y, (top + bottom) / 2), 0.011, top - bottom, STEEL)
    rail('rail_far_head', BED_X0 - 0.025, BED_Y0 + 0.06, BED_Y0 + 0.58, 0.88)
    rail('rail_far_foot', BED_X0 - 0.025, BED_Y0 + 0.92, BED_Y0 + 1.72, 0.85)
    rail('rail_table_head', BED_X1 + 0.025, BED_Y0 + 0.06, BED_Y0 + 0.58, RAIL_TABLE_SIDE_TOP)
    # one collision box: deck + mattress (nothing else can touch the bed)
    p.collision('bed', (cx, (BED_Y0 + BED_Y1) / 2, (DECK_Z0 + MATT_TOP) / 2),
                (w + 0.01, BED_L + 0.02, MATT_TOP - DECK_Z0))
    return static_model('hospital_bed', p)


# ---------------------------------------------------------------- IV pole, monitor, chair
def iv_pole_model():
    p = Parts('iv_pole')
    x, y = IV_XY
    five_leg_base(p, 'base', x, y, 0.25)
    p.cyl('pole', (x, y, 0.07 + 1.83 / 2), 0.012, 1.83, STEEL, spec=0.5)
    p.cyl('hook_bar', (x + 0.08, y, 1.86, 0, math.pi / 2, 0), 0.006, 0.20, STEEL)
    p.sph('hook_end', (x + 0.18, y, 1.865), 0.009, STEEL)
    bx = x + 0.17
    p.box('iv_bag', (bx, y, 1.72), (0.11, 0.035, 0.20), (0.93, 0.95, 0.88), transparency=0.25)
    p.box('iv_label', (bx, y + 0.0185, 1.74), (0.07, 0.002, 0.06), WHITE)
    p.cyl('bag_port', (bx, y, 1.60), 0.008, 0.04, (0.92, 0.92, 0.90))
    p.cyl('drip_chamber', (bx, y, 1.55), 0.012, 0.06, (0.90, 0.93, 0.92), transparency=0.3)
    p.cyl_between('drip_line', (bx, y, 1.52), (BED_X0 + 0.12, y + 0.13, MATT_TOP + 0.03), 0.0035,
                  (0.92, 0.93, 0.90))
    return static_model('iv_pole', p)


def monitor_model():
    p = Parts('patient_monitor')
    x, y = MONITOR_XY
    five_leg_base(p, 'base', x, y, 0.24)
    p.cyl('pole', (x, y, 0.07 + 1.16 / 2), 0.02, 1.16, STEEL)
    p.box('mount', (x + 0.02, y, 1.24), (0.06, 0.08, 0.06), STEEL_DARK)
    # dark housing facing +x (towards the bed and the room), screen texture: green ECG,
    # yellow SpO2, white respiration/blood pressure (no red or blue)
    p.box('housing', (x + 0.07, y, 1.37), (0.08, 0.38, 0.30), (0.13, 0.13, 0.14))
    p.box('screen', (x + 0.1125, y, 1.375), (0.005, 0.33, 0.25), (1.0, 1.0, 1.0), spec=0.3,
          texture=SCREEN_TEX, amb=1.0)
    p.box('handle', (x + 0.07, y, 1.535), (0.03, 0.22, 0.02), STEEL_DARK)
    p.box('power_led', (x + 0.1125, y + 0.17, 1.24), (0.004, 0.012, 0.008), LED_GREEN, emissive=LED_GREEN)
    return static_model('patient_monitor', p)


def chair_model():
    """Visitor chair with armrests; local +x = the way the seated visitor faces."""
    p = Parts('visitor_chair')
    p.box('seat', (0, 0, 0.44), (0.48, 0.48, 0.08), UPHOLSTERY)
    p.box('backrest', (-0.25, 0, 0.75, 0, -0.10, 0), (0.06, 0.48, 0.52), UPHOLSTERY)
    for i, (lx, ly) in enumerate(((0.20, 0.20), (0.20, -0.20), (-0.22, 0.20), (-0.22, -0.20))):
        p.cyl('leg%d' % i, (lx, ly, 0.20), 0.016, 0.40, CHAIR_WOOD)
    for i, sy in enumerate((0.27, -0.27)):
        p.box('arm%d' % i, (-0.02, sy, 0.66), (0.44, 0.06, 0.035), CHAIR_WOOD)
        p.box('arm_post%d' % i, (0.16, sy, 0.555), (0.035, 0.035, 0.18), CHAIR_WOOD)
    return static_model('visitor_chair', p, (CHAIR_XY[0], CHAIR_XY[1], 0, 0, 0, CHAIR_YAW))


def room_model():
    """Floor patch and the two back walls (window, whiteboard, clock); the open sides
    face the track and the overview camera."""
    p = Parts('patient_room')
    fx0, fx1, fy0 = WALL_X, BED_X1 + 0.045, 0.55       # ends under the table base, out of the camera's strips
    p.box('floor', ((fx0 + fx1) / 2, (fy0 + WALL_Y) / 2, 0.001), (fx1 - fx0, WALL_Y - fy0, 0.002), ROOM_FLOOR,
          spec=0.05)
    wy0, wx1 = -0.65, 2.45
    p.box('wall_side', (WALL_X - 0.025, (wy0 + WALL_Y + 0.05) / 2, WALL_H / 2), (0.05, WALL_Y + 0.05 - wy0, WALL_H),
          WALL, spec=0.02)
    p.box('wall_back', ((WALL_X - 0.05 + wx1) / 2, WALL_Y + 0.025, WALL_H / 2), (wx1 - WALL_X + 0.05, 0.05, WALL_H),
          WALL, spec=0.02)
    p.box('skirting_side', (WALL_X + 0.006, (wy0 + WALL_Y) / 2, 0.05), (0.012, WALL_Y - wy0, 0.10), SKIRTING)
    p.box('skirting_back', ((WALL_X + wx1) / 2, WALL_Y - 0.006, 0.05), (wx1 - WALL_X, 0.012, 0.10), SKIRTING)
    p.box('window_frame', (1.70, WALL_Y - 0.012, 1.30), (0.95, 0.025, 0.80), WHITE)
    p.box('window_glass', (1.70, WALL_Y - 0.025, 1.30), (0.85, 0.004, 0.70), GLASS, spec=0.5)
    p.box('whiteboard_frame', (WALL_X + 0.010, 1.20, 1.35), (0.018, 0.74, 0.52), FRAME_GREY)
    p.box('whiteboard', (WALL_X + 0.020, 1.20, 1.35), (0.006, 0.68, 0.46), (0.97, 0.97, 0.97), spec=0.3)
    p.cyl('clock', (0.35, WALL_Y - 0.015, 1.70, math.pi / 2, 0, 0), 0.13, 0.03, WHITE)
    p.cyl('clock_rim', (0.35, WALL_Y - 0.010, 1.70, math.pi / 2, 0, 0), 0.142, 0.02, DARK)
    p.box('clock_hour', (0.35 + 0.03, WALL_Y - 0.032, 1.71, 0, -0.5, 0), (0.07, 0.003, 0.010), DARK)
    p.box('clock_min', (0.35 - 0.012, WALL_Y - 0.034, 1.745), (0.008, 0.003, 0.10), DARK)
    return static_model('patient_room', p)


# ---------------------------------------------------------------- smart table
def _inertial(m, ixx, iyy, izz, z=0.0):
    return ('<inertial><pose>0 0 %.4f 0 0 0</pose><mass>%s</mass><inertia><ixx>%.3g</ixx><iyy>%.3g</iyy>'
            '<izz>%.3g</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>' % (z, m, ixx, iyy, izz))


def smart_table_model():
    """Base fixed to the world, N nested tubes on prismatic joints in series (lift1..liftN),
    the platform on the innermost tube. Model origin: on the floor under the column axis
    (= platform centre). Lowered: every tube's top just under the platform, the rest
    hangs below the floor (visual only, no collision), so only a short stepped neck
    shows between the base and the platform. Each stage travels TRAVEL / N."""
    n = tbl.N_STAGES
    stroke = tbl.TRAVEL / n
    plate_t = 0.03
    plate_z0 = tbl.LOWERED_TOP - plate_t                   # platform bottom when lowered
    tube_len = stroke + 0.025                              # 2.5 cm overlap when extended
    radii = [0.040 - 0.008 * i for i in range(n)]
    # joint limits 3 cm beyond the travel: DART skips the velocity servo of a joint that
    # sits at a position limit, so stages that sag onto a limit at start-up stayed there
    # (seen: 4 / 8 / 12 mm below 0, platform at 0.077 m). The controller keeps 0..stroke.
    shades = [STEEL, STEEL_DARK]
    bx = tbl.PLATFORM_ROBOT_EDGE_X - TABLE_BASE[0] / 2 - tbl.PLATFORM_CENTER[0]
    links, joints = [], []
    base = Parts('smart_table')
    base.box('base', (bx, 0, TABLE_BASE[2] / 2), TABLE_BASE, MATTE_WHITE, spec=0.03)
    base.box('status_led', (bx + TABLE_BASE[0] / 2 - 0.012, 0, TABLE_BASE[2] + 0.0005), (0.01, 0.08, 0.002),
             LED_GREEN, emissive=LED_GREEN)          # on top, not sticking out towards the robot
    base.collision('base_collision', (bx, 0, TABLE_BASE[2] / 2 + 0.0005), (TABLE_BASE[0], TABLE_BASE[1], TABLE_BASE[2] - 0.001))
    links.append('\n      <link name="base">%s%s\n      </link>'
                 % (_inertial(5.0, 0.04, 0.045, 0.08, 0.025), base))
    joints.append('\n      <joint name="world_fix" type="fixed"><parent>world</parent><child>base</child></joint>')
    parent = 'base'
    for i in range(n):
        name = 'platform' if i == n - 1 else 'stage%d' % (i + 1)
        top = plate_z0 - 0.005 * (n - 1 - i)               # lowered: tops stepped 5 mm
        p = Parts('smart_table')
        r = radii[i]
        p.cyl('tube', (0, 0, top - tube_len / 2), r, tube_len, shades[i % 2], spec=0.4)
        if i == n - 1:
            m, inert = 1.0, _inertial(1.0, 5.3e-3, 5.3e-3, 1.04e-2, plate_z0 + plate_t / 2)
            p.box('plate', (0, 0, plate_z0 + plate_t / 2), tbl.PLATFORM_SIZE + (plate_t,), (0.78, 0.79, 0.81),
                  spec=0.05)
            p.box('sensor_pad', (0, 0, tbl.LOWERED_TOP - 0.0002), (0.21, 0.21, 0.001), (0.62, 0.72, 0.66))
            p.collision('top', (0, 0, plate_z0 + plate_t / 2), tbl.PLATFORM_SIZE + (plate_t,), mu=1.0)
            sensor = """
        <sensor name="platform_contact" type="contact">
          <contact><collision>top</collision><topic>/smart_table/contact</topic></contact>
          <topic>/smart_table/contact</topic>
          <always_on>1</always_on>
          <update_rate>50</update_rate>
        </sensor>"""
        else:
            inert = _inertial(0.3, 1.1e-3, 1.1e-3, 2e-4, top - tube_len / 2)
            sensor = ''
        links.append('\n      <link name="%s">%s%s%s\n      </link>' % (name, inert, p, sensor))
        joints.append("""
      <joint name="lift%d" type="prismatic">
        <parent>%s</parent><child>%s</child>
        <axis><xyz>0 0 1</xyz><limit><lower>%.4f</lower><upper>%.4f</upper><effort>500</effort><velocity>1</velocity></limit>
          <dynamics><damping>0</damping><friction>0</friction></dynamics></axis>
      </joint>""" % (i + 1, parent, name, -LIMIT_MARGIN, stroke + LIMIT_MARGIN))
        parent = name
    ctrl = ''.join("""
      <plugin filename="ignition-gazebo-joint-position-controller-system" name="ignition::gazebo::systems::JointPositionController">
        <joint_name>lift%d</joint_name>
        <topic>/smart_table/lift%d/cmd_pos</topic>
        <use_velocity_commands>true</use_velocity_commands>
        <cmd_max>%s</cmd_max>
        <cmd_min>-%s</cmd_min>
        <initial_position>0</initial_position>
      </plugin>""" % (i + 1, i + 1, tbl.STAGE_CMD_MAX, tbl.STAGE_CMD_MAX) for i in range(n))
    return """
    <!-- smart bedside table: controller scripts/smart_table.py (started by hospital.launch.py) -->
    <model name="smart_table">
      %s%s%s%s
      <plugin filename="ignition-gazebo-joint-state-publisher-system" name="ignition::gazebo::systems::JointStatePublisher">
        <topic>/smart_table/joint_states</topic>
      </plugin>
    </model>""" % (_pose((tbl.PLATFORM_CENTER[0], tbl.PLATFORM_CENTER[1], 0)), ''.join(links), ''.join(joints), ctrl)


# ---------------------------------------------------------------- cameras
def camera_model(name, cam, look, rate, topic, w=800, h=500, hfov=1.15):
    """Fixed camera rendered by the server's Sensors system (works with gui:=false)."""
    dx, dy, dz = (look[i] - cam[i] for i in range(3))
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(-dz, math.hypot(dx, dy))   # +pitch = nose down for a camera's +x
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{cam[0]} {cam[1]} {cam[2]} 0 {pitch:.4f} {yaw:.4f}</pose>
      <link name="link">
        <sensor name="{name}" type="camera">
          <camera>
            <horizontal_fov>{hfov}</horizontal_fov>
            <image><width>{w}</width><height>{h}</height><format>R8G8B8</format></image>
            <clip><near>0.1</near><far>20</far></clip>
          </camera>
          <always_on>1</always_on>
          <update_rate>{rate}</update_rate>
          <topic>{topic}</topic>
        </sensor>
      </link>
    </model>"""


def overview_camera(rate=0.5):
    """Fixed camera over the whole track and the room. Low rate on purpose: it is for
    screenshots (tools/sim_screenshot.py); hospital_video.sdf has it at 2 Hz.
    Topic /overview_cam/image, bridged in hospital.launch.py."""
    return camera_model('overview_cam', (3.6, -1.2, 2.3), (0.5, 1.3, 0.5), rate, '/overview_cam/image', hfov=1.1)


# report pictures (hospital_photo.sdf only): name -> (camera, look-at point)
ROOM_CAMS = {
    'a': ((3.4, -1.4, 1.9), (0.5, 1.0, 0.6)),         # room from the open corner
    'b': ((-0.45, 2.75, 1.95), (1.30, 1.45, 0.55)),   # patient's side: across the bed (lowered table hidden by it)
    'c': ((2.8, 1.2, 1.2), (1.38, 1.5, 0.4)),         # smart table close-up (lowered and raised in frame)
}


ROOM_HFOV = {'a': 1.1, 'b': 1.05, 'c': 0.85}


def room_cameras():
    return ''.join(camera_model('room_cam_' + k, c, l, 0.25, '/room_cam/%s/image' % k, 1280, 800, ROOM_HFOV[k])
                   for k, (c, l) in ROOM_CAMS.items())


# ---------------------------------------------------------------- checks
def check_colours():
    """No furniture colour the line follower could take for the line (blue) or a stop
    marker (red), whatever its saturation, if it has a visible hue (S >= 0.1)."""
    bad = []
    for model, name, rgb in COLOURS:
        if model == 'floor_line':
            continue
        h, s, v = colorsys.rgb_to_hsv(*rgb[:3])
        hue = h * 180.0
        if s < 0.10:
            continue                                   # grey / white: no hue
        in_blue = BLUE_HUE[0] <= hue <= BLUE_HUE[1]
        in_red = any(lo <= hue <= hi for lo, hi in RED_HUE)
        if in_blue or in_red:
            bad.append('%s/%s rgb %s hue %.0f sat %.2f' % (model, name, rgb, hue, s))
    assert not bad, 'colours the line follower may detect:\n  ' + '\n  '.join(bad)


def _seg_dist(p, a, b):
    ax, ay = b[0] - a[0], b[1] - a[1]
    t = max(0.0, min(1.0, ((p[0] - a[0]) * ax + (p[1] - a[1]) * ay) / (ax * ax + ay * ay)))
    return math.hypot(p[0] - a[0] - t * ax, p[1] - a[1] - t * ay)


def route_points():
    pts = [(HOME_X, 0.0), (X_TURN, 0.0)]
    for i in range(1, 31):
        a = -math.pi / 2 + (math.pi / 2) * i / 30
        pts.append((X_TURN + R_TURN * math.cos(a), R_TURN + R_TURN * math.sin(a)))
    pts.append((XS, Y_END))
    return pts


def footprints():
    """Floor-plan outlines (sampled points) of everything near the route."""
    def rect(x0, x1, y0, y1, n=12):
        return ([(x0 + (x1 - x0) * i / n, y) for i in range(n + 1) for y in (y0, y1)] +
                [(x, y0 + (y1 - y0) * i / n) for i in range(n + 1) for x in (x0, x1)])

    def circle(cx, cy, r, n=24):
        return [(cx + r * math.cos(2 * math.pi * i / n), cy + r * math.sin(2 * math.pi * i / n)) for i in range(n)]
    tx0 = tbl.PLATFORM_ROBOT_EDGE_X - TABLE_BASE[0]
    return {
        'smart table base': rect(tx0, tbl.PLATFORM_ROBOT_EDGE_X, tbl.PLATFORM_CENTER[1] - TABLE_BASE[1] / 2,
                                 tbl.PLATFORM_CENTER[1] + TABLE_BASE[1] / 2),
        'smart table platform': rect(tbl.PLATFORM_CENTER[0] - tbl.PLATFORM_SIZE[0] / 2, tbl.PLATFORM_ROBOT_EDGE_X,
                                     tbl.PLATFORM_CENTER[1] - tbl.PLATFORM_SIZE[1] / 2,
                                     tbl.PLATFORM_CENTER[1] + tbl.PLATFORM_SIZE[1] / 2),
        'bed': rect(BED_X0 - 0.045, BED_X1 + 0.045, BED_Y0 - 0.05, BED_Y1 + 0.07),
        'iv pole': circle(IV_XY[0], IV_XY[1], 0.28),
        'monitor': circle(MONITOR_XY[0], MONITOR_XY[1], 0.27),
        'chair': circle(CHAIR_XY[0], CHAIR_XY[1], 0.40),
        'side wall': rect(WALL_X - 0.05, WALL_X, -0.65, WALL_Y + 0.05),
        'back wall': rect(WALL_X - 0.05, 2.45, WALL_Y, WALL_Y + 0.05),
    }


def check_clearances():
    """Everything at least the robot's half width + 3.5 cm from the line (the old
    bedside table had 4 cm), and outside both turning circles (+1.5 cm)."""
    route = route_points()
    need = ROBOT_HALF_W + 0.035
    turns = {'patient turnaround': (XS + TURN_OFFSET, STOP_Y), 'home park turn': (HOME_X + STOP_AHEAD - 0.025, -TURN_OFFSET)}
    report, bad = [], []
    for name, pts in footprints().items():
        d = min(_seg_dist(p, a, b) for p in pts for a, b in zip(route, route[1:]))
        dt = {k: min(math.hypot(p[0] - c[0], p[1] - c[1]) for p in pts) for k, c in turns.items()}
        report.append('  %-22s line %.3f m   turn circles %s' % (name, d, ', '.join('%.3f' % v for v in dt.values())))
        if d < need - 1e-6:
            bad.append('%s only %.3f m from the line (need %.3f)' % (name, d, need))
        for k, v in dt.items():
            if v < TURN_R + 0.015:
                bad.append('%s %.3f m from the %s centre (need > %.3f)' % (name, v, k, TURN_R + 0.015))
    # robot-side rail must end before the table so the patient can reach it
    assert BED_Y0 + 0.58 < tbl.PLATFORM_CENTER[1] - TABLE_BASE[1] / 2 - 0.05
    assert RAIL_TABLE_SIDE_TOP < 0.75
    # platform clear of the bed (collision boxes) and standing between the bed and the robot
    assert tbl.PLATFORM_CENTER[0] - tbl.PLATFORM_SIZE[0] / 2 - (BED_X1 + 0.005) >= 0.03
    assert abs(tbl.PLATFORM_ROBOT_EDGE_X - (XS - (REACH - 0.03))) < 1e-9
    assert not bad, '\n'.join(bad)
    return report


# ---------------------------------------------------------------- monitor screen texture
def make_screen_texture(path, w=640, h=480):
    """Patient-monitor screen: green ECG + HR, yellow SpO2 pleth, white respiration and
    blood pressure (only green / yellow / white, no red or blue)."""
    import cv2
    import numpy as np
    img = np.full((h, w, 3), 8, np.uint8)
    green, yellow, white, dim = (40, 235, 40), (0, 225, 245), (235, 235, 235), (80, 80, 80)   # BGR
    font = cv2.FONT_HERSHEY_SIMPLEX

    def ecg(u):                     # one heartbeat, u in [0, 1)
        bumps = ((0.12, 0.035, 0.12), (0.255, 0.012, -0.12), (0.28, 0.012, 1.0), (0.305, 0.012, -0.25),
                 (0.52, 0.06, 0.25))
        return sum(a * math.exp(-((u - c) / s) ** 2) for c, s, a in bumps)

    def trace(y0, amp, fn, colour, period):
        pts = [(x, int(y0 - amp * fn((x % period) / period))) for x in range(10, 430)]
        cv2.polylines(img, [np.array(pts, np.int32)], False, colour, 2, cv2.LINE_AA)
    cv2.putText(img, 'BED 1   PATIENT A', (12, 28), font, 0.7, white, 1, cv2.LINE_AA)
    cv2.line(img, (0, 40), (w, 40), dim, 1)
    cv2.putText(img, 'II', (12, 70), font, 0.55, green, 1, cv2.LINE_AA)
    trace(160, 95, ecg, green, 140)
    cv2.putText(img, 'Pleth', (12, 215), font, 0.55, yellow, 1, cv2.LINE_AA)
    trace(300, 60, lambda u: math.exp(-((u - 0.3) / 0.12) ** 2) + 0.35 * math.exp(-((u - 0.62) / 0.1) ** 2),
          yellow, 140)
    cv2.putText(img, 'Resp', (12, 340), font, 0.55, white, 1, cv2.LINE_AA)
    trace(400, 25, lambda u: math.sin(2 * math.pi * u), white, 300)
    cv2.line(img, (440, 40), (440, h), dim, 1)
    for y, label, value, colour in ((60, 'HR', '72', green), (200, 'SpO2', '98', yellow), (330, 'RR', '16', white)):
        cv2.putText(img, label, (452, y + 20), font, 0.6, colour, 1, cv2.LINE_AA)
        cv2.putText(img, value, (470, y + 105), font, 3.0, colour, 6, cv2.LINE_AA)
    cv2.line(img, (0, 440), (w, 440), dim, 1)
    cv2.putText(img, 'NIBP 120/80 (93)   14:00', (12, 470), font, 0.75, white, 2, cv2.LINE_AA)
    cv2.imwrite(path, img)


# ---------------------------------------------------------------- world
def world(overview_rate=0.5, photo=False):
    # medicine station: robot's left at the start (container at y = REACH)
    st_edge = REACH - 0.03                    # table edge nearest the robot
    st = static_box_model('medicine_station', 0.0, st_edge + 0.15, 0.30, 0.30, PLATE_H, (0.86, 0.86, 0.84))  # neutral: no blue hue
    assert st_edge - ROBOT_HALF_W >= 0.035 and XS - ROBOT_HALF_W - tbl.PLATFORM_ROBOT_EDGE_X >= 0.035
    room = room_model() + bed_model() + iv_pole_model() + monitor_model() + chair_model() + smart_table_model()
    cams = overview_camera(overview_rate) + (room_cameras() if photo else '')
    return f"""<?xml version="1.0" ?>
<!-- generated by tools/make_hospital_world.py - edit that file, not this one -->
<sdf version="1.6">
  <world name="robot_world">
    <physics name="default" type="ignored">
      <max_step_size>0.02</max_step_size>
      <real_time_factor>1</real_time_factor>
    </physics>
    <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>
    <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"/>
    <!-- contact sensors (the smart table's platform) -->
    <plugin filename="ignition-gazebo-contact-system" name="ignition::gazebo::systems::Contact"/>
    <!-- renders the robot's camera; ogre2 does not work in this VMware VM -->
    <plugin filename="ignition-gazebo-sensors-system" name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre</render_engine>
    </plugin>
    <scene>
      <ambient>0.8 0.8 0.8 1</ambient>
      <background>0.8 0.8 0.8 1</background>
      <shadows>false</shadows>
    </scene>
    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.3 0.2 -0.9</direction>
    </light>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry>
          <material>
            <ambient>0.75 0.75 0.72 1</ambient>
            <diffuse>0.75 0.75 0.72 1</diffuse>
            <specular>0.05 0.05 0.05 1</specular>
          </material>
        </visual>
      </link>
    </model>{line_model()}{st}{room}{cams}
  </world>
</sdf>
"""


if __name__ == '__main__':
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, '..', 'robot_gazebo', 'worlds', 'hospital.sdf')
    make_screen_texture(SCREEN_TEX)
    COLOURS.clear()
    open(out, 'w').write(world())
    check_colours()
    report = check_clearances()
    # same world with the overview camera at 2 Hz for recording videos (launch video:=true)
    open(out[:-4] + '_video.sdf', 'w').write(world(overview_rate=2))
    # plus three 1280x800 room cameras for report pictures (launch photo:=true)
    open(out[:-4] + '_photo.sdf', 'w').write(world(photo=True))
    n_vis = open(out).read().count('<visual ')
    print('wrote', os.path.abspath(out), '+ _video.sdf, _photo.sdf and', os.path.basename(SCREEN_TEX))
    print('medicine container spawn  : x=0.000 y=%.3f z=%.3f' % (REACH, PLATE_H + 0.031))
    print('patient stop (approx)     : joint1 at x=%.3f y=%.3f, facing +y' % (XS, Y_END - STOP_AHEAD))
    print('place point (robot left)  : x=%.3f y=%.3f' % (XS - REACH, STOP_Y))
    print('smart table platform      : centre (%.3f, %.3f), %.2f x %.2f m, top %.2f -> %.2f m'
          % (tbl.PLATFORM_CENTER + tbl.PLATFORM_SIZE + (tbl.LOWERED_TOP, tbl.RAISED_TOP)))
    print('home stop (approx)        : joint1 at x=%.3f y=0, facing -x' % (HOME_X + STOP_AHEAD))
    print('visuals in hospital.sdf   : %d' % n_vis)
    print('clearances (need line >= %.3f, turn circles > %.3f):' % (ROBOT_HALF_W + 0.035, TURN_R + 0.015))
    print('\n'.join(report))
