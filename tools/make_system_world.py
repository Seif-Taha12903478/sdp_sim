#!/usr/bin/env python3
"""Generates the hospital-system world (branch hospital-system) - edit THIS file, not the outputs.

    python3 ~/sdp_sim/tools/make_system_world.py      then  bash ~/sdp_sim/install.sh

Writes
  robot_gazebo/worlds/hospital_system.sdf        (+ _video.sdf: overview camera 2 Hz,
                                                    _photo.sdf: + report cameras)
  robot_gazebo/worlds/system/*.png               textures: room signs, pharmacy / dock signs,
                                                    patient-monitor screens, container labels
  robot_gazebo/worlds/system/med_S*.sdf          one container model per pharmacy slot
  robot_gazebo/urdf/jetarm_mecanum_system.gazebo.xacro   robot + one DetachableJoint per container
  config/hospital_map.yaml                       line network graph for the planner

Layout numbers: tools/system_layout.py. Every room is the tested room of
tools/make_hospital_world.py (same furniture functions, room coordinates) as ONE static
model per room, placed by its room pose; its smart table is a separate model with its
own topics (/smart_table_N/...). Primitives only (+ generated textures).

Checks (assert): colours (no blue / red, and no saturated green, outside the floor
markers: the line follower's line, stop-marker and junction-marker colours) and the
clearances of tools/system_layout.check() (line, every turning circle, routes < 6 m).
"""
import math
import os
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import make_hospital_world as W     # noqa: E402
import system_layout as L           # noqa: E402
tbl = W.tbl

WORLDS = os.path.join(ROOT, 'robot_gazebo', 'worlds')
TEX = os.path.join(WORLDS, 'system')            # absolute texture paths (worlds/ is not installed recursively)
XACRO_OUT = os.path.join(ROOT, 'robot_gazebo', 'urdf', 'jetarm_mecanum_system.gazebo.xacro')
MAP_OUT = os.path.join(ROOT, 'config', 'hospital_map.yaml')

GREEN_HUE = (40, 90)                # junction marker hue range (OpenCV) the colour check keeps free
GREEN_SAT = 0.40                    # ... for colours at least this saturated
AMBER = (1.0, 0.62, 0.05)           # status LEDs (green in the single room)
PATIENTS = {1: 'PATIENT A', 2: 'PATIENT B', 3: 'PATIENT C', 4: 'PATIENT D'}
Parts = W.Parts
COLOURS = W.COLOURS


# ---------------------------------------------------------------- textures
def _sign(path, lines, w=600, h=300, frame=(90, 90, 90), bg=(250, 250, 248), fg=(45, 45, 45)):
    import cv2
    import numpy as np
    img = np.full((h, w, 3), bg, np.uint8)
    cv2.rectangle(img, (6, 6), (w - 7, h - 7), frame, 10)
    font = cv2.FONT_HERSHEY_DUPLEX
    n = len(lines)
    for i, (txt, scale, th) in enumerate(lines):
        (tw, tht), _ = cv2.getTextSize(txt, font, scale, th)
        y = int(h * (i + 1) / (n + 1) + tht / 2)
        cv2.putText(img, txt, ((w - tw) // 2, y), font, scale, fg, th, cv2.LINE_AA)
    cv2.imwrite(path, img)
    return path


def make_label(path, medicine, dose, slot, cap_rgb):
    """Container label, wrapped round the cylinder: the text twice, so it reads from any side."""
    import cv2
    import numpy as np
    w, h = 1024, 256
    img = np.full((h, w, 3), (252, 252, 250), np.uint8)
    band = tuple(int(255 * c) for c in cap_rgb[::-1])
    for k in range(2):
        x0 = k * w // 2
        cv2.rectangle(img, (x0, 0), (x0 + w // 2 - 1, 34), band, -1)
        cv2.rectangle(img, (x0, h - 34), (x0 + w // 2 - 1, h - 1), band, -1)
        font = cv2.FONT_HERSHEY_DUPLEX
        for txt, y, scale, th in ((medicine.upper(), 112, 1.5, 3), (dose, 175, 1.4, 3), ('slot ' + slot, 215, 0.8, 2)):
            (tw, _), _ = cv2.getTextSize(txt, font, scale, th)
            cv2.putText(img, txt, (x0 + (w // 2 - tw) // 2, y), font, scale, (35, 35, 35), th, cv2.LINE_AA)
    img = cv2.flip(img, 0)          # Ogre's cylinder UVs run bottom -> top
    cv2.imwrite(path, img)
    return path


# ---------------------------------------------------------------- floor lines and markers
LINE_BLUE, LINE_BLUE_E = (0.0, 0.15, 0.9), (0.0, 0.05, 0.35)          # as in the single room
MARK_RED, MARK_RED_E = (0.9, 0.05, 0.05), (0.35, 0.0, 0.0)
MARK_GREEN, MARK_GREEN_E = (0.10, 0.72, 0.20), (0.03, 0.26, 0.07)     # OpenCV hue ~65


def floor_lines():
    z = W.LINE_T / 2 + 0.0005
    parts = []
    x0, x1 = L.JUNCTIONS['J2'], L.PHARM_MARK + L.MARKER / 2
    parts.append(W.box_visual('main', (x0 + x1) / 2, 0, z, 0, x1 - x0, L.LINE_W, W.LINE_T, LINE_BLUE, LINE_BLUE_E))
    for r in L.ROOMS:
        inf = L.room_info(r)
        jx = L.JUNCTIONS[inf['junction']]
        my = inf['marker'][1]
        ye = my + math.copysign(L.MARKER / 2, my)
        parts.append(W.box_visual('branch_R%d' % r, jx, ye / 2, z, math.pi / 2, abs(ye), L.LINE_W, W.LINE_T,
                                  LINE_BLUE, LINE_BLUE_E))
        parts.append(W.box_visual('stop_R%d' % r, jx, my, z + 0.0002, 0, L.MARKER, L.MARKER, W.LINE_T, MARK_RED, MARK_RED_E))
    for jn, x in L.JUNCTIONS.items():
        parts.append(W.box_visual('junction_%s' % jn, x, 0, z + 0.0002, 0, L.MARKER, L.MARKER, W.LINE_T,
                                  MARK_GREEN, MARK_GREEN_E))
    for nm, x in (('stop_pharmacy', L.PHARM_MARK), ('stop_home', L.HOME_MARK)):
        parts.append(W.box_visual(nm, x, 0, z + 0.0002, 0, L.MARKER, L.MARKER, W.LINE_T, MARK_RED, MARK_RED_E))
    return """
    <model name="floor_line">
      <static>true</static>
      <link name="link">%s
      </link>
    </model>""" % ''.join(parts)


# ---------------------------------------------------------------- patient room (one static model)
def room_shell(p, r, sign_tex):
    """Walls with the door, floor patch, window, whiteboard, clock, room sign (room coordinates)."""
    WALL, GLASS, FRAME, DARK, WHITE = W.WALL, W.GLASS, W.FRAME_GREY, W.DARK, W.WHITE
    h, t = L.WALL_H, 0.05
    y_c = L.R_HEAD - L.WALL_T                       # corridor face of the head wall
    p.box('floor', ((L.R_FAR + W.BED_X1 + 0.045) / 2, (L.R_HEAD + L.R_FOOT) / 2, 0.001),
          (W.BED_X1 + 0.045 - L.R_FAR, L.R_FOOT - L.R_HEAD, 0.002), W.ROOM_FLOOR, spec=0.05)
    walls = (('wall_far', L.R_FAR - t, L.R_FAR, y_c, L.R_FOOT + t),
             ('wall_near', L.R_NEAR, L.R_NEAR + t, y_c, L.R_FOOT + t),
             ('wall_foot', L.R_FAR - t, L.R_NEAR + t, L.R_FOOT, L.R_FOOT + t),
             ('wall_head_w', L.R_FAR - t, L.DOOR[0], y_c, L.R_HEAD),
             ('wall_head_e', L.DOOR[1], L.R_NEAR + t, y_c, L.R_HEAD))
    for nm, x0, x1, y0, y1 in walls:
        p.box(nm, ((x0 + x1) / 2, (y0 + y1) / 2, h / 2), (x1 - x0, y1 - y0, h), WALL, spec=0.02)
    p.box('door_lintel', ((L.DOOR[0] + L.DOOR[1]) / 2, (y_c + L.R_HEAD) / 2, 1.95),
          (L.DOOR[1] - L.DOOR[0], L.WALL_T, 0.10), WALL, spec=0.02)
    p.box('window_frame', (L.XS, L.R_FOOT - 0.012, 1.30), (0.95, 0.025, 0.80), WHITE)
    p.box('window_glass', (L.XS, L.R_FOOT - 0.025, 1.30), (0.85, 0.004, 0.70), GLASS, spec=0.5)
    p.box('whiteboard_frame', (L.R_FAR + 0.010, 1.20, 1.35), (0.018, 0.74, 0.52), FRAME)
    p.box('whiteboard', (L.R_FAR + 0.020, 1.20, 1.35), (0.006, 0.68, 0.46), (0.97, 0.97, 0.97), spec=0.3)
    cx = 0.35
    p.cyl('clock', (cx, L.R_FOOT - 0.015, 1.70, math.pi / 2, 0, 0), 0.13, 0.03, WHITE)
    p.cyl('clock_rim', (cx, L.R_FOOT - 0.010, 1.70, math.pi / 2, 0, 0), 0.142, 0.02, DARK)
    p.box('clock_hour', (cx + 0.03, L.R_FOOT - 0.032, 1.71, 0, -0.5, 0), (0.07, 0.003, 0.010), DARK)
    p.box('clock_min', (cx - 0.012, L.R_FOOT - 0.034, 1.745), (0.008, 0.003, 0.10), DARK)
    # room sign on the corridor face, beside the door; +x face of the thin box faces the corridor
    p.box('sign', (L.DOOR[1] + 0.20, y_c - 0.005, 1.45, 0, 0, -math.pi / 2), (0.008, 0.30, 0.15), (1, 1, 1),
          spec=0.2, texture=sign_tex, amb=1.0)


def room_model(r):
    name = 'room_%d' % r
    p = Parts(name)
    room_shell(p, r, os.path.join(TEX, 'sign_room_%d.png' % r))
    parts = [p, W.bed_parts(Parts(name, 'bed_')), W.iv_pole_parts(Parts(name, 'iv_')),
             W.monitor_parts(Parts(name, 'mon_'), os.path.join(TEX, 'monitor_room_%d.png' % r), led=AMBER),
             W.chair_parts(Parts(name, 'chair_', (W.CHAIR_XY[0], W.CHAIR_XY[1], W.CHAIR_YAW)))]
    px, py, yaw = L.room_pose(r)
    return W.static_model(name, ''.join(str(q) for q in parts), (px, py, 0, 0, 0, yaw))


# ---------------------------------------------------------------- pharmacy (cabinet + pick counter), corridor walls
BOX_COLOURS = [(0.95, 0.95, 0.93), (0.95, 0.92, 0.80), (0.98, 0.85, 0.35), (0.95, 0.65, 0.25),
               (0.62, 0.63, 0.65), (0.55, 0.42, 0.26), (0.85, 0.78, 0.65)]    # no blue / red / green hue


def pharmacy_model():
    p = Parts('pharmacy')
    x0, x1, y0, y1 = L.COUNTER
    top = L.PLATE_H
    xm = (x0 + x1) / 2
    # pick counter (the containers stand on it): collision = the whole counter block
    p.collision('counter_collision', (xm, (y0 + y1) / 2, top / 2), (x1 - x0, y1 - y0, top))
    p.box('counter', (xm, (y0 + y1) / 2, top / 2), (x1 - x0, y1 - y0, top), (0.86, 0.86, 0.84), spec=0.1)
    p.box('counter_edge', (xm, y0 + 0.006, top / 2), (x1 - x0, 0.012, top), (0.62, 0.63, 0.65))
    for s, x in L.SLOT_X.items():       # slot rings (dark grey discs under the containers)
        p.cyl('slot_%s' % s, (x, L.REACH, top + 0.0005), 0.022, 0.001, (0.45, 0.46, 0.48))
    # medication cabinet behind the counter: drawers below, glass-front shelves with boxes above
    cy0, cy1 = y1, y1 + L.CABINET_D
    cym, cw = (cy0 + cy1) / 2, x1 - x0
    body = (0.90, 0.90, 0.88)
    p.box('cab_left', (x0 + 0.01, cym, 0.95), (0.02, L.CABINET_D, 1.90), body)
    p.box('cab_right', (x1 - 0.01, cym, 0.95), (0.02, L.CABINET_D, 1.90), body)
    p.box('cab_back', (xm, cy1 - 0.01, 0.95), (cw, 0.02, 1.90), body)
    p.box('cab_top', (xm, cym, 1.89), (cw, L.CABINET_D, 0.02), body)
    p.box('cab_drawers', (xm, cy0 + 0.01, 0.30), (cw - 0.04, 0.02, 0.50), (0.80, 0.80, 0.78))
    for k, z in enumerate((0.18, 0.42)):
        p.box('cab_handle%d' % k, (xm, cy0 - 0.004, z), (0.18, 0.01, 0.015), W.STEEL_DARK)
    shelves = (0.56, 0.86, 1.16, 1.46)
    for k, z in enumerate(shelves):
        p.box('shelf%d' % k, (xm, cym, z), (cw - 0.04, L.CABINET_D - 0.03, 0.015), (0.82, 0.83, 0.84))
        n = 5
        for i in range(n):
            bw = (cw - 0.10) / n - 0.015
            bh = 0.10 + 0.05 * ((i + k) % 3)
            bx = x0 + 0.05 + (i + 0.5) * (cw - 0.10) / n
            col = BOX_COLOURS[(3 * k + i) % len(BOX_COLOURS)]
            p.box('box%d_%d' % (k, i), (bx, cym - 0.02, z + 0.0075 + bh / 2), (bw, 0.16, bh), col)
    p.box('cab_glass', (xm, cy0 + 0.004, 1.17), (cw - 0.04, 0.006, 1.38), W.GLASS, spec=0.5, transparency=0.6)
    p.box('sign', (xm, cy0 - 0.012, 1.96, 0, 0, -math.pi / 2), (0.008, 0.50, 0.12), (1, 1, 1), spec=0.2,
          texture=os.path.join(TEX, 'sign_pharmacy.png'), amb=1.0)
    # pharmacy room walls (its west wall = room 1's near wall + this half)
    t, h = 0.05, L.WALL_H
    cwall_y = (L.CORR_HALF, L.CORR_HALF + L.WALL_T)
    for nm, a0, a1, b0, b1 in (('wall_w', L.PHARM_X0 - t, L.PHARM_X0, L.CORR_HALF, L.PHARM_Y1 + t),
                               ('wall_e', L.CORR_X1, L.CORR_X1 + t, -L.CORR_HALF - L.WALL_T, L.PHARM_Y1 + t),
                               ('wall_back', L.PHARM_X0 - t, L.CORR_X1 + t, L.PHARM_Y1, L.PHARM_Y1 + t),
                               ('wall_corr_w', L.PHARM_X0, x0, cwall_y[0], cwall_y[1]),
                               ('wall_corr_e', x1, L.CORR_X1, cwall_y[0], cwall_y[1])):
        p.box(nm, ((a0 + a1) / 2, (b0 + b1) / 2, h / 2), (a1 - a0, b1 - b0, h), W.WALL, spec=0.02)
    # corridor west end + filler at the south-west corner; robot dock sign on the south wall at home
    xw = L.CORR_X0
    p.box('corr_west', (xw - 0.05, 0, h / 2), (0.10, 2 * (L.CORR_HALF + L.WALL_T), h), W.WALL, spec=0.02)
    p.box('corr_sw_fill', (xw + 0.025, -L.CORR_HALF - L.WALL_T / 2, h / 2), (0.05, L.WALL_T, h), W.WALL, spec=0.02)
    p.box('dock_sign', (L.HOME_POSE[0] - 0.08, -L.CORR_HALF + 0.005, 1.05, 0, 0, math.pi / 2), (0.008, 0.40, 0.16),
          (1, 1, 1), spec=0.2, texture=os.path.join(TEX, 'sign_dock.png'), amb=1.0)
    return W.static_model('pharmacy', p)


# ---------------------------------------------------------------- containers (spawned by scripts/pharmacy.py)
def container_sdf(slot, inv):
    cap = L.CAP_RGB[inv['cap']]
    label = os.path.join(TEX, 'label_%s.png' % slot)
    c = ' '.join('%.3f' % v for v in cap)
    return """<?xml version="1.0"?>
<!-- generated by tools/make_system_world.py: pharmacy slot %(slot)s, %(med)s %(dose)s -->
<sdf version="1.7">
  <model name="med_%(slot)s">
    <link name="body">
      <!-- same as scripts/medicine_container.sdf: 30 g, 30 mm x 60 mm cylinder -->
      <inertial>
        <mass>0.03</mass>
        <inertia><ixx>1.07e-5</ixx><iyy>1.07e-5</iyy><izz>3.4e-6</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
      </inertial>
      <collision name="collision">
        <geometry><cylinder><radius>0.015</radius><length>0.06</length></cylinder></geometry>
        <surface>
          <friction><ode><mu>1.5</mu><mu2>1.5</mu2></ode></friction>
          <bounce><restitution_coefficient>0.0</restitution_coefficient></bounce>
          <contact><ode><kp>1e6</kp><kd>1</kd><min_depth>0.001</min_depth></ode></contact>
        </surface>
      </collision>
      <visual name="bottle">
        <geometry><cylinder><radius>0.015</radius><length>0.06</length></cylinder></geometry>
        <material><ambient>0.9 0.9 0.9 1</ambient><diffuse>0.9 0.9 0.9 1</diffuse><specular>0.3 0.3 0.3 1</specular></material>
      </visual>
      <visual name="label">
        <pose>0 0 -0.004 0 0 0</pose>
        <geometry><cylinder><radius>0.0153</radius><length>0.034</length></cylinder></geometry>
        <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><specular>0.2 0.2 0.2 1</specular>
          <pbr><metal><albedo_map>%(label)s</albedo_map><metalness>0</metalness><roughness>1</roughness></metal></pbr></material>
      </visual>
      <visual name="cap">
        <pose>0 0 0.036 0 0 0</pose>
        <geometry><cylinder><radius>0.017</radius><length>0.012</length></cylinder></geometry>
        <material><ambient>%(c)s 1</ambient><diffuse>%(c)s 1</diffuse><specular>0.3 0.3 0.3 1</specular></material>
      </visual>
    </link>
  </model>
</sdf>
""" % {'slot': slot, 'med': inv['medicine'], 'dose': inv['dose'], 'label': label, 'c': c}


# ---------------------------------------------------------------- robot (+ one weld per container)
def robot_xacro(slots):
    plug = ''.join("""
    <plugin filename="libignition-gazebo-detachable-joint-system.so"
            name="ignition::gazebo::systems::DetachableJoint">
      <parent_link>link5</parent_link>
      <child_model>med_%(s)s</child_model>
      <child_link>body</child_link>
      <attach_topic>/med_%(s)s/attach</attach_topic>
      <detach_topic>/med_%(s)s/detach</detach_topic>
      <output_topic>/med_%(s)s/attached</output_topic>
      <suppress_child_warning>true</suppress_child_warning>
    </plugin>""" % {'s': s} for s in slots)
    return """<?xml version="1.0" encoding="utf-8"?>
<!-- generated by tools/make_system_world.py (hospital system): the robot of
     jetarm_mecanum.gazebo.xacro plus one DetachableJoint (assisted grasp weld) per
     pharmacy container med_<slot>. Each weld forms when its container FIRST appears
     (scripts/pharmacy.py spawns them after the robot and releases every weld at once,
     confirmed on /med_<slot>/attached). -->
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="robot">
  <xacro:include filename="$(find robot_gazebo)/urdf/jetarm_mecanum.gazebo.xacro"/>
  <gazebo>%s
  </gazebo>
</robot>
""" % plug


# ---------------------------------------------------------------- cameras
def top_camera(rate):
    """Straight-down overview of the whole building (north up), /overview_cam/image."""
    cx, cy, h = -2.22, 0.0, 7.8
    return """
    <model name="overview_cam">
      <static>true</static>
      <pose>%.3f %.3f %.3f 0 1.5708 1.5708</pose>
      <link name="link">
        <sensor name="overview_cam" type="camera">
          <camera>
            <horizontal_fov>1.1</horizontal_fov>
            <image><width>800</width><height>600</height><format>R8G8B8</format></image>
            <clip><near>0.1</near><far>20</far></clip>
          </camera>
          <always_on>1</always_on>
          <update_rate>%s</update_rate>
          <topic>/overview_cam/image</topic>
        </sensor>
      </link>
    </model>""" % (cx, cy, h, rate)


PHOTO_CAMS = {      # name -> (camera, look-at, hfov); /room_cam/<name>/image, 1280 x 800, photo world only
    'a': ((2.6, -5.2, 5.6), (-2.3, 0.4, 0.0), 1.05),       # whole building from the south-east
    'b': ((1.35, -0.40, 1.35), (-3.2, 0.10, 0.35), 1.10),  # down the corridor from the pharmacy end
    'c': ((0.27, -0.42, 0.50), (0.27, 0.32, 0.16), 0.95),  # pharmacy counter + containers
    'd': ((-0.30, 0.95, 1.55), (-1.20, 1.90, 0.45), 1.15),  # room 1: robot side of the bed, smart table
}


def photo_cameras():
    return ''.join(W.camera_model('room_cam_' + k, c, l, 0.25, '/room_cam/%s/image' % k, 1280, 800, hf)
                   for k, (c, l, hf) in PHOTO_CAMS.items())


# ---------------------------------------------------------------- map (planner)
def hospital_map(inv):
    nodes, edges = L.graph()
    out_nodes = {}
    for n, (x, y, kind, marker) in nodes.items():
        out_nodes[n] = {'x': round(x, 4), 'y': round(y, 4), 'type': kind, 'marker': marker}
    out_edges = []
    for a, b in edges:
        xa, ya = nodes[a][:2]
        xb, yb = nodes[b][:2]
        out_edges.append({'from': a, 'to': b, 'length': round(math.hypot(xb - xa, yb - ya), 4),
                          'heading_deg': round(math.degrees(math.atan2(yb - ya, xb - xa)), 2)})
    # explicit turn table at the junctions (the planner derives the same from the headings)
    turns = {}
    for jn in L.JUNCTIONS:
        nb = [e for e in out_edges if jn in (e['from'], e['to'])]
        tt = {}
        for e_in in nb:
            for e_out in nb:
                if e_in is e_out:
                    continue
                a = e_in['from'] if e_in['to'] == jn else e_in['to']
                b = e_out['to'] if e_out['from'] == jn else e_out['from']
                h_in = math.atan2(nodes[jn][1] - nodes[a][1], nodes[jn][0] - nodes[a][0])
                h_out = math.atan2(nodes[b][1] - nodes[jn][1], nodes[b][0] - nodes[jn][0])
                d = math.degrees(math.atan2(math.sin(h_out - h_in), math.cos(h_out - h_in)))
                tt['%s->%s' % (a, b)] = 'straight' if abs(d) < 1 else ('left' if d > 0 else 'right')
        turns[jn] = tt
    rooms = {}
    for r in L.ROOMS:
        i = L.room_info(r)
        rooms[r] = {'node': i['node'], 'junction': i['junction'], 'side': i['side'], 'table': i['table'],
                    'prefix': i['prefix'], 'platform_center': [round(v, 4) for v in i['platform_center']],
                    'platform_size': list(i['platform_size']), 'place_target': [round(v, 4) for v in i['place_target']],
                    'stop': [round(v, 4) for v in i['stop']], 'sign': 'Room %d' % r, 'patient_label': PATIENTS[r].title()}
    return {
        '_comment': 'generated by tools/make_system_world.py - edit tools/system_layout.py and regenerate',
        'stop_ahead': L.STOP_AHEAD, 'marker_size': L.MARKER, 'line_width': L.LINE_W,
        'home_pose': list(L.HOME_POSE), 'home_stop_x': round(L.HOME_STOP, 4),
        'turn_offset': L.TURN_OFFSET, 'turn_radius': L.TURN_R,
        'nodes': out_nodes, 'edges': out_edges, 'turns': turns, 'rooms': rooms,
        'pharmacy': {'node': 'pharmacy', 'stop_x': round(L.P_STOP, 4), 'counter_top': L.PLATE_H,
                     'reach_left': L.REACH, 'container_z': L.CONTAINER_Z,
                     'slots': {s: [L.SLOT_X[s], L.REACH] for s in L.SLOT_X},
                     'containers': {s: 'med_%s' % s for s in L.SLOT_X if s in inv}},
        'tables': {i['table']: {'prefix': i['prefix'], 'platform_center': [round(v, 4) for v in i['platform_center']]}
                   for i in (L.room_info(r) for r in L.ROOMS)},
    }


# ---------------------------------------------------------------- checks
def check_colours():
    """No blue / red (any saturation >= 0.1) and no saturated green outside the floor markers."""
    import colorsys
    bad = []
    for model, name, rgb in COLOURS:
        if model == 'floor_line':
            continue
        h, s, v = colorsys.rgb_to_hsv(*rgb[:3])
        hue = h * 180.0
        if s < 0.10:
            continue
        if W.BLUE_HUE[0] <= hue <= W.BLUE_HUE[1] or any(lo <= hue <= hi for lo, hi in W.RED_HUE):
            bad.append('%s/%s rgb %s hue %.0f sat %.2f (blue / red)' % (model, name, rgb, hue, s))
        if GREEN_HUE[0] <= hue <= GREEN_HUE[1] and s >= GREEN_SAT:
            bad.append('%s/%s rgb %s hue %.0f sat %.2f (junction green)' % (model, name, rgb, hue, s))
    assert not bad, 'colours the line follower may detect:\n  ' + '\n  '.join(bad)


def world(overview_rate=0.5, photo=False):
    rooms = ''.join(room_model(r) for r in L.ROOMS)
    tables = ''
    for r in L.ROOMS:
        i = L.room_info(r)
        cx, cy = i['platform_center']
        tables += W.smart_table_model(i['table'], i['prefix'], (cx, cy, math.radians(i['yaw_deg'])), led=AMBER)
    cams = top_camera(overview_rate) + (photo_cameras() if photo else '')
    return f"""<?xml version="1.0" ?>
<!-- generated by tools/make_system_world.py - edit that file (and tools/system_layout.py), not this one -->
<sdf version="1.6">
  <world name="robot_world">
    <physics name="default" type="ignored">
      <max_step_size>0.02</max_step_size>
      <real_time_factor>1</real_time_factor>
    </physics>
    <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>
    <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"/>
    <!-- contact sensors (the smart tables' platforms) -->
    <plugin filename="ignition-gazebo-contact-system" name="ignition::gazebo::systems::Contact"/>
    <!-- renders the cameras; ogre2 does not work in this VMware VM -->
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
          <geometry><plane><normal>0 0 1</normal><size>30 30</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>30 30</size></plane></geometry>
          <material>
            <ambient>0.75 0.75 0.72 1</ambient>
            <diffuse>0.75 0.75 0.72 1</diffuse>
            <specular>0.05 0.05 0.05 1</specular>
          </material>
        </visual>
      </link>
    </model>{floor_lines()}{pharmacy_model()}{rooms}{tables}{cams}
  </world>
</sdf>
"""


def main():
    os.makedirs(TEX, exist_ok=True)
    inv = L.load_inventory()
    assert inv, 'config/inventory.yaml missing or empty'
    for s in inv:
        assert s in L.SLOT_X, 'inventory slot %s does not exist (slots %s)' % (s, list(L.SLOT_X))
        assert inv[s]['cap'] in L.CAP_RGB, 'slot %s: unknown cap colour %s' % (s, inv[s]['cap'])
    # textures
    for r in L.ROOMS:
        _sign(os.path.join(TEX, 'sign_room_%d.png' % r), [('Room %d' % r, 3.2, 6)])
        W.make_screen_texture(os.path.join(TEX, 'monitor_room_%d.png' % r), title='ROOM %d   %s' % (r, PATIENTS[r]),
                              clock='08:00')
    _sign(os.path.join(TEX, 'sign_pharmacy.png'), [('PHARMACY', 3.0, 6)], w=1000, h=240)
    _sign(os.path.join(TEX, 'sign_dock.png'), [('ROBOT HOME', 2.2, 4), ('delivery robot dock', 1.1, 2)], w=1000, h=400)
    for s, d in inv.items():
        make_label(os.path.join(TEX, 'label_%s.png' % s), d['medicine'], d['dose'], s, L.CAP_RGB[d['cap']])
        with open(os.path.join(TEX, 'med_%s.sdf' % s), 'w') as fh:
            fh.write(container_sdf(s, d))
    # worlds
    COLOURS.clear()
    out = os.path.join(WORLDS, 'hospital_system.sdf')
    xml = world()
    # caps join the colour check (containers stand in the camera's view at the counter)
    for s, d in inv.items():
        COLOURS.append(('med_' + s, 'cap', L.CAP_RGB[d['cap']]))
    check_colours()
    with open(out, 'w') as fh:
        fh.write(xml)
    with open(out[:-4] + '_video.sdf', 'w') as fh:
        fh.write(world(overview_rate=2))
    with open(out[:-4] + '_photo.sdf', 'w') as fh:
        fh.write(world(photo=True))
    with open(XACRO_OUT, 'w') as fh:
        fh.write(robot_xacro(list(inv)))
    with open(MAP_OUT, 'w') as fh:
        yaml.safe_dump(hospital_map(inv), fh, sort_keys=False, default_flow_style=None)
    L.check(verbose=False)
    n_vis = xml.count('<visual ')
    print('wrote %s (+ _video, _photo), %d visuals (single room: 111)' % (out, n_vis))
    print('wrote %s, %s, %d container models + textures in %s' % (XACRO_OUT, MAP_OUT, len(inv), TEX))
    for r in L.ROOMS:
        i = L.room_info(r)
        print('room %d: stop (%.3f, %.3f, %+.0f deg), place target (%.3f, %.3f), platform centre (%.3f, %.3f)'
              % ((r,) + tuple(i['stop']) + tuple(i['place_target']) + tuple(i['platform_center'])))
    print('pharmacy stop x %.4f; slots %s' % (L.P_STOP, ', '.join('%s %.3f' % kv for kv in L.SLOT_X.items())))
    return n_vis


if __name__ == '__main__':
    main()
