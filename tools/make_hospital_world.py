#!/usr/bin/env python3
"""Generates robot_gazebo/worlds/hospital.sdf (run again after editing the layout).

Layout (metres, robot starts at the origin facing +x):
  - medicine station: low table on the robot's LEFT at the start
  - blue line: straight along +x, 90-degree left turn, straight along +y
  - red stop marker at the patient end, and one behind the start (used when
    the robot turns around and follows the line home)
  - patient bed with a low bedside table on the robot's LEFT at the stop
Table tops are at the arm-plate height (0.10 m) so the arm reaches them with
the same joint angles as the tested pick-and-place.
"""
import math, os, sys

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

def box_visual(name, x, y, z, yaw, sx, sy, sz, rgba, emissive=None):
    e = emissive or (0, 0, 0, 1)
    return f"""
      <visual name="{name}">
        <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 {yaw:.4f}</pose>
        <geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
        <material>
          <ambient>{rgba[0]} {rgba[1]} {rgba[2]} 1</ambient>
          <diffuse>{rgba[0]} {rgba[1]} {rgba[2]} 1</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
          <emissive>{e[0]} {e[1]} {e[2]} 1</emissive>
        </material>
      </visual>"""

def static_box_model(name, x, y, sx, sy, sz, rgba):
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x:.4f} {y:.4f} {sz/2:.4f} 0 0 0</pose>
      <link name="link">
        <collision name="c">
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
        </collision>
        {box_visual('v', 0, 0, 0, 0, sx, sy, sz, rgba)}
      </link>
    </model>"""

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
    xs = X_TURN + R_TURN
    y0, y1 = R_TURN, Y_END
    parts.append(box_visual('s2', xs, (y0 + y1) / 2, z, math.pi / 2, y1 - y0, LINE_W, LINE_T, blue, blue_e))
    # stop markers
    parts.append(box_visual('stop_patient', xs, Y_END, z + 0.0002, 0, 0.15, 0.15, LINE_T, red, red_e))
    parts.append(box_visual('stop_home', HOME_X, 0, z + 0.0002, 0, 0.15, 0.15, LINE_T, red, red_e))
    return f"""
    <model name="floor_line">
      <static>true</static>
      <link name="link">{''.join(parts)}
      </link>
    </model>"""

def overview_camera(rate=0.5):
    """Fixed camera looking over the whole track, rendered by the server's Sensors
    system (works with gui:=false). Low rate on purpose: it is only for screenshots
    (tools/sim_screenshot.py). Topic /overview_cam/image, bridged in hospital.launch.py."""
    cam = (3.0, -0.9, 2.1)             # camera position (+x side: bed is behind the track)
    look = (0.75, 0.80, 0.0)           # point it looks at (middle of the track)
    dx, dy, dz = (look[i] - cam[i] for i in range(3))
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(-dz, math.hypot(dx, dy))   # +pitch = nose down for a camera's +x
    return f"""
    <model name="overview_cam">
      <static>true</static>
      <pose>{cam[0]} {cam[1]} {cam[2]} 0 {pitch:.4f} {yaw:.4f}</pose>
      <link name="link">
        <sensor name="overview" type="camera">
          <camera>
            <horizontal_fov>1.15</horizontal_fov>
            <image><width>800</width><height>500</height><format>R8G8B8</format></image>
            <clip><near>0.1</near><far>20</far></clip>
          </camera>
          <always_on>1</always_on>
          <update_rate>{rate}</update_rate>
          <topic>/overview_cam/image</topic>
        </sensor>
      </link>
    </model>"""

def world(overview_rate=0.5):
    xs = X_TURN + R_TURN
    # medicine station: robot's left at the start (container at y = REACH)
    st_edge = REACH - 0.03                    # table edge nearest the robot
    st = static_box_model('medicine_station', 0.0, st_edge + 0.15, 0.30, 0.30, PLATE_H, (0.85, 0.85, 0.95))
    # at the patient stop the robot faces +y, its left is -x
    stop_y = Y_END - STOP_AHEAD               # arm joint1 when the robot stops
    bt_edge = xs - (REACH - 0.03)
    bedside = static_box_model('bedside_table', bt_edge - 0.15, stop_y, 0.30, 0.30, PLATE_H, (0.95, 0.9, 0.8))
    bed = static_box_model('patient_bed', bt_edge - 0.30 - 0.50, stop_y + 0.4, 0.95, 1.95, 0.50, (0.9, 0.95, 1.0))
    assert st_edge - ROBOT_HALF_W >= 0.035 and xs - ROBOT_HALF_W - bt_edge >= 0.035
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
    </model>{line_model()}{st}{bedside}{bed}{overview_camera(overview_rate)}
  </world>
</sdf>
"""

if __name__ == '__main__':
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        os.path.dirname(os.path.abspath(__file__)), '..', 'robot_gazebo', 'worlds', 'hospital.sdf')
    open(out, 'w').write(world())
    # same world with the overview camera at 2 Hz for recording videos (launch video:=true)
    out_video = out[:-4] + '_video.sdf'
    open(out_video, 'w').write(world(overview_rate=2))
    xs = X_TURN + R_TURN
    print('wrote', os.path.abspath(out), 'and', os.path.basename(out_video))
    print('medicine container spawn  : x=0.000 y=%.3f z=%.3f' % (REACH, PLATE_H + 0.031))
    print('patient stop (approx)     : joint1 at x=%.3f y=%.3f, facing +y' % (xs, Y_END - STOP_AHEAD))
    print('place point (robot left)  : x=%.3f y=%.3f' % (xs - REACH, Y_END - STOP_AHEAD))
    print('home stop (approx)        : joint1 at x=%.3f y=0, facing -x' % (HOME_X + STOP_AHEAD))
