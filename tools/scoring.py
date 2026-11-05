"""Scoring of one mission result (mission.py --result-json), shared by evaluate.py and
rescore.py so a run is always scored the same way from its raw values.

From the raw values in the JSON:
  placement_error_mm = horizontal distance container (Gazebo ground truth) -> target,
                       target = patient stop pose (chassis_link = under joint1) + R(yaw)
                       * (0, 0.20), i.e. the spot 0.20 m to the robot's LEFT
  on_table           = container centre z within 1 cm of 0.13 (table top 0.10 + half
                       the 0.06 m height); 0.03 means it lies on the floor
  tilt_deg           = angle container z axis <-> world z, from the raw quaternion
                       (normalised; missing protobuf components are 0)
  at_target          = placement_error_mm <= place_tol_mm (default 30). Needed: a
                       container left standing on the MEDICINE STATION is "on a
                       table" and upright too (run 4 of eval_5runs: lost attach)
  success            = mission completed AND on_table AND at_target AND tilt < max_tilt
                       AND (runs with the smart table) table_raised AND
                       container_on_platform_after_lift
Smart bedside table (mission.py since the hospital-room world; JSON 'placed' / 'table'):
  placement (error, on_table, tilt) is scored on the pose read the moment the
  platform's contact sensor confirmed the container, on the LOWERED table
  ('placed', placed_at = "contact sensor"); runs without it use the final pose as
  before (placed_at = "mission end"), so older results score exactly as they did.
  table_raised       = the table reported "raised" within the mission's timeout
  lift_time_s        = sim s from "lifting" to "raised"
  container_on_platform_after_lift = after the lift the container centre is over the
                       platform, z = platform top (measured) + 0.03 within 1 cm, and
                       tilt < max_tilt
"""
import math

TABLE_CENTRE_Z = 0.13
PLACE_TOL_MM = 30.0
REACH_LEFT = 0.20
CONTAINER_HALF_H = 0.03
TABLE_FIELDS = ['table_raised', 'lift_time_s', 'container_on_platform_after_lift']

FIELDS = ['run', 'started', 'started_utc', 'success', 'mission_ok', 'failed_step', 'error',
          'container_x', 'container_y', 'container_z',
          'container_qx', 'container_qy', 'container_qz', 'container_qw',
          'target_x', 'target_y', 'patient_stop_x', 'patient_stop_y', 'patient_stop_yaw_deg',
          'placement_error_mm', 'at_target', 'tilt_deg', 'tilt_source', 'on_table', 'placed_at',
          ] + TABLE_FIELDS + ['container_after_lift_z',
          'spawn_detach_attempts', 'pick_attach_attempts', 'place_detach_attempts',
          'weld_unconfirmed',
          'sim_time_s', 'mission_wall_s', 'total_wall_s', 'wall_time_valid',
          'base_final_x', 'base_final_y', 'base_final_yaw_deg', 'note']


def tilt_from_quat(q):
    qx, qy, qz, qw = q
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw) or 1.0
    qx, qy = qx / n, qy / n
    return math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (qx * qx + qy * qy)))))


def on_platform(xyz, quat, top, platform, max_tilt=10.0, z_tol=0.01):
    """Container centre over the platform rectangle, standing on it, upright."""
    (cx, cy), (sx, sy) = platform['center'], platform['size']
    inside = abs(xyz[0] - cx) <= sx / 2 and abs(xyz[1] - cy) <= sy / 2
    return bool(inside and abs(xyz[2] - (top + CONTAINER_HALF_H)) < z_tol and tilt_from_quat(quat) < max_tilt)


def score(res, max_tilt=10.0, mission_exit_ok=True, place_tol_mm=PLACE_TOL_MM):
    """res: dict from mission.py's JSON (may be empty). Returns a partial CSV row."""
    row = dict.fromkeys(FIELDS)
    row['mission_ok'] = bool(mission_exit_ok and res.get('success'))
    row['failed_step'] = res.get('failed_step')
    row['error'] = res.get('error')
    if res.get('base_final'):
        row['base_final_x'], row['base_final_y'], row['base_final_yaw_deg'] = res['base_final']
    ps = res.get('patient_stop')
    if ps:
        row['patient_stop_x'], row['patient_stop_y'], row['patient_stop_yaw_deg'] = ps
        yaw = math.radians(ps[2])
        target = (ps[0] - math.sin(yaw) * REACH_LEFT, ps[1] + math.cos(yaw) * REACH_LEFT)
    else:
        target = tuple(res['target']) if res.get('target') else None
    if target:
        row['target_x'], row['target_y'] = target
    placed = res.get('placed') or {}
    if placed.get('xyz'):           # smart table: pose when the contact sensor confirmed it
        c, q = placed['xyz'], placed.get('quat_xyzw')
        row['placed_at'] = placed.get('source')
    else:                           # older runs: final pose at the end of the mission
        c, q = res.get('container'), res.get('container_quat_xyzw')
        row['placed_at'] = 'mission end' if c else None
    if c:
        row['container_x'], row['container_y'], row['container_z'] = c
        row['on_table'] = abs(c[2] - TABLE_CENTRE_Z) < 0.01
        if target:
            row['placement_error_mm'] = 1000.0 * math.hypot(c[0] - target[0], c[1] - target[1])
            row['at_target'] = row['placement_error_mm'] <= place_tol_mm
    if q:
        row['container_qx'], row['container_qy'], row['container_qz'], row['container_qw'] = q
        row['tilt_deg'] = tilt_from_quat(q)
        row['tilt_source'] = 'raw quaternion'
    elif res.get('tilt_deg') is not None:
        row['tilt_deg'] = res['tilt_deg']
        row['tilt_source'] = 'recorded (no raw quaternion saved)'
    # confirmed attach/detach (mission.py 'weld' log; absent in runs before the fix)
    unconf = []
    for w in res.get('weld') or []:
        key = {'spawn': 'spawn_detach_attempts', 'pick': 'pick_attach_attempts',
               'place': 'place_detach_attempts'}.get(w.get('where'))
        if key:
            row[key] = w.get('attempts')
        if not w.get('confirmed'):
            unconf.append('%s %s' % (w.get('where'), w.get('kind')))
    if res.get('weld') is not None:
        row['weld_unconfirmed'] = ', '.join(unconf) or 'none'
    row['sim_time_s'] = res.get('sim_time')
    t = res.get('table')
    lift_ok = True
    if t is not None:               # smart table: lifted, and the container still on it
        row['table_raised'] = bool(t.get('raised'))
        row['lift_time_s'] = t.get('lift_time_s')
        cal = t.get('container_after_lift') or {}
        if cal.get('xyz') and t.get('platform'):
            top = t.get('height_after_lift')
            top = t['platform']['top_raised'] if top is None else top
            row['container_after_lift_z'] = cal['xyz'][2]
            row['container_on_platform_after_lift'] = on_platform(cal['xyz'], cal.get('quat_xyzw') or (0, 0, 0, 1),
                                                                  top, t['platform'], max_tilt)
        else:
            row['container_on_platform_after_lift'] = False
        lift_ok = row['table_raised'] and row['container_on_platform_after_lift']
    row['success'] = bool(row['mission_ok'] and row['on_table'] and row['at_target'] and
                          row['tilt_deg'] is not None and row['tilt_deg'] < max_tilt and lift_ok)
    return row
