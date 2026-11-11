#!/usr/bin/env python3
"""Pharmacy of the hospital system: inventory, slots, containers, weld release, stock.

    python3 pharmacy.py setup     # spawn every stocked container into its slot and release its weld
    python3 pharmacy.py status    # every container (Gazebo ground truth) vs. its slot

Needs a running hospital_system.launch.py. Inventory: ~/sdp_sim/config/inventory.yaml
(slot -> medicine, dose, cap colour, quantity 1 = a container stands there, 0 = empty);
slot positions: ~/sdp_sim/config/hospital_map.yaml (pick counter top 0.10 m, the
container 0.20 m left of the robot's joint1 when it stands on the line at the slot).

Every container med_<slot> has its own DetachableJoint in the robot model
(/med_<slot>/attach, /detach, /attached). A weld forms when its container FIRST
appears, so setup spawns the containers one by one (robot parked, arm still) and
releases each weld right away with the confirmed command (medicine_weld.Weld: resend
up to 5 times, wait for "detached" on sim time), then checks that the container rests
in its slot. Containers are spawned once per simulation and never re-spawned (a
re-spawned container is not welded again, and a name cannot exist twice).
"""
import math
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
MAP = os.path.join(ROOT, 'config', 'hospital_map.yaml')
INVENTORY = os.path.join(ROOT, 'config', 'inventory.yaml')
MODELS = os.path.join(ROOT, 'robot_gazebo', 'worlds', 'system')
WORLD = 'robot_world'
SLOT_TOL = 0.010            # m: "in its slot"
TABLE_Z = 0.13              # container centre z on a 0.10 m surface
# cap colour check: arm in the home pose turned left (joint1 +90 deg), the wrist camera sees
# the cap of the container at the robot's left (0.26 m from the lens) at about image (160, 121)
# (camera model of tools/system_layout.py); ROI x0, y0, x1, y1 - checked in tools/test_pharmacy_pick.py
CAP_ROI = (152, 113, 168, 129)
CAP_HSV = {                 # cap colour check (wrist camera): OpenCV HSV classes
    'white': lambda h, s, v: s < 45 and v >= 175,
    'grey': lambda h, s, v: s < 45 and 70 <= v < 175,
    'black': lambda h, s, v: v < 70,
    'yellow': lambda h, s, v: 22 <= h <= 36 and s >= 110 and v >= 90,
    'purple': lambda h, s, v: 135 <= h <= 165 and s >= 90 and v >= 60,   # was amber: rendered like yellow
}


def log(msg):
    print('[pharmacy] ' + msg, flush=True)


def load_yaml(path):
    import yaml
    with open(path) as fh:
        return yaml.safe_load(fh) or {}


def load_inventory(path=INVENTORY):
    out = {}
    for s, d in (load_yaml(path).get('slots') or {}).items():
        out[str(s)] = {'medicine': str(d['medicine']), 'dose': str(d['dose']), 'cap': str(d['cap']),
                       'quantity': int(d.get('quantity', 1))}
    return out


def container_name(slot):
    return 'med_' + slot


def model_file(slot):
    return os.path.join(MODELS, 'med_%s.sdf' % slot)


def tilt_deg(q):
    qx, qy, qz, qw = q
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw) or 1.0
    qx, qy = qx / n, qy / n
    return math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (qx * qx + qy * qy)))))


def gz_all_poses(prefix=None):
    """Gazebo ground truth of every model in one message: {name: ((x, y, z), (qx, qy, qz, qw))}.
    prefix: only names starting with it (e.g. 'med_'). Protobuf text output omits zeros."""
    t = subprocess.run(['ign', 'topic', '-e', '-n', '1', '-t', '/world/%s/pose/info' % WORLD],
                       capture_output=True, text=True, timeout=180).stdout
    out = {}
    for m in re.finditer(r'name: "([^"]+)"\s*id: \d+\s*position \{([^}]*)\}(?:\s*orientation \{([^}]*)\})?', t):
        name = m.group(1)
        if prefix and not name.startswith(prefix):
            continue

        def f(k, s):
            r = re.search(r'\b' + k + r': ([-\d.e+]+)', s or '')
            return float(r.group(1)) if r else 0.0
        pos = (f('x', m.group(2)), f('y', m.group(2)), f('z', m.group(2)))
        q = (0.0, 0.0, 0.0, 1.0) if m.group(3) is None else tuple(f(k, m.group(3)) for k in 'xyzw')
        out[name] = (pos, q)
    return out


class Pharmacy:
    """Slots, stock and the containers' welds (inside a ROS node)."""

    def __init__(self, node=None, m=None, inv=None):
        self.m = m or load_yaml(MAP)
        self.inv = inv if inv is not None else load_inventory()
        self.slots = {s: tuple(xy) for s, xy in self.m['pharmacy']['slots'].items()}
        for s in self.inv:
            if s not in self.slots:
                raise ValueError('inventory slot %s is not on the counter (%s)' % (s, ', '.join(self.slots)))
        self.node = node
        self.welds = {}
        if node is not None:
            from medicine_weld import Weld
            self.welds = {s: Weld(node, '/' + container_name(s)) for s in self.inv}
        self.weld_log = []

    # ------------------------------------------------------------ stock
    def stocked(self):
        return [s for s in self.slots if s in self.inv and self.inv[s]['quantity'] >= 1]

    def find(self, medicine, dose, used=()):
        """First stocked slot with this medicine + dose that is not used yet, or None."""
        for s in self.slots:
            d = self.inv.get(s)
            if d and d['quantity'] >= 1 and s not in used and \
                    d['medicine'].lower() == str(medicine).lower() and d['dose'].lower() == str(dose).lower():
                return s
        return None

    def in_slot(self, slot, pose):
        """Container pose ((x, y, z), q) standing upright in its slot?"""
        if pose is None:
            return False
        (x, y, z), q = pose
        sx, sy = self.slots[slot]
        return math.hypot(x - sx, y - sy) < SLOT_TOL and abs(z - TABLE_Z) < 0.01 and tilt_deg(q) < 10.0

    def status(self, poses=None):
        poses = gz_all_poses('med_') if poses is None else poses
        rows = []
        for s in self.slots:
            name = container_name(s)
            p = poses.get(name)
            d = self.inv.get(s, {})
            if p is None:
                where = 'not in the world'
            else:
                (x, y, z), q = p
                sx, sy = self.slots[s]
                where = '(%.4f, %.4f, %.4f) tilt %.1f deg, %.1f mm from the slot%s' % (
                    x, y, z, tilt_deg(q), 1000 * math.hypot(x - sx, y - sy), ' - IN SLOT' if self.in_slot(s, p) else '')
            rows.append((s, d.get('medicine', '-'), d.get('dose', ''), d.get('cap', ''), d.get('quantity', 0), name, where))
        return rows

    # ------------------------------------------------------------ setup
    def confirm(self, slot, kind, where):
        ok, n = self.welds[slot].command(kind)
        self.weld_log.append({'where': where, 'slot': slot, 'kind': kind, 'attempts': n, 'confirmed': ok,
                              'sim_t': self.welds[slot].sim_t})
        log('%s %s (%s) %s after %d attempt(s)' % (kind, container_name(slot), where,
                                                    'confirmed' if ok else 'NOT confirmed', n))
        return ok

    def spawn_all(self):
        """Spawn the stocked containers that are not in the world yet; release each weld."""
        poses = gz_all_poses('med_')
        for s in self.stocked():
            name = container_name(s)
            if name in poses:
                if self.in_slot(s, poses[name]):
                    log('%s already in slot %s - not spawned again' % (name, s))
                    continue
                raise RuntimeError('%s exists but is not in its slot %s: %s' % (name, s, poses[name]))
            x, y = self.slots[s]
            z = self.m['pharmacy']['container_z']
            w = self.welds[s]
            n0 = len(w.states)
            r = subprocess.run(['ros2', 'run', 'ros_ign_gazebo', 'create', '-file', model_file(s), '-name', name,
                                '-x', str(x), '-y', str(y), '-z', str(z)], capture_output=True, text=True, timeout=300)
            if r.returncode != 0:
                raise RuntimeError('spawn %s failed: %s' % (name, r.stderr.strip() or r.stdout.strip()))
            if w.wait_state('attached', since=n0, sim_seconds=3.0):
                log('%s spawned in slot %s (%.3f, %.3f): welded at first appearance - releasing' % (name, s, x, y))
                if not self.confirm(s, 'detach', 'spawn'):
                    log('WARNING: release of %s not confirmed - the pose check decides' % name)
            else:
                log('%s spawned: no weld state seen within 3 sim s - releasing anyway' % name)
                self.confirm(s, 'detach', 'spawn')
        poses = gz_all_poses('med_')
        bad = [s for s in self.stocked() if not self.in_slot(s, poses.get(container_name(s)))]
        for row in self.status(poses):
            log('%s %-12s %-7s %-6s qty %d  %s: %s' % row)
        if bad:
            raise RuntimeError('containers not resting in their slots after setup: %s' % ', '.join(bad))
        return poses


def classify_cap(bgr, roi):
    """Cap colour class from the wrist image: median HSV of the ROI (x0, y0, x1, y1)."""
    import cv2
    import numpy as np
    x0, y0, x1, y1 = roi
    hsv = cv2.cvtColor(bgr[y0:y1, x0:x1], cv2.COLOR_BGR2HSV).reshape(-1, 3)
    h, s, v = (int(np.median(hsv[:, i])) for i in range(3))
    for name, fn in CAP_HSV.items():
        if fn(h, s, v):
            return name, (h, s, v)
    return 'unknown', (h, s, v)


def main():
    if len(sys.argv) != 2 or sys.argv[1] not in ('setup', 'status'):
        print(__doc__)
        sys.exit(1)
    if sys.argv[1] == 'status':
        for row in Pharmacy().status():
            print('%s %-12s %-7s %-6s qty %d  %s: %s' % row)
        return
    import rclpy
    rclpy.init()
    node = rclpy.create_node('pharmacy_setup')
    code = 0
    try:
        Pharmacy(node).spawn_all()
        log('setup done')
    except RuntimeError as e:
        log('FAILED: %s' % e)
        code = 2
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
