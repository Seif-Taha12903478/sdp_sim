#!/usr/bin/env python3
"""Short Gazebo check of "arm home (carrying)" (run inside a fresh simulation):

    python3 ~/sdp_sim/tools/run_in_sim.py --name arm_home_held -- python3 ~/sdp_sim/tools/test_arm_home_held.py

pharmacy setup -> line up with S1 -> pick med_S1 (delivery.pick: ends at pick_lift) ->
(1) arm home carrying, normal; (2) back to pick_lift, then arm home carrying with the
joint-state cache FROZEN on purpose (subscription moved to a dead topic) -> must recover.
Checks: both 'reached'/'settled', container still held (z > 0.2), recovery logged.
"""
import json
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, 'scripts'))
import rclpy                                  # noqa: E402
from sensor_msgs.msg import JointState        # noqa: E402
import jetarm_pick_place as jp                # noqa: E402
import delivery as dv                         # noqa: E402
import pharmacy as ph                         # noqa: E402
from base_moves import BaseMover              # noqa: E402
from medicine_weld import Weld                # noqa: E402
from mission import gz_model_pose_full        # noqa: E402


def main():
    r = subprocess.run([sys.executable, '-u', os.path.join(ROOT, 'scripts', 'pharmacy.py'), 'setup'],
                       capture_output=True, text=True, timeout=3600)
    print(r.stdout[-400:], flush=True)
    if r.returncode != 0:
        sys.exit('pharmacy setup failed')
    rclpy.init()
    node = jp.PickPlace()
    base = BaseMover(node)
    weld = Weld(node, '/med_S1')
    pharm = ph.Pharmacy()
    res, ok = {}, True
    sx, sy = pharm.slots['S1']
    x, y, yaw, _ = dv.line_up(base, sx, sy - pharm.m['pharmacy']['reach_left'], 0.0)
    print('lined up at (%.4f, %.4f)' % (x, y), flush=True)
    dv.pick(node, weld, 'med_S1')
    _, plan = dv.arm_plan()
    for case in ('normal', 'frozen'):
        if case == 'frozen':
            print('back to pick_lift', flush=True)
            if node.arm(plan['pick_lift'], 3.0) is None:
                sys.exit('could not return to pick_lift')
            node.destroy_subscription(node._js_sub)          # simulate the frozen cache
            node._js_sub = node.create_subscription(JointState, '/frozen_on_purpose', node._on_js, 10)
            print('cache frozen on purpose', flush=True)
        s0, n0 = node.sim_t, node.cache_recoveries
        out = node.arm(jp.INIT_POSE, 3.0)
        rclpy.spin_once(node, timeout_sec=0.5)
        err = max(abs(a - b) for a, b in zip(node.get(jp.ARM_JOINTS), jp.INIT_POSE))
        c = gz_model_pose_full('med_S1')
        held = c is not None and c[0][2] > 0.2
        res[case] = {'result': out, 'sim_s': node.sim_t - s0, 'err_rad': err, 'container_z': c[0][2] if c else None,
                     'recoveries': node.cache_recoveries - n0}
        good = out in ('reached', 'settled') and held and (case == 'normal' or node.cache_recoveries > n0)
        ok = ok and good
        print('[check] %s %s: arm home %s in %.1f sim s, err %.4f rad, container z %s, recoveries %d'
              % ('PASS' if good else 'FAIL', case, out, res[case]['sim_s'], err,
                 '%.3f' % c[0][2] if c else '-', res[case]['recoveries']), flush=True)
    print('[result] %s %s' % ('PASS' if ok else 'FAIL', json.dumps(res)), flush=True)
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
