#!/usr/bin/env python3
"""Stage 6: run the full mission N times, each from a fresh launch, and report.

    python3 ~/sdp_sim/tools/evaluate.py --runs 5 [--out ~/sdp_sim/results/eval_DATE]

Each run: check no simulation is running -> `ros2 launch robot_gazebo
hospital.launch.py gui:=false` (own process group, log in the output dir) -> wait
for the 3 controllers -> scripts/mission.py --result-json -> shut the launch down
and make sure no Gazebo process is left -> next run.

Per run (runs.csv, rewritten after every run so partial results survive):
success, mission result / failed step / error, container final position vs. the
bedside-table target (Gazebo ground truth), placement error, tilt from upright,
whether it stands on the table, sim time and wall time of the mission, and the
wall time including launch and shutdown.
success = mission completed AND container on the table AND tilt < --max-tilt deg.
summary.txt: success rate and mean / std / max of the numbers.

Needs a shell with ROS sourced and CAMERA_TYPE set (as in ~/.bashrc).
"""
import argparse
import csv
import datetime as dt
import json
import math
import os
import signal
import statistics as st
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MISSION = os.path.join(ROOT, 'scripts', 'mission.py')
FIELDS = ['run', 'started', 'success', 'mission_ok', 'failed_step', 'error',
          'container_x', 'container_y', 'container_z', 'target_x', 'target_y',
          'placement_error_mm', 'tilt_deg', 'on_table', 'sim_time_s', 'mission_wall_s',
          'total_wall_s', 'base_final_x', 'base_final_y', 'base_final_yaw_deg', 'note']


def log(msg):
    print('[evaluate %s] %s' % (dt.datetime.now().strftime('%H:%M:%S'), msg), flush=True)


def gazebo_pids():
    """PIDs of Gazebo / sim processes (by exact command, never this script's shell)."""
    out = subprocess.run(['ps', '-eo', 'pid=,args='], capture_output=True, text=True).stdout
    pids = []
    for line in out.splitlines():
        pid, _, args = line.strip().partition(' ')
        if (args.startswith('ign gazebo') or 'ruby' in args and 'ign gazebo' in args
                or args.startswith('/opt/ros/humble/lib/ros_gz_bridge/parameter_bridge')
                or args.startswith('/opt/ros/humble/lib/robot_state_publisher/')
                or 'planar_base.py' in args or 'line_follower.py' in args):
            pids.append(int(pid))
    return [p for p in pids if p != os.getpid()]


def wait_gone(pids, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        pids = [p for p in pids if os.path.exists('/proc/%d' % p)]
        if not pids:
            return []
        time.sleep(1)
    return pids


def start_sim(logpath):
    fh = open(logpath, 'w')
    proc = subprocess.Popen(['ros2', 'launch', 'robot_gazebo', 'hospital.launch.py', 'gui:=false'],
                            stdout=fh, stderr=subprocess.STDOUT, start_new_session=True)
    return proc, fh


def wait_ready(proc, logpath, timeout):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if proc.poll() is not None:
            return 'launch exited (code %s)' % proc.returncode
        with open(logpath, errors='replace') as fh:
            t = fh.read()
        if 'Segmentation fault' in t or 'process has died' in t:
            return 'a launched process died (see %s)' % logpath
        if 'activated' in t and 'gripper_controller' in t.split('Configured and activated')[-1]:
            return None
        time.sleep(3)
    return 'controllers not active after %.0f s' % timeout


def stop_sim(proc, fh):
    try:
        os.killpg(proc.pid, signal.SIGINT)
    except ProcessLookupError:
        pass
    try:
        proc.wait(timeout=60)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        proc.wait(timeout=30)
    fh.close()
    left = wait_gone(gazebo_pids(), 20)
    if left:
        log('killing leftovers %s' % left)
        for p in left:
            try:
                os.kill(p, signal.SIGKILL)
            except ProcessLookupError:
                pass
        left = wait_gone(left, 10)
    return left


def stats(xs):
    xs = [x for x in xs if isinstance(x, (int, float))]
    if not xs:
        return 'n/a'
    sd = st.stdev(xs) if len(xs) > 1 else 0.0
    return 'mean %.2f  std %.2f  min %.2f  max %.2f  (n=%d)' % (st.mean(xs), sd, min(xs), max(xs), len(xs))


def write_outputs(out, rows, a):
    with open(os.path.join(out, 'runs.csv'), 'w', newline='') as fh:
        w = csv.DictWriter(fh, fieldnames=FIELDS)
        w.writeheader()
        w.writerows(rows)
    ok = [r for r in rows if r['success']]
    lines = [
        'Evaluation %s - %d run(s), each from a fresh headless launch' % (os.path.basename(out), len(rows)),
        'success = mission completed AND container on the table AND tilt < %.0f deg' % a.max_tilt,
        '',
        'success rate          : %d / %d (%.0f %%)' % (len(ok), len(rows), 100.0 * len(ok) / max(1, len(rows))),
        'mission completed     : %d / %d' % (sum(1 for r in rows if r['mission_ok']), len(rows)),
        'container on table    : %d / %d' % (sum(1 for r in rows if r['on_table']), len(rows)),
        'placement error (mm)  : ' + stats([r['placement_error_mm'] for r in rows]),
        'tilt (deg)            : ' + stats([r['tilt_deg'] for r in rows]),
        'mission sim time (s)  : ' + stats([r['sim_time_s'] for r in ok]),
        'mission wall time (s) : ' + stats([r['mission_wall_s'] for r in ok]),
        'run wall time (s)     : ' + stats([r['total_wall_s'] for r in rows]) + '  (incl. launch/shutdown)',
        '',
        'failures:',
    ] + ['  run %d: %s %s %s' % (r['run'], r['failed_step'] or '', r['error'] or '', r['note'] or '')
         for r in rows if not r['success']] + (['  none'] if len(ok) == len(rows) else [])
    with open(os.path.join(out, 'summary.txt'), 'w') as fh:
        fh.write('\n'.join(lines) + '\n')
    return lines


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--runs', type=int, default=5)
    p.add_argument('--out', default=os.path.join(ROOT, 'results', 'eval_' + dt.datetime.now().strftime('%Y%m%d_%H%M')))
    p.add_argument('--max-tilt', type=float, default=10.0)
    p.add_argument('--ready-timeout', type=float, default=900.0, help='wall s to wait for controllers')
    p.add_argument('--mission-timeout', type=float, default=3 * 3600.0, help='wall s per mission')
    a = p.parse_args()
    if 'CAMERA_TYPE' not in os.environ:
        sys.exit('CAMERA_TYPE is not set (source ~/.bashrc or export CAMERA_TYPE=GEMINI)')
    if gazebo_pids():
        sys.exit('a simulation is already running (%s) - stop it first' % gazebo_pids())
    os.makedirs(a.out, exist_ok=True)
    log('output: %s' % a.out)
    rows = []
    for run in range(1, a.runs + 1):
        t_run = time.monotonic()
        row = dict.fromkeys(FIELDS)
        row.update(run=run, started=dt.datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                   success=False, mission_ok=False)
        launch_log = os.path.join(a.out, 'run%02d_launch.log' % run)
        mission_log = os.path.join(a.out, 'run%02d_mission.log' % run)
        rj = os.path.join(a.out, 'run%02d_result.json' % run)
        log('run %d/%d: launching' % (run, a.runs))
        proc, fh = start_sim(launch_log)
        try:
            err = wait_ready(proc, launch_log, a.ready_timeout)
            if err:
                row.update(error=err, note='launch')
                log('run %d: %s' % (run, err))
            else:
                log('run %d: controllers active, starting mission' % run)
                t_m = time.monotonic()
                with open(mission_log, 'w') as mf:
                    try:
                        r = subprocess.run([sys.executable, '-u', MISSION, '--result-json', rj,
                                            '--label', 'evaluation run %d' % run],
                                           stdout=mf, stderr=subprocess.STDOUT, timeout=a.mission_timeout)
                        code = r.returncode
                    except subprocess.TimeoutExpired:
                        code = None
                        row.update(error='mission timed out after %.0f s' % a.mission_timeout)
                row['mission_wall_s'] = time.monotonic() - t_m
                res = {}
                if os.path.exists(rj):
                    with open(rj) as jf:
                        res = json.load(jf)
                row['mission_ok'] = bool(code == 0 and res.get('success'))
                row['failed_step'] = res.get('failed_step')
                row['error'] = row['error'] or res.get('error')
                if res.get('container'):
                    row['container_x'], row['container_y'], row['container_z'] = res['container']
                if res.get('target'):
                    row['target_x'], row['target_y'] = res['target']
                for k_out, k_in in (('placement_error_mm', 'placement_error_mm'), ('tilt_deg', 'tilt_deg'),
                                    ('on_table', 'on_table'), ('sim_time_s', 'sim_time')):
                    row[k_out] = res.get(k_in)
                if res.get('base_final'):
                    row['base_final_x'], row['base_final_y'], row['base_final_yaw_deg'] = res['base_final']
                row['success'] = bool(row['mission_ok'] and row['on_table'] and
                                      row['tilt_deg'] is not None and row['tilt_deg'] < a.max_tilt)
        finally:
            left = stop_sim(proc, fh)
            if left:
                row['note'] = (row['note'] or '') + ' leftover processes %s' % left
        row['total_wall_s'] = time.monotonic() - t_run
        for k in ('placement_error_mm', 'tilt_deg', 'sim_time_s', 'mission_wall_s', 'total_wall_s'):
            if isinstance(row[k], float):
                row[k] = round(row[k], 2)
        rows.append(row)
        log('run %d: %s  placement %s mm  tilt %s deg  sim %s s  wall %.0f s' % (
            run, 'SUCCESS' if row['success'] else 'FAIL', row['placement_error_mm'], row['tilt_deg'],
            row['sim_time_s'], row['total_wall_s']))
        write_outputs(a.out, rows, a)
        if left:
            log('stopping: simulation processes could not be removed')
            break
    print('\n'.join(write_outputs(a.out, rows, a)))


if __name__ == '__main__':
    main()
