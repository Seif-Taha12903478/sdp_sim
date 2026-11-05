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

Add runs to an existing results folder (e.g. after stopping an evaluation):
    python3 evaluate.py --out ~/sdp_sim/results/eval_X --append --total 5
--append loads the folder's runs.csv and continues numbering after the last recorded
run (leftover files of an unfinished run with that number are removed first);
--total N runs as many as needed to reach N runs in total (else --runs more).
Stop cleanly after the current run: touch <out>/STOP (removed when honoured).
Ctrl+C: stops the simulation, drops the unfinished run, writes runs.csv/summary.

Needs a shell with ROS sourced and CAMERA_TYPE set (as in ~/.bashrc).
"""
import argparse
import csv
import datetime as dt
import glob
import json
import math
import os
import signal
import statistics as st
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import scoring                  # noqa: E402  (same scoring as rescore.py)
MISSION = os.path.join(ROOT, 'scripts', 'mission.py')
FIELDS = scoring.FIELDS


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
        # the gripper spawner's success line (spawned last); both on ONE line - the
        # spawner's earlier "waiting for service" line also names gripper_controller
        if any('Configured and activated' in l and 'gripper_controller' in l for l in t.splitlines()):
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


def _ok(v):
    return v is True or str(v) == 'True'


def _wall_ok(r):
    v = r.get('wall_time_valid')
    return v is None or v == '' or _ok(v)


def _f(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def failure_reason(r, max_tilt):
    if not _ok(r['mission_ok']):
        return 'mission failed: %s %s %s' % (r['failed_step'] or '', r['error'] or '', r['note'] or '')
    why = []
    if _ok(r['on_table']) and _ok(r.get('at_target')) and (_f(r['tilt_deg']) or 0) < max_tilt:
        return 'unknown'
    if not _ok(r['on_table']):
        z = _f(r['container_z'])
        where = ', on the floor' if z is not None and z < 0.05 else (
            ', still held by the gripper: not released' if z is not None and z > 0.2 else '')
        why.append('container not on the table (z %.3f%s)' % (z, where) if z is not None else 'container not found')
    t = _f(r['tilt_deg'])
    if t is not None and t >= max_tilt:
        why.append('tilt %.1f deg' % t)
    e = _f(r['placement_error_mm'])
    if e is not None:
        far = e > scoring.PLACE_TOL_MM
        cx, cy = _f(r['container_x']), _f(r['container_y'])
        at_station = cx is not None and abs(cx) < 0.05 and abs(cy - 0.20) < 0.05
        why.append('placement error %.1f mm%s' % (e, ' (container still at the medicine station: '
                                                  'not picked)' if at_station else (' (not at the target)' if far else '')))
    return 'mission completed but ' + '; '.join(why)


def write_outputs(out, rows, a):
    with open(os.path.join(out, 'runs.csv'), 'w', newline='') as fh:
        w = csv.DictWriter(fh, fieldnames=FIELDS, extrasaction='ignore')
        w.writeheader()
        w.writerows(rows)
    ok = [r for r in rows if _ok(r['success'])]
    lines = [
        'Evaluation %s - %d run(s), each from a fresh headless launch' % (os.path.basename(out), len(rows)),
        'success = mission completed AND container on the table within %.0f mm of the target AND tilt < %.0f deg'
        % (scoring.PLACE_TOL_MM, a.max_tilt),
        '',
        'success rate          : %d / %d (%.0f %%)' % (len(ok), len(rows), 100.0 * len(ok) / max(1, len(rows))),
        'mission completed     : %d / %d' % (sum(1 for r in rows if _ok(r['mission_ok'])), len(rows)),
        'container on a table  : %d / %d' % (sum(1 for r in rows if _ok(r['on_table'])), len(rows)),
        'container at target   : %d / %d  (<= %.0f mm)' % (sum(1 for r in rows if _ok(r.get('at_target'))), len(rows),
                                                          scoring.PLACE_TOL_MM),
        'placement error (mm)  : ' + stats([_f(r['placement_error_mm']) for r in rows]),
        'tilt (deg)            : ' + stats([_f(r['tilt_deg']) for r in rows]) + '  [%s]' % ', '.join(
            sorted(set(str(r.get('tilt_source')) for r in rows if r.get('tilt_source')))),
        'mission sim time (s)  : ' + stats([_f(r['sim_time_s']) for r in rows if _ok(r['mission_ok'])]) + '  (missions completed)',
        'mission wall time (s) : ' + stats([_f(r['mission_wall_s']) for r in rows
                                            if _ok(r['mission_ok']) and _wall_ok(r)]),
        'run wall time (s)     : ' + stats([_f(r['total_wall_s']) for r in rows if _wall_ok(r)])
        + '  (incl. launch/shutdown)',
        'wall times left out   : ' + (', '.join('run %d (%s)' % (r['run'], r.get('note') or 'invalid')
                                                for r in rows if not _wall_ok(r)) or 'none'),
        '',
        'attach/detach attempts until confirmed by /medicine/attached (1 = first try, 0 = not needed):',
    ] + ['  %-14s: %s' % (label, stats([_f(r.get(k)) for r in rows]))
         for label, k in (('spawn detach', 'spawn_detach_attempts'), ('pick attach', 'pick_attach_attempts'),
                          ('place detach', 'place_detach_attempts'))] + [
        '  not confirmed : %s' % (', '.join('run %d: %s' % (r['run'], r['weld_unconfirmed']) for r in rows
                                            if r.get('weld_unconfirmed') not in (None, '', 'none'))
                                  or ('none' if any(r.get('weld_unconfirmed') for r in rows)
                                      else 'n/a (runs before the confirmed weld)')),
        '',
        'failures:',
    ] + ['  run %d: %s' % (r['run'], failure_reason(r, a.max_tilt)) for r in rows if not _ok(r['success'])] \
      + (['  none'] if len(ok) == len(rows) else [])
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
    p.add_argument('--append', action='store_true', help='add runs to the existing runs.csv in --out')
    p.add_argument('--total', type=int, default=None, help='with --append: run until N runs in total')
    a = p.parse_args()
    if 'CAMERA_TYPE' not in os.environ:
        sys.exit('CAMERA_TYPE is not set (source ~/.bashrc or export CAMERA_TYPE=GEMINI)')
    if gazebo_pids():
        sys.exit('a simulation is already running (%s) - stop it first' % gazebo_pids())
    os.makedirs(a.out, exist_ok=True)
    log('output: %s' % a.out)
    rows = []
    csv_path = os.path.join(a.out, 'runs.csv')
    if a.append:
        if os.path.exists(csv_path):
            with open(csv_path) as fh:
                rows = list(csv.DictReader(fh))
            for r in rows:
                r['run'] = int(r['run'])
        log('appending to %d existing run(s)' % len(rows))
    elif os.path.exists(csv_path):
        sys.exit('%s already has runs.csv - use --append to add runs, or another --out' % a.out)
    first = max([r['run'] for r in rows], default=0) + 1
    n_new = (a.total - len(rows)) if (a.append and a.total) else a.runs
    if n_new <= 0:
        sys.exit('already %d run(s), nothing to do' % len(rows))
    last = first + n_new - 1
    stop_file = os.path.join(a.out, 'STOP')
    for run in range(first, last + 1):
        if os.path.exists(stop_file):
            os.remove(stop_file)
            log('STOP file found - not starting run %d' % run)
            break
        for f in glob.glob(os.path.join(a.out, 'run%02d_*' % run)):   # leftovers of an aborted run
            log('removing leftover %s' % os.path.basename(f))
            os.remove(f)
        t_run = time.monotonic()
        t_run_unix = time.time()          # only to detect clock jumps / VM pauses
        row = dict.fromkeys(FIELDS)
        # local time WITH its UTC offset, plus UTC: unambiguous across time-zone changes.
        # Durations below use time.monotonic().
        row.update(run=run, started=dt.datetime.now().astimezone().isoformat(timespec='seconds'),
                   started_utc=dt.datetime.now(dt.timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ'),
                   success=False, mission_ok=False)
        launch_log = os.path.join(a.out, 'run%02d_launch.log' % run)
        mission_log = os.path.join(a.out, 'run%02d_mission.log' % run)
        rj = os.path.join(a.out, 'run%02d_result.json' % run)
        log('run %d/%d: launching' % (run, last))
        proc, fh = start_sim(launch_log)
        interrupted = False
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
                    except KeyboardInterrupt:
                        interrupted = True
                if interrupted:
                    raise KeyboardInterrupt
                row['mission_wall_s'] = time.monotonic() - t_m
                res = {}
                if os.path.exists(rj):
                    with open(rj) as jf:
                        res = json.load(jf)
                err0 = row['error']
                row.update({k: v for k, v in scoring.score(res, a.max_tilt, code == 0).items()
                            if k not in ('run', 'started', 'started_utc', 'mission_wall_s', 'total_wall_s', 'note')})
                row['error'] = err0 or row['error']
        except KeyboardInterrupt:
            interrupted = True
        finally:
            left = stop_sim(proc, fh)
            if left:
                row['note'] = (row['note'] or '') + ' leftover processes %s' % left
        if interrupted:
            log('interrupted during run %d: simulation stopped, run %d not recorded%s'
                % (run, run, ' (leftover processes %s)' % left if left else ''))
            break
        row['total_wall_s'] = time.monotonic() - t_run
        # A paused VM (laptop asleep) freezes the monotonic clock while the wall clock is
        # stepped forward on resume: the run's wall time is then not a clean sample.
        jump = (time.time() - t_run_unix) - row['total_wall_s']
        row['wall_time_valid'] = abs(jump) <= 60
        if not row['wall_time_valid']:
            row['note'] = ((row['note'] or '') + ' wall time invalid: clock jumped %+.0f s during the run '
                           '(VM paused?)' % jump).strip()
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
