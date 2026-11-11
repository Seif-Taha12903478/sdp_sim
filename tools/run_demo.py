#!/usr/bin/env python3
"""Test step e of the hospital system: the whole demo, self-contained, ~5-6 h wall.

    bash ~/sdp_sim/tools/run_demo.sh          # starts THIS script detached (survives logout)
    python3 ~/sdp_sim/tools/run_demo.py [--out DIR] [--fast-speed 0.12] [--max-hours 14]

1. no simulation may be running
2. ros2 launch robot_gazebo hospital_system.launch.py gui:=false video:=true (own process group)
3. controllers active -> arm home
4. tools/record_video.py (overview camera 2 Hz + wrist inset, captions from the scheduler log:
   hospital clock, patient, room, medicine, step), stops on "[scheduler] finished"
5. scripts/hospital_scheduler.py --demo: pharmacy setup, 4 doses on the hospital clock
   (Rooms 1 + 3 due 08:00, Room 4 08:02, Room 2 08:04), queue, deliveries, deliveries.csv
6. recorder finished (or stopped), simulation shut down, no Gazebo process left
7. summary.txt + summary.json in the output dir; the video is copied to media/system_demo.mp4
Everything goes to <out>/: run.log (this script), launch.log, scheduler.log, video.log,
the delivery JSONs (results/system/deliveries/) are listed in the summary. Touch <out>/STOP
to end after the current delivery (the scheduler is stopped, the video closed cleanly).
"""
import argparse
import csv
import datetime as dt
import json
import os
import shutil
import signal
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import evaluate as ev           # noqa: E402

PY = sys.executable


def gazebo_server_alive():
    out = subprocess.run(['ps', '-eo', 'args='], capture_output=True, text=True).stdout
    return any(ln.startswith('ign gazebo') or (ln.startswith('ruby') and 'ign gazebo' in ln) for ln in out.splitlines())


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--out', default=None)
    p.add_argument('--fast-speed', type=float, default=0.0,
                   help='scheduler --fast-speed (0 = not passed: the scheduler default 0.12 applies)')
    p.add_argument('--cap-check', action='store_true')
    p.add_argument('--max-hours', type=float, default=14.0, help='wall-clock limit for the scheduler')
    p.add_argument('--video', default=os.path.join(ROOT, 'media', 'system_demo.mp4'))
    a = p.parse_args()
    stamp = dt.datetime.now().strftime('%Y%m%d_%H%M%S')
    out = a.out or os.path.join(ROOT, 'results', 'system', 'demo_' + stamp)
    os.makedirs(out, exist_ok=True)
    summary = {'started': dt.datetime.now().isoformat(timespec='seconds'), 'out': out, 'steps': {}}
    t0 = time.monotonic()

    def save():
        summary['wall_h'] = round((time.monotonic() - t0) / 3600.0, 2)
        with open(os.path.join(out, 'summary.json'), 'w') as fh:
            json.dump(summary, fh, indent=2)

    if ev.gazebo_pids():
        ev.log('a simulation is already running (%s) - not starting' % ev.gazebo_pids())
        summary['error'] = 'simulation already running'
        save()
        sys.exit(2)
    csv_path = os.path.join(ROOT, 'results', 'deliveries.csv')
    n_rows_before = 0
    if os.path.exists(csv_path):
        with open(csv_path, newline='') as fh:
            n_rows_before = sum(1 for _ in csv.DictReader(fh))
    launch_log = os.path.join(out, 'launch.log')
    proc, fh = ev.start_sim(launch_log, 'hospital_system.launch.py', ['video:=true'])
    rec = sch = None
    video_tmp = os.path.join(out, 'system_demo.mp4')
    try:
        err = ev.wait_ready(proc, launch_log, 1800)
        if err:
            raise RuntimeError('launch: ' + err)
        ev.log('controllers active after %.0f s' % (time.monotonic() - t0))
        r = subprocess.run([PY, os.path.join(ROOT, 'scripts', 'jetarm_pick_place.py'), 'home'],
                           capture_output=True, text=True, timeout=3600)
        summary['steps']['arm_home'] = r.returncode == 0
        sched_log = os.path.join(out, 'scheduler.log')
        open(sched_log, 'w').close()
        rec = subprocess.Popen([PY, '-u', os.path.join(HERE, 'record_video.py'), video_tmp, '--caption-log', sched_log,
                                '--stop-on', '[scheduler] finished', '--tail', '8', '--wrist-inset',
                                '--title', 'Hospital medicine-delivery robot - Gazebo simulation (demo)'],
                               stdout=open(os.path.join(out, 'video.log'), 'w'), stderr=subprocess.STDOUT,
                               start_new_session=True)
        cmd = [PY, '-u', os.path.join(ROOT, 'scripts', 'hospital_scheduler.py'), '--demo']
        stop_file = os.path.join(out, 'STOP')
        cmd += ['--stop-file', stop_file]
        if a.fast_speed:
            cmd += ['--fast-speed', str(a.fast_speed)]
        if a.cap_check:
            cmd += ['--cap-check']
        ev.log('scheduler: %s' % ' '.join(cmd[2:]))
        sch = subprocess.Popen(cmd, stdout=open(sched_log, 'a'), stderr=subprocess.STDOUT, start_new_session=True)
        end = time.monotonic() + a.max_hours * 3600
        while sch.poll() is None:
            if time.monotonic() > end:
                ev.log('scheduler wall-clock limit (%.1f h) reached - stopping it' % a.max_hours)
                os.killpg(sch.pid, signal.SIGINT)
                break
            if os.path.exists(stop_file) and not summary.get('stop_file_seen'):
                ev.log('STOP file found - the scheduler ends after the current delivery')
                summary['stop_file_seen'] = True
            if proc.poll() is not None or not gazebo_server_alive():
                ev.log('the simulation is gone (launch exit %s) - stopping the scheduler' % proc.poll())
                summary['error'] = 'simulation died during the demo'
                os.killpg(sch.pid, signal.SIGINT)
                break
            time.sleep(20)
        try:
            sch.wait(timeout=600)
        except subprocess.TimeoutExpired:
            os.killpg(sch.pid, signal.SIGKILL)
        summary['steps']['scheduler_exit'] = sch.returncode
        ev.log('scheduler exited with %s' % sch.returncode)
        with open(sched_log, 'a') as fl:     # make sure the recorder sees its stop line
            fl.write('[scheduler] finished (run_demo)\n')
        try:
            rec.wait(timeout=1200)
        except subprocess.TimeoutExpired:
            ev.log('video recorder still running - SIGTERM')
            os.killpg(rec.pid, signal.SIGTERM)
            rec.wait(timeout=120)
    except (RuntimeError, subprocess.TimeoutExpired) as e:
        ev.log('FAILED: %s' % e)
        summary['error'] = str(e)
    finally:
        for pr in (sch, rec):
            if pr is not None and pr.poll() is None:
                try:
                    os.killpg(pr.pid, signal.SIGTERM if pr is rec else signal.SIGINT)
                    pr.wait(timeout=120)
                except (ProcessLookupError, subprocess.TimeoutExpired):
                    pass
        summary['leftover_processes'] = ev.stop_sim(proc, fh)
        # results
        rows = []
        if os.path.exists(csv_path):
            with open(csv_path, newline='') as fc:
                rows = list(csv.DictReader(fc))[n_rows_before:]
        keys = ('room', 'patient', 'medicine', 'dose', 'slot', 'due', 'started', 'lateness_min', 'success',
                'failed_step', 'placement_error_mm', 'table_raised', 'right_container', 'others_in_place',
                'sim_time_s', 'wall_time_s', 'result_json')
        summary['deliveries'] = [{k: r.get(k) for k in keys} for r in rows]
        if os.path.exists(video_tmp) and os.path.getsize(video_tmp) > 0:
            os.makedirs(os.path.dirname(a.video), exist_ok=True)
            shutil.copy2(video_tmp, a.video)
            summary['video'] = a.video
        ok = [r for r in rows if str(r.get('success')) == 'True']
        summary['result'] = '%d/%d deliveries successful' % (len(ok), len(rows))
        save()
        with open(os.path.join(out, 'summary.txt'), 'w') as ft:
            ft.write('Hospital system demo %s\n%s\n\n' % (summary['started'], summary['result']))
            for r in summary['deliveries']:
                ft.write('Room %s  %-10s %-12s %-7s slot %-3s due %s start %s late %s min  %s  placement %s mm  '
                         'raised %s  right %s  others %s  sim %s s  wall %s s\n' % (
                             r['room'], r['patient'], r['medicine'], r['dose'], r['slot'], (r['due'] or '')[11:16],
                             (r['started'] or '')[11:19], r['lateness_min'], 'OK' if r['success'] == 'True' else
                             'FAILED %s' % r['failed_step'], r['placement_error_mm'], r['table_raised'],
                             r['right_container'], r['others_in_place'], r['sim_time_s'], r['wall_time_s']))
            ft.write('\nvideo: %s\nwall time %.2f h\n' % (summary.get('video'), summary['wall_h']))
        ev.log('%s; summary in %s' % (summary['result'], out))


if __name__ == '__main__':
    main()
