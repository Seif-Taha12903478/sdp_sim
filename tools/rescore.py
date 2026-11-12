#!/usr/bin/env python3
"""Re-score an evaluation directory from the per-run JSON files (no simulation needed).

    python3 ~/sdp_sim/tools/rescore.py ~/sdp_sim/results/eval_5runs [--max-tilt 10]

Reads runNN_result.json and the old runs.csv, recomputes every score with
tools/scoring.py and rewrites runs.csv and summary.txt. The old files are kept as
runs_before_rescore.csv / summary_before_rescore.txt.

Times: mission_wall_s / total_wall_s come from evaluate.py, which measures them with
time.monotonic() (immune to clock and time-zone changes); the mission JSON's own
wall_time is only a fallback and a cross-check (flagged if they differ by > 5 s).
started_utc is rebuilt from the first ROS log stamp (UTC epoch seconds) in
runNN_launch.log when runs.csv has none (older evaluate.py wrote local time only).
wall_time_valid: the value already recorded in runs.csv is kept (evaluate.py measures it
during the run). Only for a run without one: false when the wall clock and the monotonic
clock disagree by > 60 s over the mission (wall_start_unix + result-file time vs.
monotonic wall_time), i.e. the VM was paused - that check needs the ORIGINAL result file
(a git checkout or a copy has a new file time). Or set by hand with --invalid-wall
RUN[:reason] (kept in wall_time_invalid.json in the folder). Invalid wall times are left
out of the stats.
"""
import argparse
import csv
import datetime as dt
import glob
import json
import os
import re
import shutil
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import scoring                       # noqa: E402
from evaluate import write_outputs   # noqa: E402


def launch_start_utc(d, run):
    """UTC start of a run from the first ROS log stamp [epoch.ns] in its launch log."""
    try:
        with open(os.path.join(d, 'run%02d_launch.log' % run), errors='replace') as fh:
            for line in fh:
                m = re.search(r'\[(1[0-9]{9})\.[0-9]+\]', line)
                if m:
                    return dt.datetime.fromtimestamp(int(m.group(1)), dt.timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ')
    except OSError:
        pass
    return None


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('dir')
    p.add_argument('--max-tilt', type=float, default=10.0)
    p.add_argument('--invalid-wall', action='append', default=[], metavar='RUN[:reason]',
                   help='mark a run\'s wall time invalid (repeatable)')
    a = p.parse_args()
    inv_path = os.path.join(a.dir, 'wall_time_invalid.json')
    invalid = json.load(open(inv_path)) if os.path.exists(inv_path) else {}
    for spec in a.invalid_wall:
        run_s, _, why = spec.partition(':')
        invalid[str(int(run_s))] = why or 'marked invalid by hand'
    if a.invalid_wall:
        with open(inv_path, 'w') as fh:
            json.dump(invalid, fh, indent=2)
    old = {}
    path = os.path.join(a.dir, 'runs.csv')
    if os.path.exists(path):
        with open(path) as fh:
            old = {int(r['run']): r for r in csv.DictReader(fh)}
        for src, dst in (('runs.csv', 'runs_before_rescore.csv'), ('summary.txt', 'summary_before_rescore.txt')):
            if os.path.exists(os.path.join(a.dir, src)) and not os.path.exists(os.path.join(a.dir, dst)):
                shutil.copy(os.path.join(a.dir, src), os.path.join(a.dir, dst))
    rows = []
    for rj in sorted(glob.glob(os.path.join(a.dir, 'run*_result.json'))):
        run = int(re.search(r'run(\d+)_result', rj).group(1))
        with open(rj) as fh:
            res = json.load(fh)
        o = old.get(run, {})
        row = scoring.score(res, a.max_tilt, mission_exit_ok=True)
        notes = [o.get('note')] if o.get('note') else []
        mono = o.get('mission_wall_s')
        jw = res.get('wall_time')
        if mono not in (None, ''):
            mission_wall = float(mono)
            if jw is not None and abs(float(jw) - mission_wall) > 5:
                notes.append('mission wall time: evaluate %.0f s vs mission JSON %.0f s' % (mission_wall, jw))
        else:
            mission_wall = round(jw, 2) if jw is not None else None
        valid = True
        recorded = str(o.get('wall_time_valid'))
        if recorded in ('True', 'False'):
            # measured by evaluate.py during the run (or by an earlier rescore): keep it. The file
            # time below is only right for the ORIGINAL result file - a git checkout or a copy
            # sets a new one, which made every run of a cloned folder look like a paused VM.
            valid = recorded == 'True'
        elif res.get('wall_start_unix') and res.get('wall_time') is not None:
            jump = (os.path.getmtime(rj) - res['wall_start_unix']) - float(res['wall_time'])
            if abs(jump) > 60:
                valid = False
                notes.append('wall time invalid: clock jumped %+.0f s during the mission (VM paused?)' % jump)
        if str(run) in invalid:
            valid = False
            notes.append('wall time invalid: %s' % invalid[str(run)])
        row.update(run=run, started=o.get('started'), started_utc=o.get('started_utc') or launch_start_utc(a.dir, run),
                   mission_wall_s=mission_wall, total_wall_s=o.get('total_wall_s'),
                   wall_time_valid=valid, note='; '.join(dict.fromkeys(n for n in notes if n)) or None)
        for k in ('placement_error_mm', 'tilt_deg', 'sim_time_s'):
            if isinstance(row[k], float):
                row[k] = round(row[k], 2)
        rows.append(row)
    # runs that never produced a JSON (launch failures): keep the old row
    for run, o in old.items():
        if run not in [r['run'] for r in rows]:
            r = dict.fromkeys(scoring.FIELDS)
            r.update({k: o.get(k) for k in scoring.FIELDS if k in o}, run=run, success=False)
            rows.append(r)
    rows.sort(key=lambda r: r['run'])
    for r in rows:
        for k in ('mission_wall_s', 'total_wall_s'):
            try:
                r[k] = float(r[k]) if r[k] not in (None, '') else None
            except ValueError:
                pass
    print('\n'.join(write_outputs(a.dir, rows, a)))


if __name__ == '__main__':
    main()
