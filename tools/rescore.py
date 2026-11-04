#!/usr/bin/env python3
"""Re-score an evaluation directory from the per-run JSON files (no simulation needed).

    python3 ~/sdp_sim/tools/rescore.py ~/sdp_sim/results/eval_5runs [--max-tilt 10]

Reads runNN_result.json (+ runNN_mission.log for the wall time if needed) and the
old runs.csv (for run start, wall times and notes), recomputes every score with
tools/scoring.py and rewrites runs.csv and summary.txt. The old files are kept as
runs_before_rescore.csv / summary_before_rescore.txt.
"""
import argparse
import csv
import glob
import json
import os
import re
import shutil
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import scoring                       # noqa: E402
from evaluate import write_outputs   # noqa: E402


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('dir')
    p.add_argument('--max-tilt', type=float, default=10.0)
    a = p.parse_args()
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
        row.update(run=run, started=o.get('started'),
                   mission_wall_s=round(res.get('wall_time'), 2) if res.get('wall_time') else o.get('mission_wall_s'),
                   total_wall_s=o.get('total_wall_s'), note=o.get('note'))
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
