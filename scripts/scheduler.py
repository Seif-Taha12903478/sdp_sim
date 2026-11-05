#!/usr/bin/env python3
"""Stage 5: medication scheduler - triggers mission.py at the delivery times.

    python3 scheduler.py                        # run the schedule (default ~/sdp_sim/schedule.yaml)
    python3 scheduler.py --list                 # show the next due time of every entry
    python3 scheduler.py --in-minutes 1         # test: deliver the first entry 1 min from now
    python3 scheduler.py --in-minutes 1 --entry 2 --once

Schedule YAML (see ~/sdp_sim/schedule.yaml): a list `deliveries` of
{patient, medicine, time, bed}; time "HH:MM" = daily, "YYYY-MM-DD HH:MM" = once.

Times are wall-clock times of day (a schedule is about real time); inside a
mission every wait is on simulation feedback. Deliveries that fall due while
another mission runs are queued and run one after another (one robot). Each
mission must end parked at the start (mission.py step 10) for the next one.
Every delivery is appended to the log CSV (default ~/sdp_sim/results/deliveries.csv)
with its result; the mission's JSON result is kept next to it.
"""
import argparse
import csv
import datetime as dt
import json
import os
import subprocess
import sys
import time

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
MISSION = os.path.join(HERE, 'mission.py')
sys.path.insert(0, os.path.join(ROOT, 'tools'))
import scoring                      # noqa: E402  (same scoring as the evaluation)

WELD_FIELDS = ['spawn_detach_attempts', 'pick_attach_attempts', 'place_detach_attempts',
               'weld_unconfirmed']
LOG_FIELDS = ['due', 'started', 'finished', 'patient', 'medicine', 'bed', 'success',
              'failed_step', 'error', 'placement_error_mm', 'at_target', 'tilt_deg', 'on_table',
              ] + WELD_FIELDS + ['sim_time_s', 'wall_time_s', 'result_json']


def log(msg):
    print('[scheduler %s] %s' % (dt.datetime.now().strftime('%H:%M:%S'), msg), flush=True)


def load(path):
    with open(path) as fh:
        data = yaml.safe_load(fh) or {}
    out = []
    for i, d in enumerate(data.get('deliveries') or [], 1):
        for k in ('patient', 'medicine', 'time'):
            if k not in d:
                sys.exit('%s: delivery %d has no "%s"' % (path, i, k))
        t = str(d['time']).strip()
        try:
            if len(t) <= 5:
                hh, mm = (int(v) for v in t.split(':'))
                spec = ('daily', dt.time(hh, mm))
            else:
                spec = ('once', dt.datetime.strptime(t, '%Y-%m-%d %H:%M'))
        except ValueError:
            sys.exit('%s: delivery %d: bad time "%s" (use HH:MM or YYYY-MM-DD HH:MM)' % (path, i, t))
        out.append({'n': i, 'patient': str(d['patient']), 'medicine': str(d['medicine']),
                    'bed': int(d.get('bed', 1)), 'spec': spec})
    if not out:
        sys.exit('%s: no deliveries' % path)
    return out


def next_due(entry, after):
    kind, v = entry['spec']
    if kind == 'once':
        return v if v > after else None
    due = dt.datetime.combine(after.date(), v)
    return due if due > after else due + dt.timedelta(days=1)


def write_log_row(path, row):
    """Append a row; if the file has an older header, rewrite it once with the
    current columns (old rows keep their values, new columns stay empty)."""
    if os.path.exists(path):
        with open(path, newline='') as fh:
            rd = csv.DictReader(fh)
            header, old = rd.fieldnames or [], list(rd)
        if header != LOG_FIELDS:
            with open(path, 'w', newline='') as fh:
                w = csv.DictWriter(fh, fieldnames=LOG_FIELDS, extrasaction='ignore')
                w.writeheader()
                w.writerows(old)
            log('%s: upgraded to the current columns (%d old row(s) kept)' % (path, len(old)))
    new = not os.path.exists(path)
    with open(path, 'a', newline='') as fh:
        w = csv.DictWriter(fh, fieldnames=LOG_FIELDS, extrasaction='ignore')
        if new:
            w.writeheader()
        w.writerow(row)


def run_delivery(entry, due, a):
    os.makedirs(a.log_dir, exist_ok=True)
    stamp = dt.datetime.now().strftime('%Y%m%d_%H%M%S')
    rj = os.path.join(a.log_dir, 'mission_%s.json' % stamp)
    label = 'delivery %d: %s to %s (bed %d), due %s' % (
        entry['n'], entry['medicine'], entry['patient'], entry['bed'], due.strftime('%Y-%m-%d %H:%M'))
    started = dt.datetime.now()
    if entry['bed'] != 1:
        log('bed %d does not exist in this world - skipped' % entry['bed'])
        res = {'success': False, 'error': 'unknown bed %d' % entry['bed']}
    else:
        log('START ' + label)
        cmd = [sys.executable, '-u', MISSION, '--result-json', rj, '--label', label]
        r = subprocess.run(cmd)
        res = {}
        if os.path.exists(rj):
            with open(rj) as fh:
                res = json.load(fh)
        res.setdefault('success', r.returncode == 0)
        log('%s %s' % ('DONE ' if res.get('success') else 'FAILED', label))
    row = {'due': due.strftime('%Y-%m-%d %H:%M'), 'started': started.strftime('%Y-%m-%d %H:%M:%S'),
           'finished': dt.datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
           'patient': entry['patient'], 'medicine': entry['medicine'], 'bed': entry['bed'],
           'success': res.get('success'), 'failed_step': res.get('failed_step'),
           'error': res.get('error'), 'placement_error_mm': res.get('placement_error_mm'),
           'tilt_deg': res.get('tilt_deg'), 'on_table': res.get('on_table'),
           'sim_time_s': round(res['sim_time'], 2) if res.get('sim_time') is not None else None,
           'wall_time_s': round(res['wall_time'], 1) if res.get('wall_time') is not None else None,
           'result_json': rj if os.path.exists(rj) else ''}
    # placement / tilt / attach-detach attempts scored like tools/evaluate.py
    sc = scoring.score(res) if res.get('container') or res.get('weld') is not None else {}
    for k in ('placement_error_mm', 'at_target', 'tilt_deg', 'on_table') + tuple(WELD_FIELDS):
        if sc.get(k) is not None:
            row[k] = round(sc[k], 2) if isinstance(sc[k], float) else sc[k]
    if sc and res.get('success'):
        row['success'] = sc['success']      # completed AND delivered at the target
    path = os.path.join(a.log_dir, 'deliveries.csv')
    write_log_row(path, row)
    log('logged to %s' % path)
    return bool(res.get('success'))


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--schedule', default=os.path.join(ROOT, 'schedule.yaml'))
    p.add_argument('--log-dir', default=os.path.join(ROOT, 'results'))
    p.add_argument('--list', action='store_true', help='print the next due times and exit')
    p.add_argument('--in-minutes', type=float, default=None,
                   help='test: make one entry due this many minutes from now (ignores its time)')
    p.add_argument('--entry', type=int, default=1, help='entry used by --in-minutes (1-based)')
    p.add_argument('--once', action='store_true', help='exit after the first delivery')
    a = p.parse_args()

    entries = load(a.schedule)
    now = dt.datetime.now()
    if a.in_minutes is not None:
        e = next((x for x in entries if x['n'] == a.entry), None)
        if e is None:
            sys.exit('no entry %d' % a.entry)
        e = dict(e, spec=('once', now + dt.timedelta(minutes=a.in_minutes)))
        entries = [e]
    due = {e['n']: next_due(e, now) for e in entries}
    for e in entries:
        log('entry %d: %s for %s (bed %d) -> next %s' % (
            e['n'], e['medicine'], e['patient'], e['bed'],
            due[e['n']].strftime('%Y-%m-%d %H:%M:%S') if due[e['n']] else 'never (past)'))
    if a.list:
        return

    try:
        while any(due.values()):
            now = dt.datetime.now()
            ready = sorted((d, n) for n, d in due.items() if d and d <= now)
            if not ready:
                nxt = min(d for d in due.values() if d)
                time.sleep(min(30.0, max(0.5, (nxt - now).total_seconds())))
                continue
            d, n = ready[0]
            e = next(x for x in entries if x['n'] == n)
            run_delivery(e, d, a)
            due[n] = next_due(e, max(d, dt.datetime.now()) if e['spec'][0] == 'daily' else d)
            if a.once:
                break
    except KeyboardInterrupt:
        log('stopped')


if __name__ == '__main__':
    main()
