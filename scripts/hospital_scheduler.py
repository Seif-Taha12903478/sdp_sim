#!/usr/bin/env python3
"""Smart medication scheduler of the hospital system (branch hospital-system).

    python3 hospital_scheduler.py                 # real time (Asia/Dubai), config/doses.yaml, runs forever
    python3 hospital_scheduler.py --demo          # demo: 4 doses on a HOSPITAL CLOCK, then exit
    python3 hospital_scheduler.py --list          # the doses of today and their queue order
    python3 hospital_scheduler.py --once          # run what is due now (real time), then exit
    python3 hospital_scheduler.py --now 3         # test: one dose for room 3 due now (real time), then exit

Needs a running hospital_system.launch.py (headless is fine). At start it sets up the
pharmacy (scripts/pharmacy.py setup: containers spawned, welds released) unless every
stocked container already stands in its slot.

Doses (config/doses.yaml): patient, room, medicine, dose, daily times "HH:MM". When doses
are due, the queue is ordered by due time, then by route length (nearest room first);
one container at a time: for each dose the scheduler picks the first stocked slot of that
medicine + dose (config/inventory.yaml), runs scripts/delivery.py (pharmacy -> room ->
home), scores it (tools/scoring.score_delivery: the right container on the right room's
raised platform, every other container where it belongs) and appends a row to
results/deliveries.csv. A dose that falls due while another delivery runs waits; it is
run as soon as possible and its lateness (start - due) is logged. No container left for
a medicine -> "out of stock": skipped, logged, the scheduler goes on.

Clocks: normal mode = the real local time in the doses file's time_zone (Asia/Dubai).
--demo = HOSPITAL CLOCK: simulated time (from /joint_states stamps) mapped to
--demo-start (08:00) at the moment the demo starts; the log line "[clock] hospital
08:00:00 = sim T" lets tools/record_video.py caption every frame with hospital time.
Demo doses: Room 3 (Ibuprofen) and Room 1 (Paracetamol) both due 08:00 -> Room 1 first
(nearer), then Room 3; Room 4 (Metformin) due 08:02; Room 2 (Amoxicillin) due 08:04 - they
fall due while earlier deliveries run, so they queue and their lateness is logged.
"""
import argparse
import csv
import datetime as dt
import json
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, 'tools'))

DOSES = os.path.join(ROOT, 'config', 'doses.yaml')
LOG_DIR = os.path.join(ROOT, 'results')
DELIVERY = os.path.join(HERE, 'delivery.py')
PHARMACY = os.path.join(HERE, 'pharmacy.py')

LOG_FIELDS = ['clock', 'due', 'started', 'finished', 'lateness_min', 'patient', 'room', 'medicine', 'dose',
              'slot', 'container', 'success', 'failed_step', 'error', 'placement_error_mm', 'at_target',
              'tilt_deg', 'on_table', 'table_raised', 'lift_time_s', 'container_on_platform_after_lift',
              'right_container', 'others_in_place', 'pick_attach_attempts', 'place_detach_attempts',
              'weld_unconfirmed', 'route_errors', 'sim_time_s', 'wall_time_s', 'result_json']

DEMO = [  # (minutes after the demo start, room, patient, medicine, dose)
    (0, 3, 'Patient C', 'Ibuprofen', '400 mg'),       # due together with room 1:
    (0, 1, 'Patient A', 'Paracetamol', '500 mg'),     #   room 1 goes first (shorter route)
    (2, 4, 'Patient D', 'Metformin', '500 mg'),
    (4, 2, 'Patient B', 'Amoxicillin', '250 mg'),
]


def log(msg, clock=None):
    stamp = clock.now().strftime('%H:%M:%S') if clock else dt.datetime.now().strftime('%H:%M:%S')
    print('[scheduler %s] %s' % (stamp, msg), flush=True)


# ------------------------------------------------------------------ clocks
class RealClock:
    """Real local time in a time zone (default Asia/Dubai)."""
    name = 'real'

    def __init__(self, tz='Asia/Dubai'):
        from zoneinfo import ZoneInfo
        self.tz = ZoneInfo(tz)

    def now(self):
        return dt.datetime.now(self.tz).replace(microsecond=0)

    def wait_until(self, when, max_wall=60.0):
        time.sleep(max(0.0, min(max_wall, (when - self.now()).total_seconds())))


class HospitalClock:
    """Simulated time mapped to a start time: hospital = start + (sim - sim0).
    sim_now: callable returning the current sim time (seconds)."""
    name = 'hospital'

    def __init__(self, start, sim_now, sim0=None):
        self.start, self.sim_now = start, sim_now
        self.sim0 = sim_now() if sim0 is None else sim0

    def at(self, sim_t):
        return self.start + dt.timedelta(seconds=sim_t - self.sim0)

    def now(self):
        return self.at(self.sim_now()).replace(microsecond=0)

    def wait_until(self, when, max_wall=60.0):
        end = time.monotonic() + max_wall
        while self.now() < when and time.monotonic() < end:
            time.sleep(0.5)


class SimTime:
    """Sim time from /joint_states stamps (background spin thread)."""

    def __init__(self):
        import threading
        import rclpy
        from sensor_msgs.msg import JointState
        rclpy.init()
        self.node = rclpy.create_node('hospital_scheduler_clock')
        self.t = None

        def on_js(m):
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            if self.t is None or t > self.t:
                self.t = t
        self.node.create_subscription(JointState, '/joint_states', on_js, 5)
        threading.Thread(target=lambda: rclpy.spin(self.node), daemon=True).start()
        end = time.monotonic() + 600
        while self.t is None and time.monotonic() < end:
            time.sleep(0.2)
        if self.t is None:
            raise RuntimeError('no /joint_states - is the simulation running?')

    def __call__(self):
        return self.t


# ------------------------------------------------------------------ doses and queue (no ROS)
class Dose:
    def __init__(self, n, patient, room, medicine, dose, due):
        self.id, self.patient, self.room, self.medicine, self.dose, self.due = n, patient, int(room), medicine, dose, due
        self.state = 'pending'          # pending | done | failed | skipped
        self.slot = None

    def key(self):
        return '%s|%d|%s %s|%s' % (self.patient, self.room, self.medicine, self.dose, self.due.isoformat())

    def __repr__(self):
        return 'Dose(%s, room %d, %s %s, due %s)' % (self.patient, self.room, self.medicine, self.dose,
                                                     self.due.strftime('%H:%M'))


def load_doses(path=DOSES):
    import yaml
    with open(path) as fh:
        data = yaml.safe_load(fh) or {}
    entries = []
    for i, e in enumerate(data.get('doses') or [], 1):
        for k in ('patient', 'room', 'medicine', 'dose', 'times'):
            if k not in e:
                raise ValueError('%s: dose %d has no "%s"' % (path, i, k))
        times = e['times'] if isinstance(e['times'], list) else [e['times']]
        for t in times:
            hh, mm = (int(v) for v in str(t).split(':'))
            entries.append({'patient': str(e['patient']), 'room': int(e['room']), 'medicine': str(e['medicine']),
                            'dose': str(e['dose']), 'hh': hh, 'mm': mm})
    return data.get('time_zone', 'Asia/Dubai'), entries


def doses_for_day(entries, day, tz, start_id=1):
    out = []
    for i, e in enumerate(entries):
        due = dt.datetime(day.year, day.month, day.day, e['hh'], e['mm'], tzinfo=tz)
        out.append(Dose(start_id + i, e['patient'], e['room'], e['medicine'], e['dose'], due))
    return out


def demo_doses(start):
    return [Dose(i + 1, p, r, m, d, start + dt.timedelta(minutes=k)) for i, (k, r, p, m, d) in enumerate(DEMO)]


def order_queue(doses, now, route_len):
    """Due doses (due <= now, still pending) in run order: due time, then route length."""
    due = [d for d in doses if d.state == 'pending' and d.due <= now]
    return sorted(due, key=lambda d: (d.due, route_len(d.room), d.id))


def lateness_min(started, due):
    return round((started - due).total_seconds() / 60.0, 2)


class Stock:
    """Which slot serves a dose; a slot counts as used once its container left it."""

    def __init__(self, inv):
        self.inv = inv
        self.used = set()

    def slot_for(self, medicine, dose):
        for s in sorted(self.inv):
            d = self.inv[s]
            if d['quantity'] >= 1 and s not in self.used and d['medicine'].lower() == medicine.lower() \
                    and d['dose'].replace(' ', '').lower() == dose.replace(' ', '').lower():
                return s
        return None

    def left(self, medicine, dose):
        return sum(1 for s, d in self.inv.items() if d['quantity'] >= 1 and s not in self.used
                   and d['medicine'].lower() == medicine.lower()
                   and d['dose'].replace(' ', '').lower() == dose.replace(' ', '').lower())


def route_lengths():
    """Room -> route length pharmacy -> room (m), for the queue's tie-break."""
    import route_planner as rp
    net = rp.Network()
    return {int(r): net.shortest('pharmacy', 'R%s' % r)[0] for r in net.m['rooms']}


# ------------------------------------------------------------------ CSV
def write_row(path, row):
    """Append a row; an older header (main's scheduler) is upgraded once: old rows kept."""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    fields = list(LOG_FIELDS)
    if os.path.exists(path) and os.path.getsize(path) > 0:
        with open(path, newline='') as fh:
            rd = csv.DictReader(fh)
            old_fields, rows = rd.fieldnames or [], list(rd)
        if old_fields != fields:
            merged = old_fields + [f for f in fields if f not in old_fields]
            if merged != old_fields:
                with open(path, 'w', newline='') as fh:
                    w = csv.DictWriter(fh, fieldnames=merged, extrasaction='ignore')
                    w.writeheader()
                    w.writerows(rows)
            fields = merged
    new = not os.path.exists(path) or os.path.getsize(path) == 0
    with open(path, 'a', newline='') as fh:
        w = csv.DictWriter(fh, fieldnames=fields, extrasaction='ignore')
        if new:
            w.writeheader()
        w.writerow(row)


# ------------------------------------------------------------------ one delivery
def run_delivery(d, clock, a, out_dir):
    import scoring
    stamp = clock.now().strftime('%H%M%S')
    rj = os.path.join(out_dir, 'delivery_%s_R%d_%s.json' % (stamp, d.room, d.slot))
    label = '%s, Room %d, %s %s (slot %s), due %s' % (d.patient, d.room, d.medicine, d.dose, d.slot,
                                                       d.due.strftime('%H:%M'))
    cmd = [sys.executable, '-u', DELIVERY, '--room', str(d.room), '--slot', d.slot, '--patient', d.patient,
           '--medicine', d.medicine, '--dose', d.dose, '--result-json', rj, '--label', label]
    if a.fast_speed:
        cmd += ['--fast-speed', str(a.fast_speed)]
    if a.cap_check:
        cmd += ['--cap-check']
    started = clock.now()
    log('START %s - lateness %.1f min' % (label, lateness_min(started, d.due)), clock)
    t0 = time.monotonic()
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
    for line in proc.stdout:
        sys.stdout.write(line)
        sys.stdout.flush()
    code = proc.wait()
    wall = time.monotonic() - t0
    finished = clock.now()
    res = {}
    try:
        with open(rj) as fh:
            res = json.load(fh)
    except (OSError, ValueError):
        pass
    row = scoring.score_delivery(res, mission_exit_ok=(code == 0))
    row.update(clock=clock.name, due=d.due.isoformat(), started=started.isoformat(), finished=finished.isoformat(),
               lateness_min=lateness_min(started, d.due), patient=d.patient, room=d.room, medicine=d.medicine,
               dose=d.dose, slot=d.slot, container='med_' + d.slot, wall_time_s=round(wall, 1), result_json=rj)
    log('END %s: %s%s; placement %s mm, table raised %s, right container %s, others in place %s' % (
        label, 'SUCCESS' if row['success'] else 'FAILED', '' if row['success'] else ' (%s: %s)' % (
            row.get('failed_step'), row.get('error')),
        '%.1f' % row['placement_error_mm'] if row.get('placement_error_mm') is not None else '-',
        row.get('table_raised'), row.get('right_container'), row.get('others_in_place')), clock)
    return row, res


def container_left_slot(res, slot):
    """True if the delivery took the container out of its slot (the slot is used)."""
    import pharmacy as ph
    f = (res.get('containers_final') or {}).get('med_' + slot)
    if not f:
        return any(s.get('step', 0) >= 6 for s in res.get('steps') or [])
    ph_ = ph.Pharmacy()
    return not ph_.in_slot(slot, (tuple(f['xyz']), tuple(f['quat_xyzw'])))


# ------------------------------------------------------------------ main
def pharmacy_ready():
    import pharmacy as ph
    p = ph.Pharmacy()
    poses = ph.gz_all_poses('med_')
    return all(p.in_slot(s, poses.get(ph.container_name(s))) for s in p.stocked())


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--doses', default=DOSES)
    p.add_argument('--demo', action='store_true', help='4 demo doses on the hospital clock, then exit')
    p.add_argument('--demo-start', default='08:00', help='hospital clock at the start of the demo (HH:MM)')
    p.add_argument('--once', action='store_true', help='real time: run the doses due now, then exit')
    p.add_argument('--list', action='store_true')
    p.add_argument('--now', type=int, default=None, metavar='ROOM',
                   help='test: one dose for this room (demo patient / medicine) due now, real time, then exit')
    p.add_argument('--log-dir', default=LOG_DIR, help='deliveries.csv here, results JSON in <log-dir>/system/')
    p.add_argument('--fast-speed', type=float, default=0.0, help='delivery.py --fast-speed (0 = off)')
    p.add_argument('--cap-check', action='store_true')
    p.add_argument('--no-setup', action='store_true', help='do not set up the pharmacy')
    a = p.parse_args()

    import pharmacy as ph
    inv = ph.load_inventory()
    stock = Stock(inv)
    lengths = route_lengths()
    tz_name, entries = load_doses(a.doses)
    if a.list:
        clock = RealClock(tz_name)
        today = clock.now().date()
        doses = doses_for_day(entries, today, clock.tz)
        for d in sorted(doses, key=lambda d: (d.due, lengths[d.room], d.id)):
            s = stock.slot_for(d.medicine, d.dose)
            print('%s  %-10s Room %d  %-12s %-7s slot %s  (route %.2f m)' % (
                d.due.strftime('%H:%M'), d.patient, d.room, d.medicine, d.dose, s or 'OUT OF STOCK', lengths[d.room]))
        return

    out_dir = os.path.join(a.log_dir, 'system', 'deliveries')
    os.makedirs(out_dir, exist_ok=True)
    csv_path = os.path.join(a.log_dir, 'deliveries.csv')
    if not a.no_setup:
        if pharmacy_ready():
            log('pharmacy ready: every stocked container stands in its slot')
        else:
            log('pharmacy setup (spawn the containers, release their welds)')
            r = subprocess.run([sys.executable, '-u', PHARMACY, 'setup'], text=True)
            if r.returncode != 0:
                log('pharmacy setup FAILED - stopping')
                sys.exit(2)
    if a.demo:
        sim = SimTime()
        hh, mm = (int(v) for v in a.demo_start.split(':'))
        today = dt.date.today()
        clock = HospitalClock(dt.datetime(today.year, today.month, today.day, hh, mm), sim)
        print('[clock] hospital %s = sim %.3f' % (clock.start.strftime('%H:%M:%S'), clock.sim0), flush=True)
        doses = demo_doses(clock.start)
        log('DEMO on the hospital clock (sim time from %s): %s' % (clock.start.strftime('%H:%M'), ', '.join(
            '%s %s room %d due %s' % (d.medicine, d.dose, d.room, d.due.strftime('%H:%M')) for d in doses)), clock)
    elif a.now is not None:
        clock = RealClock(tz_name)
        k, r, pat, med, ds = [e for e in DEMO if e[1] == a.now][0]
        doses = [Dose(1, pat, r, med, ds, clock.now())]
        a.once = True
        log('test dose due now: %s, Room %d, %s %s' % (pat, r, med, ds), clock)
    else:
        clock = RealClock(tz_name)
        start = clock.now()
        doses = [d for d in doses_for_day(entries, start.date(), clock.tz) if d.due >= start - dt.timedelta(minutes=1)]
        log('real time (%s): %d doses left today' % (tz_name, len(doses)), clock)
    done_keys = set()
    while True:
        now = clock.now()
        queue = order_queue(doses, now, lambda r: lengths[r])
        if queue:
            log('due now: %s' % ', '.join('%s room %d (due %s)' % (d.medicine, d.room, d.due.strftime('%H:%M'))
                                         for d in queue), clock)
            d = queue[0]
            d.slot = stock.slot_for(d.medicine, d.dose)
            if d.slot is None:
                d.state = 'skipped'
                log('OUT OF STOCK: %s %s for %s (room %d) - skipped' % (d.medicine, d.dose, d.patient, d.room), clock)
                write_row(csv_path, {'clock': clock.name, 'due': d.due.isoformat(), 'started': now.isoformat(),
                                     'patient': d.patient, 'room': d.room, 'medicine': d.medicine, 'dose': d.dose,
                                     'success': False, 'failed_step': 'out of stock',
                                     'error': 'no container of %s %s left' % (d.medicine, d.dose)})
                continue
            row, res = run_delivery(d, clock, a, out_dir)
            if container_left_slot(res, d.slot):
                stock.used.add(d.slot)
            d.state = 'done' if row['success'] else 'failed'
            done_keys.add(d.key())
            write_row(csv_path, row)
            if not row['success'] and (res.get('steps') or [{}])[-1].get('step', 0) > 1:
                # the robot moved and did not complete the delivery: it is not parked at home
                log('delivery failed after the robot moved - stopping the scheduler (no further doses)', clock)
                for q in doses:
                    if q.state == 'pending':
                        q.state = 'not run'
                break
            continue
        pending = [d for d in doses if d.state == 'pending']
        if not pending:
            if a.demo or a.once:
                break
            # real time: tomorrow's doses
            tomorrow = clock.now().date() + dt.timedelta(days=1)
            doses = [d for d in doses_for_day(entries, tomorrow, clock.tz) if d.key() not in done_keys]
            log('all doses of today handled; next day: %d doses' % len(doses), clock)
            continue
        if a.once and all(d.due > now for d in pending):
            break
        clock.wait_until(min(d.due for d in pending))
    summary = {d.key(): d.state for d in doses}
    log('%s done: %s' % ('demo' if a.demo else 'scheduler',
                         ', '.join('%s room %d %s' % (d.medicine, d.room, d.state) for d in doses)), clock)
    print('[scheduler] finished %s' % json.dumps(summary), flush=True)


if __name__ == '__main__':
    main()
