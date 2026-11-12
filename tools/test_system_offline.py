#!/usr/bin/env python3
"""Unit tests of the hospital system without Gazebo (test step a).

    python3 ~/sdp_sim/tools/test_system_offline.py [-v]

planner (every delivery leg, turns, lengths), scheduler (queue order: due time, then route
length; lateness; out of stock; demo schedule; hospital clock; CSV header upgrade) and the
delivery scoring (right container on the right raised platform, others in place).
The junction logic with the synthetic camera is tools/offline_route_sim.py.
"""
import copy
import csv
import datetime as dt
import math
import os
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, 'scripts'))
import route_planner as rp          # noqa: E402
import hospital_scheduler as hs     # noqa: E402
import scoring                      # noqa: E402
import system_layout as L           # noqa: E402

INV = {'S1': {'medicine': 'Paracetamol', 'dose': '500 mg', 'cap': 'white', 'quantity': 1},
       'S2': {'medicine': 'Amoxicillin', 'dose': '250 mg', 'cap': 'yellow', 'quantity': 1},
       'S3': {'medicine': 'Ibuprofen', 'dose': '400 mg', 'cap': 'purple', 'quantity': 1},
       'S4': {'medicine': 'Metformin', 'dose': '500 mg', 'cap': 'grey', 'quantity': 0},
       'S6': {'medicine': 'Paracetamol', 'dose': '500 mg', 'cap': 'white', 'quantity': 1}}


class Planner(unittest.TestCase):
    def setUp(self):
        self.net = rp.Network()

    def acts(self, pl):
        return [(e['node'], e['action']) for e in pl['events']]

    def test_every_leg(self):
        want = {1: ([('home', 'pass'), ('J1', 'right'), ('R1', 'stop')], [('J1', 'left'), ('home', 'stop')]),
                2: ([('home', 'pass'), ('J1', 'left'), ('R2', 'stop')], [('J1', 'right'), ('home', 'stop')]),
                3: ([('home', 'pass'), ('J1', 'straight'), ('J2', 'right'), ('R3', 'stop')],
                    [('J2', 'left'), ('J1', 'straight'), ('home', 'stop')]),
                4: ([('home', 'pass'), ('J1', 'straight'), ('J2', 'left'), ('R4', 'stop')],
                    [('J2', 'right'), ('J1', 'straight'), ('home', 'stop')])}
        for room, (out, back) in want.items():
            leg1, leg2, leg3 = rp.delivery_legs(self.net, room)
            self.assertEqual(self.acts(leg1), [('pharmacy', 'stop')])
            self.assertFalse(leg1['turn_round'])
            self.assertEqual(self.acts(leg2), out, 'room %d out' % room)
            self.assertTrue(leg2['turn_round'], 'turn round at the pharmacy')
            self.assertEqual(self.acts(leg3), back, 'room %d back' % room)
            self.assertTrue(leg3['turn_round'], 'turn round at the bedside table')

    def test_lengths_and_dijkstra(self):
        self.assertAlmostEqual(self.net.shortest('pharmacy', 'R3')[0], 0.63 + 0.67 + 2.90 + 1.90, places=3)
        self.assertEqual(self.net.shortest('R1', 'R4')[1], ['R1', 'J1', 'J2', 'R4'])
        self.assertLess(max(rp.delivery_legs(self.net, 4)[1]['length'] - 2 * L.STOP_AHEAD, 0), 6.0)

    def test_turn_table_matches_headings(self):
        for a in self.net.nodes:
            for b in self.net.nodes:
                if a != b:
                    self.net.plan(a, b)        # raises if the map's turn table disagrees

    def test_unknown_node(self):
        with self.assertRaises(KeyError):
            self.net.plan('home', 'R9')


class Scheduler(unittest.TestCase):
    def setUp(self):
        self.t0 = dt.datetime(2026, 11, 10, 8, 0)
        self.lengths = {1: 3.2, 2: 3.2, 3: 6.1, 4: 6.1}

    def test_queue_due_then_distance(self):
        ds = hs.demo_doses(self.t0)
        q = hs.order_queue(ds, self.t0, lambda r: self.lengths[r])
        self.assertEqual([d.room for d in q], [1, 3], 'room 1 first: same due time, shorter route')
        q = hs.order_queue(ds, self.t0 + dt.timedelta(minutes=10), lambda r: self.lengths[r])
        self.assertEqual([d.room for d in q], [1, 3, 4, 2], 'by due time, ties by route length')

    def test_not_due_yet(self):
        ds = hs.demo_doses(self.t0)
        self.assertEqual(hs.order_queue(ds, self.t0 - dt.timedelta(seconds=1), lambda r: 0), [])

    def test_demo_schedule(self):
        ds = hs.demo_doses(self.t0)
        self.assertEqual(sorted(d.room for d in ds), [1, 2, 3, 4], 'one dose per room')
        self.assertEqual(len({d.medicine for d in ds}), 4, '4 different medicines')
        dues = sorted(d.due for d in ds)
        self.assertEqual(dues[0], dues[1], 'two doses due at the same time')
        self.assertLessEqual((dues[-1] - dues[0]).total_seconds(), 10 * 60, 'a few hospital minutes apart')
        stock = hs.Stock(__import__('pharmacy').load_inventory())
        for d in ds:
            self.assertIsNotNone(stock.slot_for(d.medicine, d.dose), 'demo medicine %s in stock' % d.medicine)

    def test_lateness(self):
        self.assertEqual(hs.lateness_min(self.t0 + dt.timedelta(minutes=3, seconds=30), self.t0), 3.5)
        self.assertEqual(hs.lateness_min(self.t0, self.t0), 0.0)

    def test_stock_and_out_of_stock(self):
        st = hs.Stock(copy.deepcopy(INV))
        self.assertEqual(st.slot_for('Paracetamol', '500 mg'), 'S1')
        st.used.add('S1')
        self.assertEqual(st.slot_for('paracetamol', '500mg'), 'S6', 'second container of the same medicine')
        st.used.add('S6')
        self.assertIsNone(st.slot_for('Paracetamol', '500 mg'), 'out of stock')
        self.assertIsNone(st.slot_for('Metformin', '500 mg'), 'quantity 0 = empty slot')
        self.assertIsNone(st.slot_for('Aspirin', '100 mg'), 'not in the inventory')
        self.assertEqual(st.left('Ibuprofen', '400 mg'), 1)
        # --list plans like a real run: a container serves one dose (3 Paracetamol doses, 2 containers)
        ds = [hs.Dose(i + 1, 'P', 1, 'Paracetamol', '500 mg', self.t0 + dt.timedelta(hours=i)) for i in range(3)]
        plan = hs.plan_slots(ds, hs.Stock(copy.deepcopy(INV)), lambda r: self.lengths[r])
        self.assertEqual([s for _, s in plan], ['S1', 'S6', None], 'second container, then out of stock')

    def test_hospital_clock(self):
        sim = {'t': 100.0}
        c = hs.HospitalClock(dt.datetime(2026, 11, 10, 8, 0), lambda: sim['t'])
        self.assertEqual(c.now(), dt.datetime(2026, 11, 10, 8, 0))
        sim['t'] = 100.0 + 185.0
        self.assertEqual(c.now(), dt.datetime(2026, 11, 10, 8, 3, 5))
        self.assertEqual(c.at(160.0), dt.datetime(2026, 11, 10, 8, 1))

    def test_real_clock_time_zone(self):
        c = hs.RealClock('Asia/Dubai')
        self.assertEqual(c.now().utcoffset(), dt.timedelta(hours=4))

    def test_doses_file(self):
        tz, entries = hs.load_doses()
        self.assertEqual(tz, 'Asia/Dubai')
        self.assertTrue(entries)
        from zoneinfo import ZoneInfo
        ds = hs.doses_for_day(entries, dt.date(2026, 11, 10), ZoneInfo(tz))
        self.assertTrue(all(d.due.tzinfo is not None for d in ds))

    def test_csv_upgrade_keeps_old_rows(self):
        d = tempfile.mkdtemp()
        path = os.path.join(d, 'deliveries.csv')
        with open(path, 'w', newline='') as fh:
            w = csv.DictWriter(fh, fieldnames=['due', 'patient', 'success'])
            w.writeheader()
            w.writerow({'due': 'x', 'patient': 'old', 'success': True})
        hs.write_row(path, {'due': 'y', 'patient': 'new', 'room': 3, 'success': False})
        with open(path, newline='') as fh:
            rows = list(csv.DictReader(fh))
        self.assertEqual([r['patient'] for r in rows], ['old', 'new'])
        self.assertEqual(rows[1]['room'], '3')
        self.assertEqual(rows[0]['room'], '')
        # the single-room scheduler shares this file: it must keep the hospital columns and values
        import scheduler as single
        single.log = lambda msg: None
        single.write_log_row(path, {'due': 'z', 'patient': 'single', 'bed': 1, 'success': True})
        with open(path, newline='') as fh:
            rd = csv.DictReader(fh)
            rows = list(rd)
        self.assertEqual([r['patient'] for r in rows], ['old', 'new', 'single'])
        self.assertEqual(rows[1]['room'], '3', 'hospital column kept by the single-room scheduler')
        self.assertTrue(set(hs.LOG_FIELDS) <= set(rd.fieldnames) and 'bed' in rd.fieldnames)
        self.assertEqual(rows[2]['bed'], '1')
        hs.write_row(path, {'due': 'w', 'patient': 'hospital again', 'room': 2, 'slot': 'S2'})
        with open(path, newline='') as fh:
            rows = list(csv.DictReader(fh))
        self.assertEqual([(r['patient'], r['room'], r['bed']) for r in rows][2:],
                         [('single', '', '1'), ('hospital again', '2', '')])


def _res(room=3, slot='S3', placed_err=0.0005, tilt_q=(0, 0, 0, 1), raised=True, others_move=0.0, wrong=False):
    center = list(L.room_info(room)['platform_center'])
    target = list(L.room_info(room)['place_target'])
    name = 'med_' + slot
    start = {'med_S%d' % k: {'xyz': [L.SLOT_X['S%d' % k], 0.2, 0.13], 'quat_xyzw': [0, 0, 0, 1]} for k in range(1, 7)}
    final = copy.deepcopy(start)
    on = [target[0], target[1], 0.83]
    final[name] = {'xyz': on, 'quat_xyzw': list(tilt_q)}
    if wrong:                         # the neighbour got delivered instead
        final[name] = start[name]
        final['med_S4'] = {'xyz': on, 'quat_xyzw': [0, 0, 0, 1]}
    final['med_S1']['xyz'][0] += others_move
    return {'success': True, 'delivery': {'room': room, 'slot': slot, 'container': name, 'platform_center': center},
            'target': target, 'placed': {'xyz': [target[0] + placed_err, target[1], 0.13], 'quat_xyzw': [0, 0, 0, 1]},
            'table': {'raised': raised, 'lift_time_s': 18.6, 'height_after_lift': 0.80 if raised else 0.10,
                      'platform': {'center': center, 'size': [0.25, 0.25]},
                      'container_after_lift': {'xyz': on, 'quat_xyzw': list(tilt_q)}},
            'containers_start': start, 'containers_final': final, 'weld': [], 'legs': {}, 'sim_time': 262.0}


class Scoring(unittest.TestCase):
    def test_success(self):
        r = scoring.score_delivery(_res())
        self.assertTrue(r['success'], r)
        self.assertTrue(r['right_container'])
        self.assertTrue(r['others_in_place'])
        self.assertAlmostEqual(r['placement_error_mm'], 0.5, places=3)

    def test_wrong_container(self):
        r = scoring.score_delivery(_res(wrong=True))
        self.assertFalse(r['success'])
        self.assertFalse(r['right_container'])

    def test_other_container_moved(self):
        r = scoring.score_delivery(_res(others_move=0.02))
        self.assertFalse(r['success'])
        self.assertEqual(r['others_moved'], 'med_S1')

    def test_not_raised_or_off_target_or_tilted(self):
        self.assertFalse(scoring.score_delivery(_res(raised=False))['success'])
        self.assertFalse(scoring.score_delivery(_res(placed_err=0.04))['success'])
        q = (math.sin(math.radians(15) / 2), 0, 0, math.cos(math.radians(15) / 2))
        self.assertFalse(scoring.score_delivery(_res(tilt_q=q))['success'])

    def test_mission_failed(self):
        self.assertFalse(scoring.score_delivery(_res(), mission_exit_ok=False)['success'])
        self.assertFalse(scoring.score_delivery({})['success'])

    def test_other_room(self):
        for room in (1, 2, 4):
            self.assertTrue(scoring.score_delivery(_res(room=room))['success'], room)


if __name__ == '__main__':
    unittest.main()
