#!/usr/bin/env python3
"""Route follower for the hospital line network: the tested line follower + junctions.

    python3 route_follower.py --from pharmacy --to R3 [--heading 180] [--turn-round]
    python3 route_follower.py --plan plan.json [--result-json out.json] [--fast-speed 0.12]

Extends line_follower.py WITHOUT changing its tested parts: the same image strips,
blue line detection, PD steering (LineFollower.analyse / LineFollower.steer) and the
same red stop (red fraction of the strips > --stop-frac for 2 frames). Added:

  * a plan from route_planner.py: the markers on the way, in order, each with its
    colour, map position and action (pass | straight | left | right | stop);
  * a look-ahead band (image rows 50-95 %, 0.19-0.35 m ahead of joint1) that sees a
    marker before the steering strips do (green: junction, red: a marker to pass);
  * every marker must match the plan: colour, order and position - the marker's
    distance ahead (map position vs /odom) must be inside a window when it is seen.
    A marker of the wrong colour, one outside its window, or a planned marker that
    the robot reaches without seeing it -> stop with an error (exit 3);
  * straight / pass: BLIND on /odom (heading of the map edge held, small sideways
    correction onto the edge) until the marker and the branch lines have left the
    strips, still watching for the next marker - a branch can never steer the robot;
  * left / right: stop, drive onto the junction centre on /odom, rotate to the next
    edge's heading on /odom (base_moves.BaseMover), then follow the tested way again;
  * --fast-speed V: V m/s on long straights only (>= 0.8 m before the next marker and
    >= 0.3 m after the last one), the normal speed near junctions and markers.

RouteLogic has no ROS in it (tools/offline_route_sim.py runs it with a synthetic camera).
Exit code 0 = arrived at the planned stop marker, 2 = line lost, 3 = route error, 1 = other.
"""
import argparse
import json
import math
import os
import sys
import time

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import line_follower as lf          # noqa: E402  (tested follower: analyse, steer, colour ranges)

GREEN_HSV = ((45, 120, 60), (85, 255, 255))   # junction markers (world: hue ~65)
LOOK = (0.50, 0.95)              # look-ahead band, fractions of the image height
LOOK_FRAC = 0.02                 # fraction of the band in a marker colour = marker in view
LOOK_WINDOW = (0.15, 0.62)       # m: marker centre ahead of joint1 when a look-ahead marker is confirmed
STOP_WINDOW = (0.15, 0.45)       # m: marker centre ahead at a red stop (Gazebo: 0.2777)
MISSED_D = 0.12                  # m: a planned marker this close without being seen = missed
BLIND_END = 0.05                 # m: blind pass ends when the marker centre is this close ahead
MAX_XTE = 0.08                   # m: cross-track error vs the map edge that stops the robot
FAST_AHEAD, FAST_AFTER = 0.80, 0.30


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class RouteLogic:
    """Route logic for one leg. base: pose_now() -> (x, y, yaw, t) without waiting,
    send(vx, vy, wz), and for the maneuvers (main loop only, they block):
    pose() -> (x, y, yaw, t), translate(dx, dy, vmax), rotate(dyaw), stop()."""

    def __init__(self, a, plan, base, log=print):
        self.a, self.plan, self.base, self.log = a, plan, base, log
        self.events = [dict(e) for e in plan['events']]
        self.k = 0
        self.mode = 'follow'
        self.prev_t, self.prev_err = None, 0.0          # used by LineFollower.steer
        self.red_frames = 0
        self.look = {'red': 0, 'green': 0}
        self.lost_since = None
        self.t_start = None
        self.result, self.error = None, None
        self.pending = None
        self.last_event_xy = None
        self.done = []                  # what happened, per event (-> result JSON)
        self.max_xte = 0.0
        self.frames = 0
        self.fast_frames = 0
        self.debug = None

    # ---------------------------------------------------------------- geometry
    def event(self):
        return self.events[self.k] if self.k < len(self.events) else None

    @staticmethod
    def ahead(ev, x, y):
        """Signed distance of the event marker ahead of joint1 along its edge."""
        h = math.radians(ev['edge_heading'])
        return (ev['x'] - x) * math.cos(h) + (ev['y'] - y) * math.sin(h)

    @staticmethod
    def xte(ev, x, y):
        """Cross-track error vs the event's edge line (+ = robot left of it)."""
        h = math.radians(ev['edge_heading'])
        return -(x - ev['x']) * math.sin(h) + (y - ev['y']) * math.cos(h)

    def since_last(self, ev, x, y):
        if self.last_event_xy is None:
            return 1e9
        h = math.radians(ev['edge_heading'])
        lx, ly = self.last_event_xy
        return (x - lx) * math.cos(h) + (y - ly) * math.sin(h)

    # ---------------------------------------------------------------- image
    def look_fractions(self, bgr):
        h = bgr.shape[0]
        band = bgr[int(LOOK[0] * h):int(LOOK[1] * h)]
        hsv = cv2.cvtColor(cv2.GaussianBlur(band, (3, 3), 0), cv2.COLOR_BGR2HSV)
        n = float(hsv.shape[0] * hsv.shape[1])
        red = sum(cv2.inRange(hsv, np.array(lo), np.array(hi)) for lo, hi in lf.RED_HSV)
        green = cv2.inRange(hsv, np.array(GREEN_HSV[0]), np.array(GREEN_HSV[1]))
        return {'red': np.count_nonzero(red) / n, 'green': np.count_nonzero(green) / n}

    # ---------------------------------------------------------------- results
    def fail(self, msg):
        self.base.send(0.0, 0.0, 0.0)
        self.result, self.error = 'error', msg
        self.log('ROUTE ERROR: ' + msg)

    def record(self, ev, how, x, y, yaw, t, d):
        self.done.append({'node': ev['node'], 'action': ev['action'], 'how': how, 'sim_t': round(t, 3),
                          'pose': [round(x, 4), round(y, 4), round(math.degrees(yaw), 2)],
                          'marker_ahead': round(d, 4)})
        self.log('[sim %.1f] %s %s (%s): marker %.3f m ahead, robot (%.3f, %.3f, %.1f deg)'
                 % (t, ev['action'], ev['node'], how, d, x, y, math.degrees(yaw)))

    def next_event(self, x, y):
        self.k += 1
        self.mode = 'follow'
        self.prev_t, self.prev_err = None, 0.0
        self.red_frames = 0
        self.look = {'red': 0, 'green': 0}
        self.lost_since = None
        self.last_event_xy = (x, y)
        if self.k >= len(self.events) and self.result is None:
            self.fail('route ended without a stop event')

    # ---------------------------------------------------------------- per camera frame
    def on_frame(self, bgr, t):
        if self.result or self.pending:
            return
        self.frames += 1
        if self.t_start is None:
            self.t_start = t
        x, y, yaw, _ = self.base.pose_now()
        ev = self.event()
        if ev is None:
            return self.fail('no event left')
        fr = self.look_fractions(bgr)                   # before analyse(): it draws on the image
        angle, red_frac = lf.LineFollower.analyse(self, bgr)
        self.debug = (angle, red_frac, fr)
        d = self.ahead(ev, x, y)
        e = self.xte(ev, x, y)
        self.max_xte = max(self.max_xte, abs(e))
        if abs(e) > MAX_XTE:
            return self.fail('%.3f m off the line to %s (robot %.3f, %.3f)' % (e, ev['node'], x, y))

        # first frame: a marker to pass that is already too close to be seen in its window
        if self.frames == 1 and ev['action'] in ('pass', 'straight') and d < LOOK_WINDOW[0]:
            if d <= BLIND_END:
                self.record(ev, 'already passed at the start', x, y, yaw, t, d)
                self.next_event(x, y)
                ev = self.event()
                d = self.ahead(ev, x, y)
            else:
                self.record(ev, 'blind from the start (too close to see)', x, y, yaw, t, d)
                self.mode = 'blind'

        if self.mode == 'blind':
            if d <= BLIND_END:
                self.next_event(x, y)
                ev = self.event()
                d = self.ahead(ev, x, y)
                e = self.xte(ev, x, y)
            else:
                h = math.radians(ev['edge_heading'])
                vy = max(-0.03, min(0.03, -1.0 * e))
                self.base.send(self.a.speed, vy, 1.5 * wrap(h - yaw))
                return

        # ---- follow (tested steering) + marker checks
        if ev['action'] == 'stop':
            if t - self.t_start > self.a.ignore_stop and red_frac > self.a.stop_frac:
                self.red_frames += 1
            else:
                self.red_frames = 0
            if self.red_frames >= 2:
                self.base.send(0.0, 0.0, 0.0)
                if STOP_WINDOW[0] <= d <= STOP_WINDOW[1]:
                    self.record(ev, 'red stop (strips)', x, y, yaw, t, d)
                    self.result = 'arrived'
                else:
                    self.fail('red marker in the strips, but %s is %.2f m ahead (window %.2f-%.2f)'
                              % (ev['node'], d, STOP_WINDOW[0], STOP_WINDOW[1]))
                return
            if d < MISSED_D:
                return self.fail('missed the stop marker %s (%.2f m ahead, not seen)' % (ev['node'], d))
        else:
            if d < MISSED_D:
                return self.fail('missed the %s marker %s (%.2f m ahead, not seen)' % (ev['colour'], ev['node'], d))
        for c in ('red', 'green'):
            self.look[c] = self.look[c] + 1 if fr[c] > LOOK_FRAC else 0
        want = ev['colour'] if ev['action'] != 'stop' else None
        for c in ('red', 'green'):
            if self.look[c] < 2:
                continue
            if ev['action'] == 'stop' and c == 'red':
                continue                                   # the stop marker itself, handled by the strips
            if c != want:
                return self.fail('unexpected %s marker (next planned: %s %s, %.2f m ahead)'
                                 % (c, ev['action'], ev['node'], d))
            if not LOOK_WINDOW[0] <= d <= LOOK_WINDOW[1]:
                return self.fail('%s marker seen, but %s is %.2f m ahead (window %.2f-%.2f)'
                                 % (c, ev['node'], d, LOOK_WINDOW[0], LOOK_WINDOW[1]))
            if ev['action'] in ('pass', 'straight'):
                self.record(ev, 'seen, blind pass', x, y, yaw, t, d)
                self.mode = 'blind'
                h = math.radians(ev['edge_heading'])
                self.base.send(self.a.speed, max(-0.03, min(0.03, -e)), 1.5 * wrap(h - yaw))
            else:
                self.record(ev, 'seen, turn %s' % ev['action'], x, y, yaw, t, d)
                self.base.send(0.0, 0.0, 0.0)
                self.pending = ('turn', ev)
            return

        if angle is None:
            self.base.send(0.0, 0.0, 0.0)
            self.lost_since = self.lost_since if self.lost_since is not None else t
            if t - self.lost_since > self.a.lost_timeout:
                self.result, self.error = 'lost', 'line lost for %.1f s' % self.a.lost_timeout
                self.log('Line lost for %.1f s - stopping.' % self.a.lost_timeout)
            return
        self.lost_since = None
        speed = self.a.speed
        if self.a.fast_speed and d >= FAST_AHEAD and self.since_last(ev, x, y) >= FAST_AFTER:
            speed = self.a.fast_speed
            self.fast_frames += 1
        vx, wz = lf.LineFollower.steer(self, angle, t, speed)
        self.base.send(vx, 0.0, wz)

    # ---------------------------------------------------------------- maneuvers (main loop)
    def run_pending(self):
        kind, ev = self.pending
        self.base.stop()
        x, y, yaw, t = self.base.pose()
        dxw, dyw = ev['x'] - x, ev['y'] - y
        c, s = math.cos(yaw), math.sin(yaw)
        self.base.translate(c * dxw + s * dyw, -s * dxw + c * dyw, 0.05)
        x, y, yaw, t = self.base.pose()
        dyaw = wrap(math.radians(ev['out_heading']) - yaw)
        self.base.rotate(dyaw)
        x, y, yaw, t = self.base.pose()
        self.log('[sim %.1f] at %s: on the junction centre (%.3f, %.3f), turned %s to %.1f deg'
                 % (t, ev['node'], x, y, ev['action'], math.degrees(yaw)))
        self.done[-1]['after_turn'] = [round(x, 4), round(y, 4), round(math.degrees(yaw), 2)]
        self.pending = None
        self.next_event(x, y)

    def summary(self):
        return {'result': self.result, 'error': self.error, 'events': self.done, 'max_xte': round(self.max_xte, 4),
                'frames': self.frames, 'fast_frames': self.fast_frames,
                'plan': {'from': self.plan.get('from'), 'to': self.plan.get('to'),
                         'actions': [(e['node'], e['action']) for e in self.events]}}


# -------------------------------------------------------------------- ROS node
def make_node(a, plan):
    import rclpy  # noqa: F401
    from base_moves import BaseMover

    class Adapter:
        def __init__(self, bm):
            self.bm = bm
            self.send = bm.send
            self.stop = bm.stop
            self.pose = bm.pose
            self.rotate = bm.rotate

        def pose_now(self):
            return self.bm.pose(fresh=False)

        def translate(self, dx, dy, vmax):
            return self.bm.translate(dx, dy, vmax=vmax)

    class RouteFollowerNode(lf.LineFollower):
        def __init__(self):
            super().__init__(a)
            self.bm = BaseMover(self)
            self.logic = RouteLogic(a, plan, Adapter(self.bm), log=lambda m: self.get_logger().info(m))

        def on_image(self, msg):
            if self.logic.result or self.logic.pending or self.bm.odom is None:
                return
            try:
                bgr = lf.image_to_bgr(msg)
            except ValueError as e:
                self.get_logger().error(str(e))
                return
            t = self.stamp(msg)
            self.logic.on_frame(bgr, t)
            now = time.monotonic()
            if now - self.last_debug > 1.0 and self.logic.debug:
                self.last_debug = now
                angle, red_frac, fr = self.logic.debug
                ev = self.logic.event()
                txt = '%s angle=%s red=%.2f look r/g %.2f/%.2f next %s' % (
                    self.logic.mode, 'none' if angle is None else '%.3f' % angle, red_frac, fr['red'], fr['green'],
                    '%s %s' % (ev['action'], ev['node']) if ev else '-')
                cv2.putText(bgr, txt, (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 0, 0), 1)
                cv2.imwrite('/tmp/line_follower_debug.jpg', bgr)
                self.get_logger().info('sim t=%.1f  %s' % (t - (self.logic.t_start or t), txt))

    return RouteFollowerNode()


def build_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--plan', default=None, help='plan JSON (route_planner.Network.plan)')
    p.add_argument('--from', dest='src', default=None)
    p.add_argument('--to', dest='dst', default=None)
    p.add_argument('--heading', type=float, default=None, help='robot heading at FROM (deg)')
    p.add_argument('--turn-round', action='store_true', help='do the planned turn round first (base_moves)')
    p.add_argument('--offset', type=float, default=0.12, help='turn-round strafe (m)')
    p.add_argument('--result-json', default=None)
    p.add_argument('--fast-speed', type=float, default=0.0, help='speed on long straights (0 = off)')
    # the tested line follower's parameters (same defaults)
    p.add_argument('--speed', type=float, default=0.08)
    p.add_argument('--kp', type=float, default=1.2)
    p.add_argument('--kd', type=float, default=0.1)
    p.add_argument('--max-turn', type=float, default=0.6)
    p.add_argument('--min-area', type=float, default=15)
    p.add_argument('--stop-frac', type=float, default=0.15)
    p.add_argument('--ignore-stop', type=float, default=0.0)
    p.add_argument('--lost-timeout', type=float, default=2.0)
    p.add_argument('--show', action='store_true')
    return p.parse_args(argv)


def load_plan(a):
    if a.plan:
        with open(a.plan) as fh:
            return json.load(fh)
    if not (a.src and a.dst):
        sys.exit('give --plan FILE or --from NODE --to NODE')
    import route_planner
    return route_planner.Network().plan(a.src, a.dst, a.heading)


def main():
    a = build_args()
    plan = load_plan(a)
    import rclpy
    rclpy.init()
    node = make_node(a, plan)
    node.get_logger().info('route %s -> %s: %s' % (plan['from'], plan['to'], ', '.join(
        '%s %s' % (e['action'], e['node']) for e in plan['events'])))
    code = 1
    try:
        if plan.get('turn_round'):
            if not a.turn_round:
                sys.exit('the plan starts with a turn round: do it first or pass --turn-round')
            node.get_logger().info('turn round (strafe right %.2f, rotate 180, strafe right %.2f)' % (a.offset, a.offset))
            node.bm.turnaround(a.offset)
        while rclpy.ok() and node.logic.result is None:
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.logic.pending:
                node.logic.run_pending()
        code = {'arrived': 0, 'lost': 2, 'error': 3}.get(node.logic.result, 1)
    except KeyboardInterrupt:
        print('\nAborted.')
    except RuntimeError as e:
        node.logic.result, node.logic.error = 'error', 'maneuver failed: %s' % e
        code = 3
    finally:
        for _ in range(3):
            node.bm.send()
            time.sleep(0.05)
        out = node.logic.summary()
        try:
            x, y, yaw, t = node.bm.pose(fresh=False)
            out['final'] = [round(x, 4), round(y, 4), round(math.degrees(yaw), 2)]
            out['sim_t_end'] = t
            out['sim_t_start'] = node.logic.t_start
        except Exception:      # noqa: BLE001 - never lose the result over the summary
            pass
        if a.result_json:
            with open(a.result_json, 'w') as fh:
                json.dump(out, fh, indent=2)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print('Result: %s%s' % (node.logic.result or 'aborted', (' (%s)' % node.logic.error) if node.logic.error else ''))
    print('Final: %s' % (out.get('final'),))
    sys.exit(code)


if __name__ == '__main__':
    main()
