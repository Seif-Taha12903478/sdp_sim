#!/usr/bin/env python3
"""Route planner on the hospital line network (config/hospital_map.yaml, Dijkstra).

    python3 route_planner.py FROM TO [--heading DEG]       e.g.  route_planner.py pharmacy R3
    python3 route_planner.py --all                         every delivery leg

Nodes: home, pharmacy, junctions J1..Jn, rooms R1..R4 (each node is a floor marker:
red = stop marker, green = junction). Edges are straight tape segments with a length
and a heading; the turn at a junction (left / right / straight) follows from the
headings of the edge in and the edge out (the map also lists them in `turns`).

plan(FROM, TO, heading) -> dict:
  turn_round   True if the robot must turn round first (it faces away from the first
               edge - at a dead end: the pharmacy counter or a room table)
  events       one per node after FROM, in driving order:
                 {node, colour, action, x, y, edge_heading, out_heading}
               action: 'pass'     red marker on the way (home between pharmacy and J1)
                       'straight' | 'left' | 'right'   at a green junction marker
                       'stop'     red marker at TO (same stop as the tested line follower)
  length       metres of tape from FROM's marker to TO's marker
The line follower (route_follower.py) matches every marker it sees against this list
(colour, order and the position from odometry) and performs the action.
"""
import argparse
import heapq
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
MAP = os.path.join(ROOT, 'config', 'hospital_map.yaml')


def load_map(path=MAP):
    import yaml
    with open(path) as fh:
        return yaml.safe_load(fh)


def wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


class Network:
    def __init__(self, m=None):
        self.m = m or load_map()
        self.nodes = self.m['nodes']
        self.adj = {n: [] for n in self.nodes}
        for e in self.m['edges']:
            a, b, l = e['from'], e['to'], float(e['length'])
            self.adj[a].append((b, l))
            self.adj[b].append((a, l))

    def heading(self, a, b):
        na, nb = self.nodes[a], self.nodes[b]
        return math.degrees(math.atan2(nb['y'] - na['y'], nb['x'] - na['x']))

    def shortest(self, src, dst):
        """Dijkstra: (length, [src, ..., dst])."""
        if src not in self.nodes or dst not in self.nodes:
            raise KeyError('unknown node %s / %s (nodes: %s)' % (src, dst, ', '.join(self.nodes)))
        dist, prev, todo = {src: 0.0}, {}, [(0.0, src)]
        while todo:
            d, n = heapq.heappop(todo)
            if n == dst:
                break
            if d > dist.get(n, 1e18):
                continue
            for nb, l in self.adj[n]:
                nd = d + l
                if nd < dist.get(nb, 1e18):
                    dist[nb], prev[nb] = nd, n
                    heapq.heappush(todo, (nd, nb))
        if dst not in dist:
            raise ValueError('no route %s -> %s' % (src, dst))
        path = [dst]
        while path[-1] != src:
            path.append(prev[path[-1]])
        return dist[dst], path[::-1]

    def plan(self, src, dst, heading_deg=None):
        """heading_deg: the robot's heading at src (None = facing along the first edge)."""
        length, path = self.shortest(src, dst)
        if len(path) < 2:
            raise ValueError('already at %s' % dst)
        h0 = self.heading(path[0], path[1])
        turn_round = heading_deg is not None and abs(wrap_deg(h0 - heading_deg)) > 90.0
        events = []
        for i in range(1, len(path)):
            n = path[i]
            nd = self.nodes[n]
            h_in = self.heading(path[i - 1], n)
            ev = {'node': n, 'colour': nd['marker'], 'x': nd['x'], 'y': nd['y'], 'edge_heading': round(h_in, 3)}
            if i == len(path) - 1:
                ev['action'], ev['out_heading'] = 'stop', round(h_in, 3)
            else:
                h_out = self.heading(n, path[i + 1])
                d = wrap_deg(h_out - h_in)
                if nd['type'] == 'junction':
                    ev['action'] = 'straight' if abs(d) < 1.0 else ('left' if d > 0 else 'right')
                    tt = (self.m.get('turns') or {}).get(n, {}).get('%s->%s' % (path[i - 1], path[i + 1]))
                    if tt and tt != ev['action']:
                        raise ValueError('map turn table says %s at %s, headings say %s' % (tt, n, ev['action']))
                else:
                    if abs(d) > 1.0:
                        raise ValueError('route turns at the non-junction node %s' % n)
                    ev['action'] = 'pass'
                ev['out_heading'] = round(h_out, 3)
            events.append(ev)
        return {'from': src, 'to': dst, 'path': path, 'length': round(length, 4),
                'start_heading': round(h0, 3), 'turn_round': bool(turn_round), 'events': events}


def describe(p):
    acts = (['turn round'] if p['turn_round'] else []) + [
        '%s %s' % ('stop at' if e['action'] == 'stop' else ('pass' if e['action'] == 'pass' else e['node']),
                   e['node'] if e['action'] in ('stop', 'pass') else e['action'])
        for e in p['events']]
    return '%s -> %s (%.2f m): %s' % (p['from'], p['to'], p['length'], ', '.join(acts))


# robot headings at the dead ends after a stop (the robot faces the marker it stopped at)
def heading_at(net, node, came_from):
    return net.heading(came_from, node)


def delivery_legs(net, room):
    """The three legs of a delivery (home -> pharmacy -> room -> home) with the
    robot's heading at each start, as the delivery mission drives them."""
    r = 'R%d' % room
    leg1 = net.plan('home', 'pharmacy', heading_deg=0.0)             # parked at home facing +x
    leg2 = net.plan('pharmacy', r, heading_deg=heading_at(net, 'pharmacy', 'home'))
    j = net.shortest('pharmacy', r)[1][-2]
    leg3 = net.plan(r, 'home', heading_deg=heading_at(net, r, j))
    return leg1, leg2, leg3


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('src', nargs='?')
    p.add_argument('dst', nargs='?')
    p.add_argument('--heading', type=float, default=None, help="robot heading at FROM (deg)")
    p.add_argument('--all', action='store_true', help='print the legs of every delivery')
    p.add_argument('--json', action='store_true')
    a = p.parse_args()
    net = Network()
    if a.all:
        for room in sorted(net.m['rooms']):
            print('Room %s' % room)
            for leg in delivery_legs(net, int(room)):
                print('   ' + describe(leg))
        return
    if not (a.src and a.dst):
        p.error('give FROM and TO, or --all')
    pl = net.plan(a.src, a.dst, a.heading)
    print(json.dumps(pl, indent=2) if a.json else describe(pl))


if __name__ == '__main__':
    sys.exit(main())
