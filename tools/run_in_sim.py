#!/usr/bin/env python3
"""Run one command against a fresh headless hospital-system simulation, then shut it down.

    python3 ~/sdp_sim/tools/run_in_sim.py [--launch-arg video:=true] [--name NAME] -- COMMAND ...

no simulation may run -> ros2 launch robot_gazebo hospital_system.launch.py gui:=false
[launch args] -> controllers active -> arm home -> COMMAND (its output -> command.log) ->
clean shutdown, no Gazebo process left. Logs in results/system/<NAME>_<date>/.
Exit code = the command's (2 = launch failed). Used for test step d:
    run_in_sim.py --name delivery_room3 -- python3 ~/sdp_sim/scripts/hospital_scheduler.py --now 3
"""
import argparse
import datetime as dt
import json
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import evaluate as ev           # noqa: E402


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--launch-arg', action='append', default=[])
    p.add_argument('--name', default='run')
    p.add_argument('cmd', nargs=argparse.REMAINDER)
    a = p.parse_args()
    cmd = a.cmd[1:] if a.cmd and a.cmd[0] == '--' else a.cmd
    if not cmd:
        p.error('no command')
    if ev.gazebo_pids():
        sys.exit('a simulation is already running (%s) - stop it first' % ev.gazebo_pids())
    out = os.path.join(ROOT, 'results', 'system', '%s_%s' % (a.name, dt.datetime.now().strftime('%Y%m%d_%H%M%S')))
    os.makedirs(out, exist_ok=True)
    t0 = time.monotonic()
    proc, fh = ev.start_sim(os.path.join(out, 'launch.log'), 'hospital_system.launch.py', a.launch_arg)
    code, summary = 2, {'command': cmd}
    try:
        err = ev.wait_ready(proc, os.path.join(out, 'launch.log'), 1800)
        if err:
            raise RuntimeError('launch: ' + err)
        ev.log('controllers active after %.0f s; arm home' % (time.monotonic() - t0))
        subprocess.run([sys.executable, os.path.join(ROOT, 'scripts', 'jetarm_pick_place.py'), 'home'],
                       capture_output=True, text=True, timeout=3600)
        ev.log('running: %s' % ' '.join(cmd))
        with open(os.path.join(out, 'command.log'), 'w') as fl:
            code = subprocess.run(cmd, stdout=fl, stderr=subprocess.STDOUT).returncode
        ev.log('command exited with %d' % code)
    except (RuntimeError, subprocess.TimeoutExpired) as e:
        ev.log('FAILED: %s' % e)
        summary['error'] = str(e)
    finally:
        summary['leftover_processes'] = ev.stop_sim(proc, fh)
        summary.update(exit_code=code, wall_s=round(time.monotonic() - t0, 1))
        json.dump(summary, open(os.path.join(out, 'summary.json'), 'w'), indent=2)
        ev.log('done in %.0f s wall; logs in %s' % (summary['wall_s'], out))
    sys.exit(code)


if __name__ == '__main__':
    main()
