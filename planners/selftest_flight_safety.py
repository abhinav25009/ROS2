#!/usr/bin/env python3
"""Offline tests for the flight-safety fixes from the 2026-11-17 flight test
(docs/AHPPA_flight_test_report.pdf, section 9). No simulator needed.

  C3  _plan_layers: change floors vertically, fly level at the higher altitude
  C4  _check_crash: tilt past 60 deg for 0.25 s -> force-disarm, CRASHED
  C5  _fallback_ok: 2D fallback only on the ground floor
  C6  OctoMap3DQuery memory: thin obstacles survive being cleared

  source /opt/ros/humble/setup.bash && source ~/px4_ros_ws/install/setup.bash
  python3 planners/selftest_flight_safety.py
"""
import contextlib
import io
import os
import sys
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
with contextlib.redirect_stdout(io.StringIO()):
    import navigator_astar as N                                  # noqa: E402
from core.octomap_3d_query import OctoMap3DQuery                  # noqa: E402

Z = N.Z_LAYERS
FAILS = []


def check(name, cond, detail=''):
    print(f"  [{'PASS' if cond else 'FAIL'}] {name}{'  ' + detail if detail else ''}")
    if not cond:
        FAILS.append(name)


def bare_nav():
    n = N.AutonomousNavigator.__new__(N.AutonomousNavigator)
    n.state = 'NAVIGATING'
    n.states = []
    n._set_state = lambda s: (n.states.append(s), setattr(n, 'state', s))
    n.cmds = []
    n._send_command = lambda c, param1=0.0, param2=0.0: n.cmds.append((c, param1, param2))
    return n


class FakeOcto:
    def __init__(self, clear):
        self.clear = clear
        self.calls = []

    def is_column_clear(self, x, y, z0, z1, xy_radius=0.3):
        self.calls.append((x, y, z0, z1))
        return self.clear


class FakePlanner:
    """Straight line on whatever layer it is asked for; records calls."""
    def __init__(self):
        self.calls = []

    def plan(self, sx, sy, gx, gy, start_layer=None, goal_layer=None):
        self.calls.append((start_layer, goal_layer))
        zs, zg = Z[start_layer], Z[goal_layer if goal_layer is not None else start_layer]
        return [(sx, sy, zs), ((sx + gx) / 2, (sy + gy) / 2, zg), (gx, gy, zg)]


print('\n-- C3 floor changes --')
n = bare_nav()
n.octomap_3d = FakeOcto(True)
fp = FakePlanner()
with contextlib.redirect_stdout(io.StringIO()):
    p = n._plan_layers(fp, 2.9, 3.4, 4, 5.4, -3.2, 6)          # the crash goal: 3.6 -> 4.8 m
check('climb: starts with a vertical leg at the start xy',
      p[0] == (2.9, 3.4, Z[4]) and p[1][:2] == (2.9, 3.4) and p[1][2] == Z[6], str(p[:2]))
check('climb: whole horizontal leg at the higher altitude (4.8 m)',
      all(abs(q[2] - Z[6]) < 1e-9 for q in p[1:]))
check('climb: planner asked for a level path on the goal layer', fp.calls == [(6, 6)], str(fp.calls))
fp = FakePlanner()
with contextlib.redirect_stdout(io.StringIO()):
    p = n._plan_layers(fp, 5.4, -3.2, 6, 3.0, 0.0, 1)          # 4.8 m -> 1.8 m via the atrium
check('descend: level at the start altitude, vertical leg at the goal',
      all(abs(q[2] - Z[6]) < 1e-9 for q in p[:-1]) and p[-1] == (3.0, 0.0, Z[1]), str(p[-2:]))
n.octomap_3d = FakeOcto(False)
fp = FakePlanner()
with contextlib.redirect_stdout(io.StringIO()):
    p = n._plan_layers(fp, 2.9, 3.4, 4, 5.4, -3.2, 6)
check('blocked column: falls back to the layered planner', fp.calls == [(4, 6)], str(fp.calls))
fp = FakePlanner()
with contextlib.redirect_stdout(io.StringIO()):
    n._plan_layers(fp, 0, 0, 1, 4, -3, 1)
check('same floor: unchanged single plan call', fp.calls == [(1, 1)])

print('\n-- C5 2D fallback --')
n = bare_nav()
n.gz_pos = np.array([3.0, 3.5, 3.6])
check('refused upstairs', not n._fallback_ok(None))
n.gz_pos = np.array([1.0, 1.0, 1.8])
check('allowed on the ground floor for a ground-floor goal', n._fallback_ok(None) and n._fallback_ok(1))
check('refused on the ground floor for an upstairs goal', not n._fallback_ok(4))

print('\n-- C4 crash response --')
n = bare_nav()
n.CRASH_TILT_DEG, n.CRASH_HOLD_S, n._tilt_since = 60.0, 0.25, None
n.waypoints, n.goal_xy, n.goal_layer = [(0, 0, 1.8)], (1, 1), 1
clock = [100.0]
n._now = lambda: clock[0]
n.tilt_deg = 20.0
check('normal tilt: no action', not n._check_crash() and not n.cmds)
n.tilt_deg = 75.0
check('brief tilt spike: no action yet', not n._check_crash() and not n.cmds)
clock[0] += 0.1
n.tilt_deg = 30.0
n._check_crash()
clock[0] += 0.3
n.tilt_deg = 75.0
check('spike interrupted: timer restarts', not n._check_crash())
clock[0] += 0.3
with contextlib.redirect_stdout(io.StringIO()):
    hit = n._check_crash()
check('sustained tilt: crash declared', hit and n.state == 'CRASHED')
check('force-disarm sent (400, 0, 21196)', n.cmds == [(400, 0.0, 21196.0)], str(n.cmds))
check('goal and path cleared', n.waypoints == [] and n.goal_xy is None)
n.tilt_deg = 0.0
check('stays CRASHED, no repeat commands', n._check_crash() and len(n.cmds) == 1)
n = bare_nav()
n.state = 'INIT'
n.tilt_deg, n._tilt_since = 90.0, None
n._now = lambda: 0.0
check('ignored before takeoff (INIT)', not n._check_crash() and not n.cmds)

print('\n-- C6 occupancy memory --')
q = OctoMap3DQuery(resolution=0.10)
wall = np.array([[i, 0, 30] for i in range(50)], np.int64)
post = np.array([[27, 20, 35]], np.int64)
noise = np.array([[5, 5, 5]], np.int64)
k = q._apply_memory(np.vstack([wall, post]), 0.0)
k = q._apply_memory(np.vstack([wall, post, noise]), 2.0)
k = q._apply_memory(wall, 4.0)                 # post cleared by OctoMap
ks = set(map(tuple, k.tolist()))
check('post seen twice survives being cleared', (27, 20, 35) in ks, f'kept {q.remembered_only} by memory')
check('single-hit noise is not kept', (5, 5, 5) not in ks)
k = q._apply_memory(wall, 63.0)
check('post forgotten after the 60 s TTL', (27, 20, 35) not in set(map(tuple, k.tolist())))
check('encode/decode round trip incl. negatives',
      np.array_equal(q._decode(q._encode(np.array([[-500, 7, -3], [1000, -2000, 60]]))),
                     np.array([[-500, 7, -3], [1000, -2000, 60]])))
q2 = OctoMap3DQuery(resolution=0.10)
q2.MEMORY_TTL = 0
q2._apply_memory(np.vstack([wall, post]), 0.0)
q2._apply_memory(np.vstack([wall, post]), 2.0)
check('MEMORY_TTL = 0 disables memory',
      (27, 20, 35) not in set(map(tuple, q2._apply_memory(wall, 4.0).tolist())))

print('\n-- goal reached on a dense vertical descent --')
n = bare_nav()
n.WP_REACH_DIST = 0.25
n.waypoints = N.densify_path([(3.0, 0.0, 4.8), (3.0, 0.0, 1.8)])
n.gz_pos = np.array([2.96, 0.05, 1.87])                  # 7 cm above the goal, as flown
n.wp_idx = min(range(len(n.waypoints)), key=lambda i: n._wp_dist(n.waypoints[i]))
check('closest point is not the last one (the old failure)', n.wp_idx == len(n.waypoints) - 2,
      f'{n.wp_idx} of {len(n.waypoints)}')
check('now counts as reached', n._at_goal())
n.gz_pos = np.array([2.96, 0.05, 2.4])
n.wp_idx = min(range(len(n.waypoints)), key=lambda i: n._wp_dist(n.waypoints[i]))
check('0.6 m above the goal is not reached', not n._at_goal())
n.waypoints = N.densify_path([(0.0, 0.0, 1.8), (4.0, 0.0, 1.8)])
n.gz_pos = np.array([0.1, 0.0, 1.8])
n.wp_idx = 1
check('start of a path is not reached', not n._at_goal())

print('\n-- path check covers segments, not just waypoints --')


class WallOcto:
    """Occupied only on a thin wall x in [1.95, 2.05] - between waypoints."""
    is_ready = True

    def __init__(self):
        self.calls = 0

    def is_column_clear(self, x, y, z0, z1, xy_radius=0.3):
        self.calls += 1
        return not (1.95 - xy_radius <= x <= 2.05 + xy_radius)


n = bare_nav()
n.occupancy_grid = object()
n.octomap_3d = WallOcto()
n.gz_pos = np.array([0.0, 0.0, 1.8])
n.ALTITUDE = 1.8
n.waypoints = [(0.0, 0.0, 1.8), (4.0, 0.0, 1.8), (8.0, 0.0, 1.8)]   # wall between wp 0 and 1
n.wp_idx = 0
check('wall between two far-apart waypoints is detected', n._path_blocked())
n.waypoints = [(0.0, 0.0, 1.8), (1.0, 3.0, 1.8), (1.2, 8.0, 1.8)]   # never crosses x = 2
check('path that never crosses the wall is clear', not n._path_blocked())
n.waypoints = [(0.0, 0.0, 1.8), (-1.0, 0.0, 1.8), (-7.0, 0.0, 1.8), (-7.0, 9.0, 1.8), (3.0, 9.0, 1.8)]
n.octomap_3d = WallOcto()
check('wall beyond the 5 m horizon is not checked', not n._path_blocked())
check('horizon bounds the work', n.octomap_3d.calls < 120, f'{n.octomap_3d.calls} samples')
n.gz_pos = None
n.waypoints = [(0.0, 0.0, 1.8), (4.0, 0.0, 1.8)]
n.octomap_3d = WallOcto()
n.octomap_3d.is_ready = True
check('no pose yet -> falls back to the waypoint check', not n._segments_blocked())

print('\n-- AHPPA v3: fast "commanded but not moving" trigger --')


class FakeAhppa:
    V3 = True

    def __init__(self):
        self.reports = []

    def report_stuck(self, x, y, hx, hy, source='fast'):
        self.reports.append((x, y, hx, hy, source))
        return True


def fly_ticks(n, wps, poses, dt=0.05):
    """Feed poses (list, or fn(i) -> pose) at 20 Hz; return the ticks that fired."""
    n.waypoints, n.wp_idx = wps, 0
    fired = []
    for i in range(len(poses) if isinstance(poses, list) else 200):
        clock[0] += dt
        n.gz_pos = np.array(poses[i] if isinstance(poses, list) else poses(i))
        if n._fast_stuck():
            fired.append(i)
    return fired


fake = FakeAhppa()
saved_ahppa = N._ahppa
N._ahppa = fake
clock = [100.0]
n = bare_nav()
n._now = lambda: clock[0]
n._stuck_hist, n._stuck_wps = [], None
path = N.densify_path([(0.0, 0.0, 1.8), (6.0, 0.0, 1.8)])
f = fly_ticks(n, path, [(1.0, 0.0, 1.8)] * 60)                         # pinned 3 s
check('pinned: fires once the 1.5 s window is full', len(f) >= 1 and 28 <= f[0] <= 31, str(f))
check('pinned: then restarts (at most one event per 1.5 s)', all(b - a >= 29 for a, b in zip(f, f[1:])), str(f))
check('pinned: reports heading along the path', fake.reports and fake.reports[0][2] > 0.29
      and abs(fake.reports[0][3]) < 1e-9, str(fake.reports[:1]))
fake.reports.clear()
f = fly_ticks(n, N.densify_path([(0.0, 0.0, 1.8), (6.0, 0.0, 1.8)]),
              lambda i: (0.0 + 0.3 * 0.05 * i, 0.0, 1.8))                 # 0.3 m/s, slowest cruise
check('flying at 0.3 m/s never fires', not f, str(f))
f = fly_ticks(n, N.densify_path([(0.0, 0.0, 1.8), (0.0, 0.0, 3.6), (5.0, 0.0, 3.6)]),
              lambda i: (0.0, 0.0, 1.8 + 0.2 * 0.05 * i))                 # 0.2 m/s straight up
check('vertical climb never fires (3D distance)', not f, str(f))
g = N.densify_path([(0.0, 0.0, 1.8), (3.0, 0.0, 1.8)])
f = fly_ticks(n, g, [(2.7, 0.0, 1.8)] * 60)                           # settling 0.3 m from the goal
check('settling at the goal never fires', not f, str(f))
p1 = N.densify_path([(0.0, 0.0, 1.8), (6.0, 0.0, 1.8)])
n.waypoints, n.wp_idx = p1, 0
for i in range(20):                                                     # 1 s pinned on path 1
    clock[0] += 0.05
    n.gz_pos = np.array([1.0, 0.0, 1.8])
    n._fast_stuck()
f = fly_ticks(n, N.densify_path([(1.0, 0.0, 1.8), (1.0, 4.0, 1.8)]), [(1.0, 0.0, 1.8)] * 20)
check('a new path restarts the window (hover to plan is not "stuck")', not f, str(f))
N._ahppa = None
check('no-op without the portfolio', not n._fast_stuck())
N._ahppa = SimpleNamespace(V3=False)
check('no-op with AHPPA_V3 off', not n._fast_stuck())

print('\n-- AHPPA v3: impact trigger + back-off --')
N._ahppa = fake
fake.reports.clear()


def impact_nav():
    m = bare_nav()
    m._now = lambda: clock[0]
    m._tilt_hist, m._recover = [], None
    m._stuck_hist, m._stuck_wps = [], None
    m.px4_pos = np.array([0.0, 0.0, -1.8])
    m.sent, m.replans = [], []
    m._send_position = lambda x, y, z, yaw=0.0: m.sent.append((x, y, z, yaw))
    m.current_yaw = 1.5708                     # facing east (NED yaw)
    m.MAX_NAV_DELTA = 1.5
    m._replan = lambda: m.replans.append(clock[0])
    m.waypoints, m.wp_idx = N.densify_path([(0.0, 0.0, 1.8), (8.0, 0.0, 1.8)]), 0
    return m


def run(m, samples):
    """samples: list of (x, tilt) or (x, y, tilt); 20 Hz; returns ticks where _impact fired."""
    out = []
    for i, sm in enumerate(samples):
        x, y, tilt = sm if len(sm) == 3 else (sm[0], 0.0, sm[1])
        clock[0] += 0.05
        m.gz_pos = np.array([x, y, 1.8])
        m.tilt_deg = tilt
        if m._recovering():
            continue
        if m._impact():
            out.append(i)
    return out


m = impact_nav()
cruise = [(1.0 + 0.3 * 0.05 * i, 0.5) for i in range(40)]           # 2 s calm at 0.3 m/s
bump = [(1.6, 18.7), (1.5, 23.0), (1.3, 26.0)]
f = run(m, cruise + bump)
check('bump after calm cruise fires once', f == [40], str(f))
check('no barrier yet during the 0.3 s bounce phase', not fake.reports)
n_sent = len(m.sent)
run(m, [(1.2, 30.0)] * 10)                                          # thrown back to x=1.2, 0.5 s
r = fake.reports[-1] if fake.reports else None
check('after the bounce phase: barrier at the pre-impact pose, across the heading', r is not None and r[4] == 'impact'
      and 1.45 < r[0] < 1.55 and r[2] > 0.99 and abs(r[3]) < 1e-6, str(r))
check('backs off opposite the heading (-x), 0.6 m', m._recover is not None
      and abs(m._recover['target'][0] - (r[0] - 0.6)) < 1e-6, str(m._recover and m._recover['target']))
check('while recovering: holds setpoints, no replan yet', len(m.sent) > n_sent and not m.replans)
check('back-off setpoints keep the current yaw (not 0 = north)', all(abs(c[3] - 1.5708) < 1e-6 for c in m.sent[n_sent:]))
run(m, [(1.0, 18.0), (1.0, 4.0), (1.0, 28.0)] * 10)                 # oscillating through 5 deg
check('one level tick mid-oscillation does not end recovery', not m.replans)
run(m, [(1.0, 2.0)] * 40)                                           # level again for 2 s
check('replans once, after RECOVER_MIN_S and level', len(m.replans) == 1 and m._recover is None,
      str(m.replans))
m = impact_nav()
run(m, cruise + bump)
run(m, [(1.0, 25.0)] * 220)                                         # never steady: 11 s
check('recovery times out after RECOVER_MAX_S and replans anyway', len(m.replans) == 1, str(m.replans))
m = impact_nav()
f = run(m, [(1.0 + 0.01 * i, 3.0 + 0.2 * i) for i in range(40)])   # slow lean 3 -> 11 deg over 2 s
check('gradual tilt is not a bump', not f, str(f))
m = impact_nav()
f = run(m, [(1.0, 12.0)] * 30)                                      # tilted from the start
check('no calm history -> no trigger', not f, str(f))
m = impact_nav()
f = run(m, cruise + [(1.6, 8.0)] * 5)                               # 8 deg: below threshold
check('8 deg is below the impact threshold', not f, str(f))
N._ahppa = SimpleNamespace(V3=False)
m = impact_nav()
check('no-op with AHPPA_V3 off', not run(m, cruise + bump))
N._ahppa = saved_ahppa

print(f"\n  {'ALL PASS' if not FAILS else 'FAILED: ' + ', '.join(FAILS)}\n")
sys.exit(1 if FAILS else 0)
