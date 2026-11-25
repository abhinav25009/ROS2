#!/usr/bin/env python3
"""Offline test of the LIVE portfolio wiring (portfolio.py), no simulator.

Covers what selftest.py cannot, because selftest.py exercises ahppa.py and
the navigator never uses ahppa.py:
  - observe() feeds the watchdog on the installed clock, down-sampled
  - deadlock_edge() fires once per trip
  - a tripped watchdog makes PortfolioPlanner.plan() dispatch ESCAPE
  - dwell timing follows the installed clock, not wall time
  - every plan lands in a persistent CSV

Needs ROS sourced (planner_3d imports core.octomap_3d_query):
  source /opt/ros/humble/setup.bash
  AHPPA_PORTFOLIO=1 python3 selftest_portfolio.py
"""
import csv
import glob
import os
import sys
import tempfile
from types import SimpleNamespace

import numpy as np

os.environ['AHPPA_PORTFOLIO'] = '1'
LOGDIR = tempfile.mkdtemp(prefix='ahppa_test_')
os.environ['AHPPA_LOG_DIR'] = LOGDIR

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import portfolio as P                                    # noqa: E402
from dispatcher import OPEN, ESCAPE                      # noqa: E402
from core.octomap_3d_query import OctoMap3DQuery         # noqa: E402

FAILS = []


def check(name, cond, detail=""):
    print(f"  [{'PASS' if cond else 'FAIL'}] {name}"
          f"{'  ' + detail if detail else ''}")
    if not cond:
        FAILS.append(name)


# -- a fake sim clock that does NOT advance with wall time ------------------
class SimClock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


clk = SimClock()
P.set_clock(clk)


def hover(seconds, x=0.0, y=0.0, z=1.8, hz=20, vx=0.0):
    """Drive observe() like the navigator's 20 Hz control loop."""
    out = False
    for _ in range(int(seconds * hz)):
        clk.t += 1.0 / hz
        x += vx / hz
        out = P.observe(x, y, z, goal_active=True)
    return out


print("\n-- watchdog via observe() --")
check("clock installed as sim", P.CLOCK_IS_SIM)
check("no trip while flying at 0.5 m/s", not hover(10, vx=0.5))
P.observe(0, 0, 1.8, goal_active=False)                  # goal cleared
check("goal_active=False resets the buffer", len(P.WATCHDOG.buf) == 0)
check("no trip before window fills (4 s hover)", not hover(4))
check("trips after ~6 s of hovering", hover(3))
check("window is ~6 s of sim time", 5.5 <= P.WATCHDOG.window_s <= 6.5,
      f"{P.WATCHDOG.window_s:.2f} s")
check("deadlock_edge fires once", P.deadlock_edge())
check("deadlock_edge then stays quiet", not P.deadlock_edge())
hover(1)
check("still quiet while the same deadlock lasts", not P.deadlock_edge())

# -- a planner on a small open map -------------------------------------------
res, W, H = 0.1, 100, 100                                # 10 m x 10 m
grid = SimpleNamespace(
    info=SimpleNamespace(resolution=res, width=W, height=H,
                         origin=SimpleNamespace(
                             position=SimpleNamespace(x=-5.0, y=-5.0))),
    data=[0] * (W * H))
octo = OctoMap3DQuery(resolution=res)
# a wall far from the start so clearance reads open
wall = np.array([[4.5, y, z] for y in np.arange(-4, 4, res)
                 for z in np.arange(0.0, 3.0, res)], dtype=np.float32)
octo.voxel_centers = wall
octo.occupied_set = set(octo._world_to_key(*p) for p in wall)

print("\n-- dispatch --")
import io, contextlib                                    # noqa: E402
with contextlib.redirect_stdout(io.StringIO()):
    pl = P.PortfolioPlanner(grid, octo, safety_margin=0.25)
    path = pl.plan(-3.0, 0.0, 3.0, 0.0, start_layer=1, goal_layer=1)
check("tripped watchdog -> ESCAPE", pl.last_mode == ESCAPE, pl.last_mode)
check("ESCAPE still returns a path", bool(path), f"{len(path or [])} wps")

# drone gets moving: watchdog clears, dispatcher leaves ESCAPE after dwell
hover(7, vx=0.5)
check("watchdog clears once the drone moves", not P.WATCHDOG.tripped)
with contextlib.redirect_stdout(io.StringIO()):
    pl.plan(-3.0, 0.0, 3.0, 0.0, start_layer=1, goal_layer=1)
check("leaves ESCAPE for OPEN in open space", pl.last_mode == OPEN,
      pl.last_mode)

print("\n-- clock --")
# dwell must follow the installed clock: freeze sim time, then try to switch
P.DISPATCH.min_dwell_s = 0.5
mode0 = P.DISPATCH.mode
clk.t += 0.1                                             # 0.1 s of sim time
P.DISPATCH.update(0.99, now=P.CLOCK())                   # wants DENSE
check("dwell blocks a switch 0.1 s (sim) after the last one",
      P.DISPATCH.mode == mode0, P.DISPATCH.mode)
clk.t += 1.0
P.DISPATCH.update(0.99, now=P.CLOCK())
check("switch allowed after 1.1 s of sim time", P.DISPATCH.mode != mode0,
      P.DISPATCH.mode)

print("\n-- first decision --")
from dispatcher import HysteresisDispatcher, DENSE, MEDIUM   # noqa: E402
for tight, want in ((0.90, DENSE), (0.50, MEDIUM), (0.10, OPEN)):
    d = HysteresisDispatcher()
    got = d.update(tight, now=5.0)
    check(f"first update at tightness {tight:.2f} -> {want}",
          got == want and d.switches == 0, got)
d = HysteresisDispatcher()
check("first update with deadlock -> ESCAPE",
      d.update(0.1, deadlock=True, now=5.0) == ESCAPE)

print("\n-- sloping ceiling --")
# A roof pitched 12.6 deg (as house_2floor's): z = 7.0 - 0.2235 * x, x in
# [0, 7]. Walls 3 m away on both sides at the flight altitude. With the
# default band the reading must come from the walls and NOT change as the
# drone flies under the slope - until the roof itself enters the band.
from clearance_feature import ClearanceEstimator, BAND_Z_DEFAULT  # noqa: E402
xs_ = np.arange(0.0, 7.01, 0.1)
roof = np.array([[x, y, 7.0 - 0.2235 * x] for x in xs_ for y in np.arange(-4, 4.01, 0.1)],
                dtype=np.float32)
walls = np.array([[x, y, z] for x in xs_ for y in (-3.0, 3.0) for z in np.arange(0.2, 7.0, 0.1)],
                 dtype=np.float32)
vox = np.concatenate([roof, walls])
dflt, old = ClearanceEstimator(), ClearanceEstimator(band_z=1.0)
check("default band is the planner's +/-0.3 m", abs(BAND_Z_DEFAULT - 0.3) < 1e-9 and dflt.band_z == 0.3)
at42 = [dflt.clearance(vox, (x, 0.0, 4.2)) for x in (1.0, 3.0, 5.0, 6.5)]
check("4.2 m under the slope: reading = wall distance everywhere",
      max(at42) - min(at42) < 0.05 and abs(at42[0] - 3.0) < 0.1,
      " ".join(f"{c:.2f}" for c in at42))
at48_old = [old.clearance(vox, (x, 0.0, 4.8)) for x in (1.0, 3.0, 5.0, 6.5)]
check("old +/-1.0 m band at 4.8 m: reading drifts along the slope",
      max(at48_old) - min(at48_old) > 1.0, " ".join(f"{c:.2f}" for c in at48_old))
at48 = [dflt.clearance(vox, (x, 0.0, 4.8)) for x in (1.0, 3.0, 5.0, 6.5)]
check("default band at 4.8 m: steady at wall distance",
      max(at48) - min(at48) < 0.05, " ".join(f"{c:.2f}" for c in at48))
eave = dflt.clearance(vox, (6.5, 0.0, 5.4))     # roof at 5.55 m, 0.15 m above
check("roof within 0.3 m of the drone (eaves, 5.4 m) still counts", eave < 0.3, f"{eave:.2f}")

print("\n-- persistent log --")
files = glob.glob(os.path.join(LOGDIR, 'portfolio_*.csv'))
check("one CSV created", len(files) == 1, LOGDIR)
rows = list(csv.DictReader(open(files[0]))) if files else []
check("one row per plan", len(rows) == 2, f"{len(rows)} rows")
if rows:
    check("row records ESCAPE + deadlock",
          rows[0]['mode'] == ESCAPE and rows[0]['deadlock'] == '1')
    check("row records sim clock", rows[0]['sim_clock'] == '1')
    check("row t is sim time", abs(float(rows[0]['t']) - 100) < 60,
          rows[0]['t'])

print("\n-- member time budget --")
# A long detour: 300 x 300 cells, a wall across with a gap at the far end.
from lazy_theta import LazyThetaStar                     # noqa: E402
from clearance_astar import ClearanceAStar               # noqa: E402
import time as _time                                     # noqa: E402
g3 = np.zeros((1, 300, 300), bool)
g3[0, 150, :290] = True
for cls in (LazyThetaStar, ClearanceAStar):
    m = cls([1.8], resolution=0.1, origin_x=0.0, origin_y=0.0)
    full = m.plan(g3, (5.0, 2.0, 1.8), (5.0, 28.0, 1.8))
    check(f"{cls.__name__}: no budget -> path, gave_up None",
          bool(full) and m.gave_up is None, f"{m.expansions} expansions")
    m.time_budget_s = 1e-4
    t0 = _time.perf_counter()
    p = m.plan(g3, (5.0, 2.0, 1.8), (5.0, 28.0, 1.8))
    dt = _time.perf_counter() - t0
    check(f"{cls.__name__}: tiny budget -> None, gave_up 'budget'",
          p is None and m.gave_up == 'budget', f"{dt * 1000:.0f} ms, {m.expansions} expansions")
    m.time_budget_s = None
    check(f"{cls.__name__}: budget cleared -> path again", bool(m.plan(g3, (5.0, 2.0, 1.8), (5.0, 28.0, 1.8))))

check("portfolio default budget is 5 s", abs(P.MEMBER_BUDGET_S - 5.0) < 1e-9)
with contextlib.redirect_stdout(io.StringIO()):
    pl2 = P.PortfolioPlanner(grid, octo, safety_margin=0.25)
check("budget installed on both members",
      pl2._lt.time_budget_s == 5.0 and pl2._ca.time_budget_s == 5.0)


def _over_budget(*_a, **_k):
    pl2._lt.gave_up = 'budget'
    return None


pl2._lt.plan = _over_budget
P.DISPATCH = HysteresisDispatcher()
P.WATCHDOG.reset()
buf = io.StringIO()
with contextlib.redirect_stdout(buf):
    path = pl2.plan(-3.0, 0.0, 3.0, 0.0, start_layer=1, goal_layer=1)
check("member over budget -> stock A* path returned",
      pl2.last_mode == OPEN and bool(path) and pl2.last_gave_up == 'budget', pl2.last_mode)
check("console says why", 'over its 5 s budget' in buf.getvalue(), buf.getvalue().strip()[-90:])
rows = list(csv.DictReader(open(files[0]))) if files else []
check("log row: OPEN->A*, error 'gave up (budget)'",
      rows and rows[-1]['planner'] == 'OPEN->A*' and rows[-1]['error'] == 'gave up (budget)',
      str(rows[-1] if rows else ''))
P.MEMBER_BUDGET_S = 0.0
with contextlib.redirect_stdout(io.StringIO()):
    pl3 = P.PortfolioPlanner(grid, octo, safety_margin=0.25)
check("AHPPA_MEMBER_BUDGET_S=0 disables the cap", pl3._lt.time_budget_s is None)
P.MEMBER_BUDGET_S = 5.0

print("\n-- race (AHPPA_RACE) --")
PARENT = os.getpid()
real_stock = P.MultiLayerPlanner.plan


def no_zombies():
    try:
        return os.waitpid(-1, os.WNOHANG) == (0, 0) and False
    except ChildProcessError:
        return True                                      # no children at all


def race_plan(lt_delay=None, st_delay=None, lt_fail=False, st_crash=False):
    """One race on the open map; delays/failures apply only in the children."""
    with contextlib.redirect_stdout(io.StringIO()):
        pl = P.PortfolioPlanner(grid, octo, safety_margin=0.25)
    real_lt = pl._lt.plan

    def lt(*a, **k):
        if os.getpid() != PARENT:
            if lt_delay:
                _time.sleep(lt_delay)
            if lt_fail:
                return None
        return real_lt(*a, **k)

    def st(self, *a, **k):
        if os.getpid() != PARENT:
            if st_crash:
                os._exit(3)
            if st_delay:
                _time.sleep(st_delay)
        return real_stock(self, *a, **k)
    pl._lt.plan = lt
    P.MultiLayerPlanner.plan = st
    P.WATCHDOG.reset()
    t0 = _time.perf_counter()
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            path = pl.plan(-3.0, 0.0, 3.0, 0.0, start_layer=1, goal_layer=1)
    finally:
        P.MultiLayerPlanner.plan = real_stock
    return pl, path, _time.perf_counter() - t0


P.RACE, P.RACE_WAIT = True, 0.0
pl, path, dt = race_plan()
rows = list(csv.DictReader(open(files[0])))
check("race: mode RACE, path returned", pl.last_mode == 'RACE' and bool(path), pl.last_mode)
check("race: log row says who won, with times",
      rows[-1]['planner'] in ('RACE:lt', 'RACE:stock') and 't_' in rows[-1]['note'],
      f"{rows[-1]['planner']}  {rows[-1]['note']}")
check("race: no child processes left", no_zombies())
pl, path, dt = race_plan(lt_delay=2.0)
check("lt slow, wait 0 -> stock wins, lt killed early",
      rows and pl.last_race['p_stock'] and path == pl.last_race['p_stock'] and dt < 1.5,
      f"{dt:.2f} s")
check("  ... and the killed child is reaped", no_zombies())
pl, path, dt = race_plan(st_delay=2.0)
check("stock slow -> lt wins without waiting for stock",
      path and path == pl.last_race['p_lt'] and dt < 1.5 and 't_stock' not in pl.last_race,
      f"{dt:.2f} s")
P.RACE_WAIT = 2.0
pl, path, dt = race_plan(lt_delay=0.6, st_delay=0.4)
check("wait 2: lt at ~0.6 s beats the 3 x 0.4 s deadline -> lt",
      path == pl.last_race['p_lt'], str({k: v for k, v in pl.last_race.items() if k.startswith('t_')}))
pl, path, dt = race_plan(lt_delay=2.5, st_delay=0.3)
check("wait 2: lt later than 3 x stock -> stock, returned at ~0.9 s",
      path == pl.last_race['p_stock'] and dt < 1.6, f"{dt:.2f} s")
P.RACE_WAIT = 0.0
pl, path, dt = race_plan(lt_fail=True, st_delay=0.3)
check("lt finds nothing -> stock path", bool(path) and path == pl.last_race['p_stock'])
pl, path, dt = race_plan(st_crash=True)
check("stock child crashes -> lt path still used", bool(path) and path == pl.last_race['p_lt'])
P.RACE_HARD_S = 0.5
pl, path, dt = race_plan(lt_delay=3.0, st_delay=3.0)
check("both children hang -> killed at RACE_HARD_S, stock run in-process",
      bool(path) and pl.last_race['p_lt'] is None and pl.last_race['p_stock'] is None and dt < 2.5,
      f"{dt:.2f} s")
check("  ... nothing left running", no_zombies())
P.RACE_HARD_S = 30.0
P.RACE_OBSERVE = True
pl, path, dt = race_plan(lt_delay=0.2, st_delay=0.5)
check("observe mode times both children",
      't_lt' in pl.last_race and 't_stock' in pl.last_race and path == pl.last_race['p_lt'],
      str({k: v for k, v in pl.last_race.items() if k.startswith('t_')}))
P.RACE_OBSERVE = False
P.WATCHDOG.tripped = True
with contextlib.redirect_stdout(io.StringIO()):
    pl = P.PortfolioPlanner(grid, octo, safety_margin=0.25)
    pl.plan(-3.0, 0.0, 3.0, 0.0, start_layer=1, goal_layer=1)
check("deadlock still dispatches ESCAPE, not RACE", pl.last_mode == ESCAPE, pl.last_mode)
P.WATCHDOG.reset()
P.RACE = False

print(f"\n  {'ALL PASS' if not FAILS else 'FAILED: ' + ', '.join(FAILS)}\n")
sys.exit(1 if FAILS else 0)
