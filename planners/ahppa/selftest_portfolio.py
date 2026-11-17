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

print(f"\n  {'ALL PASS' if not FAILS else 'FAILED: ' + ', '.join(FAILS)}\n")
sys.exit(1 if FAILS else 0)
