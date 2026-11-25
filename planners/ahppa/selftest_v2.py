#!/usr/bin/env python3
"""Offline test of AHPPA v2 (refine.py + PortfolioPlanner._plan_v2).

  source /opt/ros/humble/setup.bash
  python3 planners/ahppa/selftest_v2.py
"""
import contextlib
import io
import math
import os
import sys
import tempfile
from types import SimpleNamespace

import numpy as np

os.environ['AHPPA_PORTFOLIO'] = '1'
os.environ['AHPPA_V2'] = '1'
os.environ['AHPPA_LOG_DIR'] = tempfile.mkdtemp(prefix='ahppa_v2_test_')

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import portfolio as P                                    # noqa: E402
import refine as R                                       # noqa: E402
from dispatcher import ESCAPE                            # noqa: E402
from core.octomap_3d_query import OctoMap3DQuery         # noqa: E402
from planners.planner_3d import MultiLayerPlanner        # noqa: E402

FAILS = []


def check(name, cond, detail=""):
    print(f"  [{'PASS' if cond else 'FAIL'}] {name}{'  ' + detail if detail else ''}")
    if not cond:
        FAILS.append(name)


class Clock:
    t = 50.0

    def __call__(self):
        return self.t


clk = Clock()
P.set_clock(clk)

# 12 m x 10 m room, an L-shaped wall between start and goal, and a pillar
res, W, H = 0.1, 120, 100
grid = SimpleNamespace(info=SimpleNamespace(resolution=res, width=W, height=H,
                       origin=SimpleNamespace(position=SimpleNamespace(x=-6.0, y=-5.0))),
                       data=[0] * (W * H))
pts = []
for y in np.arange(-5.0, 2.0, res):                      # vertical wall x = 0, gap above y = 2
    pts += [[0.0, y, z] for z in np.arange(0.0, 3.0, res)]
for x in np.arange(-3.0, 0.0, res):                      # foot of the L
    pts += [[x, -1.0, z] for z in np.arange(0.0, 3.0, res)]
for x in np.arange(2.5, 3.0, res):                       # pillar
    for y in np.arange(2.5, 3.0, res):
        pts += [[x, y, z] for z in np.arange(0.0, 3.0, res)]
octo = OctoMap3DQuery(resolution=res)
octo.voxel_centers = np.array(pts, dtype=np.float32)
octo.occupied_set = set(octo._world_to_key(*p) for p in octo.voxel_centers)

with contextlib.redirect_stdout(io.StringIO()):
    pl = P.PortfolioPlanner(grid, octo, safety_margin=0.3)
lt, G = pl._lt, pl._grids

print("\n-- shortcut() --")
with contextlib.redirect_stdout(io.StringIO()):
    raw = MultiLayerPlanner.plan(pl, -3.0, -3.0, 4.0, -3.0, start_layer=1, goal_layer=1)
check("stock A* finds the detour", bool(raw), f"{len(raw) if raw else 0} waypoints")
sm = R.shortcut(lt, G, raw)
check("smoothed path keeps endpoints", sm[0] == raw[0] and sm[-1] == raw[-1])
check("smoothed path has far fewer waypoints", len(sm) < len(raw) / 5, f"{len(raw)} -> {len(sm)}")
check("smoothed path is not longer", R.path_length(sm) <= R.path_length(raw) + 1e-9,
      f"{R.path_length(raw):.2f} -> {R.path_length(sm):.2f} m")
check("every smoothed SEGMENT is in free space", R.segments_clear(lt, G, sm))
check("shortcut of a 2-point path is a no-op", R.shortcut(lt, G, sm[:2]) == sm[:2])

print("\n-- corridor / refine() --")
cg = R.corridor_grids(lt, G, sm, 1.0)
check("corridor only adds blocked cells", bool((cg | G).sum() == cg.sum()) and cg.sum() > G.sum())
with contextlib.redirect_stdout(io.StringIO()):
    full = lt.plan(G, sm[0], sm[-1])
rf = R.refine(lt, G, sm, radius=1.0)
check("refined path is valid", R.segments_clear(lt, G, rf))
check("refined <= smoothed", R.path_length(rf) <= R.path_length(sm) + 1e-9,
      f"{R.path_length(sm):.3f} -> {R.path_length(rf):.3f} m")
check("refined ~= unrestricted Lazy Theta*", abs(R.path_length(rf) - R.path_length(full)) < 0.05,
      f"{R.path_length(rf):.3f} vs {R.path_length(full):.3f} m")
check("refine() restores the member's budget", lt.time_budget_s == P.MEMBER_BUDGET_S)

print("\n-- segments_clear() catches what a waypoint check misses --")
a, b = (-3.0, 0.5, 1.8), (3.0, 0.5, 1.8)                 # both ends free, wall in between
check("both waypoints free", lt._free(G, lt.to_cell(*a)) and lt._free(G, lt.to_cell(*b)))
check("segment through the wall is rejected", not R.segments_clear(lt, G, [a, b]))

print("\n-- PortfolioPlanner with AHPPA_V2=1 --")
P.WATCHDOG.reset()
with contextlib.redirect_stdout(io.StringIO()):
    p = pl.plan(-3.0, -3.0, 4.0, -3.0, start_layer=1, goal_layer=1)
check("normal plan uses mode V2", pl.last_mode == P.V2_MODE, str(pl.last_mode))
check("V2 path is the smoothed A* path", p == sm, f"{len(p)} waypoints")
check("V2 path reaches the goal", math.dist(p[-1][:2], (4.0, -3.0)) < 0.3)
P.WATCHDOG.tripped = True
with contextlib.redirect_stdout(io.StringIO()):
    q = pl.plan(-3.0, -3.0, 4.0, -3.0, start_layer=1, goal_layer=1)
check("tripped watchdog -> ESCAPE", pl.last_mode == ESCAPE, str(pl.last_mode))
check("ESCAPE path valid and smoothed", bool(q) and R.segments_clear(lt, G, q) and len(q) < len(raw) / 5)
P.WATCHDOG.tripped = False
orig = pl._ca.plan
pl._ca.plan = lambda *a, **k: None
P.WATCHDOG.tripped = True
with contextlib.redirect_stdout(io.StringIO()):
    q = pl.plan(-3.0, -3.0, 4.0, -3.0, start_layer=1, goal_layer=1)
check("clearance A* failing in ESCAPE -> smoothed A* instead", q == sm)
P.WATCHDOG.tripped = False
pl._ca.plan = orig
P.V2_REFINE = 'sync'
with contextlib.redirect_stdout(io.StringIO()):
    q = pl.plan(-3.0, -3.0, 4.0, -3.0, start_layer=1, goal_layer=1)
check("V2_REFINE=sync returns the refined path", R.path_length(q) <= R.path_length(sm) + 1e-9
      and R.segments_clear(lt, G, q))
P.V2_REFINE = 'off'
P.V2 = False
with contextlib.redirect_stdout(io.StringIO()):
    pl.plan(-3.0, -3.0, 4.0, -3.0, start_layer=1, goal_layer=1)
check("AHPPA_V2 off -> shipped dispatcher again", pl.last_mode != P.V2_MODE, str(pl.last_mode))

print(f"\n{'ALL PASS' if not FAILS else 'FAILED: ' + ', '.join(FAILS)}")
sys.exit(1 if FAILS else 0)
