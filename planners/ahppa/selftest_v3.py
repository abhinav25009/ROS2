#!/usr/bin/env python3
"""Offline test of AHPPA v3 stuck memory (PortfolioPlanner._plan_v3).

  source /opt/ros/humble/setup.bash
  python3 planners/ahppa/selftest_v3.py
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
os.environ['AHPPA_V3'] = '1'
os.environ['AHPPA_LOG_DIR'] = tempfile.mkdtemp(prefix='ahppa_v3_test_')

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import portfolio as P                                    # noqa: E402
import refine as R                                       # noqa: E402
from dispatcher import ESCAPE                            # noqa: E402
from core.octomap_3d_query import OctoMap3DQuery         # noqa: E402

FAILS = []


def check(name, cond, detail=""):
    print(f"  [{'PASS' if cond else 'FAIL'}] {name}{'  ' + detail if detail else ''}")
    if not cond:
        FAILS.append(name)


class Clock:
    t = 10.0

    def __call__(self):
        return self.t


P.set_clock(Clock())
res, W, H = 0.1, 120, 100


def planner(walls):
    grid = SimpleNamespace(info=SimpleNamespace(resolution=res, width=W, height=H,
                           origin=SimpleNamespace(position=SimpleNamespace(x=-6.0, y=-5.0))),
                           data=[0] * (W * H))
    pts = [[x, y, z] for (x, y) in walls for z in np.arange(0.0, 2.8, res)]
    # ceiling at 2.8 m, as indoors: without it the planner flies over the walls
    pts += [[x, y, 2.85] for x in np.arange(-6.0, 6.0, res) for y in np.arange(-5.0, 5.0, res)]
    octo = OctoMap3DQuery(resolution=res)
    octo.voxel_centers = np.array(pts or [[50.0, 50.0, 50.0]], dtype=np.float32)
    octo.occupied_set = set(octo._world_to_key(*p) for p in octo.voxel_centers)
    with contextlib.redirect_stdout(io.StringIO()):
        return P.PortfolioPlanner(grid, octo, safety_margin=0.4)


def plan(pl, *a):
    with contextlib.redirect_stdout(io.StringIO()):
        return pl.plan(*a, start_layer=1, goal_layer=1)


def min_dist(path, x, y):
    """Smallest distance from (x, y) to the path's segments."""
    best = math.inf
    for a, b in zip(path, path[1:]):
        ax, ay, bx, by = a[0], a[1], b[0], b[1]
        L2 = (bx - ax) ** 2 + (by - ay) ** 2
        t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((x - ax) * (bx - ax) + (y - ay) * (by - ay)) / L2))
        best = min(best, math.hypot(ax + t * (bx - ax) - x, ay + t * (by - ay) - y))
    return best


print("\n-- report_stuck() bookkeeping --")
P.clear_memory()
P.V3 = False
check("no-op with AHPPA_V3 off", not P.report_stuck(0, 0, 1, 0) and not P.MEMORY)
P.V3 = True
check("zero heading is ignored", not P.report_stuck(0, 0, 0, 0) and not P.MEMORY)
with contextlib.redirect_stdout(io.StringIO()):
    ok = P.report_stuck(1.0, 2.0, 3.0, 4.0)
m = P.MEMORY[-1] if P.MEMORY else {}
check("barrier stored BAR_D ahead, unit heading", ok and abs(m['cx'] - 1.18) < 1e-6 and abs(m['cy'] - 2.24) < 1e-6
      and abs(m['hx'] - 0.6) < 1e-9, str(m))
with contextlib.redirect_stdout(io.StringIO()):
    for i in range(P.MEMORY_MAX + 5):
        P.report_stuck(i, 0, 1, 0)
check("memory is capped, oldest dropped", len(P.MEMORY) == P.MEMORY_MAX and P.MEMORY[0]['cx'] > 5)
P.clear_memory()
check("clear_memory()", not P.MEMORY and not P._escape_request)

print("\n-- open room, unseen glass ahead --")
pl = planner([])
p0 = plan(pl, -3.0, -3.0, 4.0, -3.0)
check("without memory: straight line through the glass", bool(p0) and min_dist(p0, 0.0, -3.0) < 0.1,
      f"{len(p0)} waypoints")
check("normal plan is V2 mode", pl.last_mode == P.V2_MODE, str(pl.last_mode))
with contextlib.redirect_stdout(io.StringIO()):
    P.report_stuck(-0.4, -3.0, 1.0, 0.0)              # pinned at x=-0.4 facing +x
pl = planner([])                                       # the navigator builds a new planner per plan
p1 = plan(pl, -0.4, -3.0, 4.0, -3.0)
check("after a stuck report: planned in ESCAPE", pl.last_mode == ESCAPE, str(pl.last_mode))
check("new path keeps clear of the barrier (> margin)", bool(p1) and min_dist(p1, -0.1, -3.0) > 0.4,
      f"min dist {min_dist(p1, -0.1, -3.0):.2f} m")
check("new path still reaches the goal", bool(p1) and math.dist(p1[-1][:2], (4.0, -3.0)) < 0.3)
check("memory used as bars", pl.last_memory == 'bar', str(pl.last_memory))
p2 = plan(pl, -0.4, -3.0, 4.0, -3.0)
check("escape request is consumed by one plan", pl.last_mode == P.V2_MODE, str(pl.last_mode))
check("memory persists for later plans", bool(p2) and min_dist(p2, -0.1, -3.0) > 0.4)
check("planner grids restored after planning", not pl._grids[1, int((-3.0 + 5) / res), int((-0.1 + 6) / res)])

print("\n-- unseen post in the middle of the only doorway --")
P.clear_memory()
wall = [(0.0, y) for y in np.arange(-5.0, -1.2, res)] + [(0.0, y) for y in np.arange(1.25, 5.0, res)]
pl = planner(wall)                                     # door y in (-1.2, 1.2)
with contextlib.redirect_stdout(io.StringIO()):
    P.report_stuck(-0.45, 0.0, 1.0, 0.0)
q = plan(pl, -0.45, 0.0, 3.0, 0.0)
check("bar would seal the door -> relaxed to the contact point", pl.last_memory == 'dot', str(pl.last_memory))
check("path through the door exists", bool(q) and math.dist(q[-1][:2], (3.0, 0.0)) < 0.3)
check("path avoids the contact point", bool(q) and min_dist(q, -0.15, 0.0) > 0.3,
      f"{min_dist(q, -0.15, 0.0):.2f} m" if q else '')
check("path is valid on the sensed map", bool(q) and R.segments_clear(pl._lt, pl._grids, q))
P.clear_memory()

print("\n-- v3.1 surface fusion (contacts from PX4 glass flight B20-v3-10) --")
P.clear_memory()


def hit(x, y, hx, hy):
    with contextlib.redirect_stdout(io.StringIO()):
        P.report_stuck(x, y, hx, hy, source='impact')


hit(3.55, 1.40, 0.94, 0.35)
hit(3.61, -1.26, 0.84, -0.54)
P.FUSE = False
check("fusion off: one bar per contact", len(P.memory_segments(kind='bar')) == 2)
P.FUSE = True
sg = P.memory_segments(kind='bar')
x0, y0, x1, y1 = sg[0] if sg else (0, 0, 0, 0)
ang = math.degrees(math.atan2(abs(x1 - x0), abs(y1 - y0)))
check("two angled hits on one pane -> ONE segment along the pane", len(sg) == 1 and ang < 5
      and abs((x0 + x1) / 2 - 3.85) < 0.15, f"{len(sg)} segment(s), {ang:.1f} deg off the pane")
check("bar = contact span + BAR_W/2 each end", len(sg) == 1 and abs(abs(y1 - y0) - (2.95 + P.BAR_W)) < 0.3,
      f"{abs(y1 - y0):.2f} m")
hl = P.memory_segments(kind='hull')
check("hull = contact span + HULL_EXT each end", len(hl) == 1
      and abs(math.dist(hl[0][:2], hl[0][2:]) - (2.95 + 2 * P.HULL_EXT)) < 0.3)
P.clear_memory()
hit(3.0, 0.0, 1.0, 0.0)
hit(6.0, 3.0, 0.0, 1.0)
check("hits on an L corner (headings 90 deg apart) are not fused", len(P.memory_segments(kind='bar')) == 2)
P.clear_memory()

print("\n-- v3.1: memory seals the gap -> relaxed to the measured hull, not to dots --")
# room y in (-2.0, 4.9); unseen pane x = 0 from the bottom wall to y = 2.5,
# gap y in (2.5, 4.9) - the glass_test.sdf layout
top = [(x, yy) for x in np.arange(-6.0, 6.0, res) for yy in (-2.0, 4.9)]
pl = planner(top)
hit(-0.45, -1.0, 0.93, 0.36)
hit(-0.40, 1.2, 0.85, -0.53)
hit(-0.42, 2.0, 1.0, -0.1)
q = plan(pl, -2.0, 0.0, 3.0, 0.0)
check("3 hits along the pane: bars seal the gap -> 'hull'", pl.last_memory == 'hull', str(pl.last_memory))
cross = [((a[1] + b[1]) / 2) for a, b in zip(q, q[1:]) if (a[0] < 0) != (b[0] < 0)] if q else []
check("path crosses the pane line only through the gap", bool(q) and cross and all(c > 2.9 for c in cross),
      f"crosses at y={[round(c, 2) for c in cross]}")
P.FUSE = False
pl = planner(top)
q = plan(pl, -2.0, 0.0, 3.0, 0.0)
cross0 = [((a[1] + b[1]) / 2) for a, b in zip(q, q[1:]) if (a[0] < 0) != (b[0] < 0)] if q else []
check("(v3 as flown, for contrast: falls back to dots, path crosses the glass)", pl.last_memory == 'dot'
      and any(c < 2.5 for c in cross0), f"{pl.last_memory}, crosses at y={[round(c, 2) for c in cross0]}")
P.FUSE = True
P.clear_memory()

print("\n-- v2 unchanged when no memory --")
pl = planner(wall)
q = plan(pl, -3.0, 0.0, 3.0, 0.0)
check("no memory: same as v2", pl.last_mode == P.V2_MODE and pl.last_memory == 'none' and bool(q))

print(f"\n{'ALL PASS' if not FAILS else 'FAILED: ' + ', '.join(FAILS)}")
sys.exit(1 if FAILS else 0)
