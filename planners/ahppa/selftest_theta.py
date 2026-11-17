#!/usr/bin/env python3
"""Offline correctness tests for lazy_theta.py. No ROS, no Gazebo."""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from lazy_theta import LazyThetaStar, GridAStar

RES = 0.25
Z = [1.2, 1.8, 2.4]
H = W = 40
OX = OY = 0.0
P = dict(resolution=RES, origin_x=OX, origin_y=OY)

ok = fail = 0


def check(name, cond, detail=""):
    global ok, fail
    if cond:
        ok += 1
        print(f"  PASS  {name}")
    else:
        fail += 1
        print(f"  FAIL  {name}  {detail}")


def empty():
    return np.zeros((len(Z), H, W), dtype=bool)


def wmid(r, c, li=1):
    return (OX + (c + .5) * RES, OY + (r + .5) * RES, Z[li])


def plen(p):
    return sum(math.dist(x, y) for x, y in zip(p, p[1:]))


def collision_free(t, g, p):
    for x, y in zip(p, p[1:]):
        if not t.line_of_sight(g, t.to_cell(*x), t.to_cell(*y)):
            return False
        if not t._free(g, t.to_cell(*y)):
            return False
    return True


t = LazyThetaStar(Z, **P)
a = GridAStar(Z, **P)

g = empty()
s, e = wmid(5, 5), wmid(30, 34)
p = t.plan(g, s, e)
check("open space -> 2 waypoints", p is not None and len(p) == 2)
check("open space -> straight line length",
      p is not None and abs(plen(p) - math.dist(s, e)) < 1e-6)

g = empty(); g[:, :, 20] = True; g[:, 18:22, 20] = False
p = t.plan(g, wmid(20, 5), wmid(20, 35))
check("wall with gap -> path found", p is not None)
check("wall with gap -> collision free", p is not None and collision_free(t, g, p))

g = empty(); g[:, :, 20] = True
check("sealed wall -> None", t.plan(g, wmid(20, 5), wmid(20, 35)) is None)

g = empty()
p = t.plan(g, wmid(10, 10), wmid(10, 10))
check("start == goal", p is not None and len(p) == 1)

g = empty(); g[:, 9:12, 9:12] = True
p = t.plan(g, wmid(10, 10), wmid(30, 30))
check("start inside obstacle -> nudged", p is not None and collision_free(t, g, p))

g = empty(); g[1] = True; g[1, 18:22, 18:22] = False
p = t.plan(g, wmid(20, 20, 0), wmid(20, 20, 2))
check("climbs through a shaft", p is not None and collision_free(t, g, p))

g = empty(); g[:, 15:25, 15:25] = True
pairs = [((1, 2, 2), (1, 35, 35)), ((1, 5, 30), (1, 30, 5)), ((0, 10, 10), (2, 30, 30))]
check("LOS symmetric",
      all(t.line_of_sight(g, x, y) == t.line_of_sight(g, y, x) for x, y in pairs))


def random_grids(seed):
    rng = np.random.default_rng(seed)
    while True:
        g = empty()
        g[:, 0, :] = g[:, -1, :] = g[:, :, 0] = g[:, :, -1] = True
        for _ in range(rng.integers(5, 60)):
            r, c = rng.integers(2, H - 5), rng.integers(2, W - 5)
            g[:, r:r + 3, c:c + 3] = True
        free = np.argwhere(~g[1])
        if len(free) < 50:
            yield None
            continue
        s_ = free[rng.integers(len(free))]
        e_ = free[rng.integers(len(free))]
        ws, we = wmid(s_[0], s_[1]), wmid(e_[0], e_[1])
        yield None if math.dist(ws, we) < 4.0 else (g, ws, we)


bad_coll = bad_len = bad_wp = tried = 0
worst = 0.0
gen = random_grids(11)
for _ in range(120):
    q = next(gen)
    if q is None:
        continue
    g, ws, we = q
    pt, pa = t.plan(g, ws, we), a.plan(g, ws, we)
    if pt is None or pa is None:
        continue
    tried += 1
    if not collision_free(t, g, pt):
        bad_coll += 1
    ratio = plen(pt) / max(plen(pa), 1e-9)
    worst = max(worst, ratio)
    if ratio > 1.02:
        bad_len += 1
    if len(pt) > len(pa):
        bad_wp += 1
check(f"random grids: all Theta* paths collision free ({tried} queries)",
      bad_coll == 0, f"{bad_coll} clipped")
check("random grids: never >2% longer than A*", bad_len == 0,
      f"{bad_len} longer, worst {worst:.4f}")
check("random grids: never more waypoints than A*", bad_wp == 0, f"{bad_wp} worse")

bad_a = tried_a = 0
gen = random_grids(11)
for _ in range(120):
    q = next(gen)
    if q is None:
        continue
    g, ws, we = q
    pa = a.plan(g, ws, we)
    if pa is None:
        continue
    tried_a += 1
    if not collision_free(t, g, pa):
        bad_a += 1
check(f"random grids: A* paths collision free too ({tried_a} queries)",
      bad_a == 0, f"{bad_a} clipped")

t2 = LazyThetaStar(Z, max_expansions=5, **P)
g = empty(); g[:, :, 20] = True; g[:, 18:22, 20] = False
check("expansion cap honoured", t2.plan(g, wmid(20, 2), wmid(20, 37)) is None)

g = empty()
t.plan(g, wmid(5, 5), wmid(30, 30))
n1 = t.expansions
t.plan(g, wmid(5, 5), wmid(30, 30))
check("counters reset between calls", t.expansions == n1)

print(f"\n  {ok} passed, {fail} failed\n")
sys.exit(1 if fail else 0)
