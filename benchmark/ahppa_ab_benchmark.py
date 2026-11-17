#!/usr/bin/env python3
"""A/B benchmark: stock layered A* (before AHPPA) vs AHPPA portfolio (after).

Both planners get the SAME ground-truth map and the SAME start/goal pairs, so
any difference comes from the planner alone - no sensor noise, no flight
controller, no map-building order.

MAP
    Every <box> collision in worlds/house_2floor.sdf is voxelised at 0.10 m
    (the OctoMap resolution used in flight). Ground plane and floor are left
    out, as octomap's occupancy_min_z=0.15 leaves them out in flight. The
    projected OccupancyGrid is fully known (no unknown cells), so this
    measures planning, not exploration.

PLANNER SETTINGS (as in navigator_astar.goal_cb)
    safety_margin 0.40 m, unknown_penalty 50.0, start layer from start z,
    goal layer from goal z.

METRICS
    ok          planner returned a path
    ms          median of 3 runs, wall time, includes planner construction
                is EXCLUDED (both share MultiLayerPlanner.__init__)
    length      3D polyline length (m)
    waypoints   number of returned points
    turning     sum of heading changes (deg)
    min_clear   smallest distance from the path (sampled every 0.05 m) to
                any occupied voxel centre (m)
    mode        AHPPA mode chosen (after only)

  source /opt/ros/humble/setup.bash
  python3 benchmark/ahppa_ab_benchmark.py            # writes benchmark/ahppa_ab_results.csv
"""
import contextlib
import csv
import io
import math
import os
import statistics
import sys
import time
import xml.etree.ElementTree as ET
from types import SimpleNamespace

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(REPO, 'planners', 'ahppa'))
os.environ['AHPPA_PORTFOLIO'] = '1'
os.environ.setdefault('AHPPA_LOG_DIR', '/tmp/ahppa_ab_logs')

from core.octomap_3d_query import OctoMap3DQuery            # noqa: E402
from planners.planner_3d import (MultiLayerPlanner, Z_LAYERS,  # noqa: E402
                                 layer_for_z)
import portfolio as P                                         # noqa: E402
from dispatcher import HysteresisDispatcher                   # noqa: E402

RES = 0.10
SAFETY_MARGIN = 0.40
UNKNOWN_PENALTY = 50.0
X0, X1, Y0, Y1 = -8.0, 15.0, -7.0, 7.0
SKIP = {'ground_plane', 'gf_floor', 'garden_patio', 'living_rug'}

# name, category, start (x,y,z), goal (x,y,z)
QUERIES = [
    ("Q01 hall: open crossing",          "open",       (0.0, 0.0, 1.8),   (5.0, -3.0, 1.8)),
    ("Q02 hall: diagonal past chairs",   "open",       (6.0, 3.5, 1.8),   (1.5, -3.5, 1.8)),
    ("Q03 hall -> living (door A)",      "doorway",    (2.0, -2.5, 1.8),  (-4.5, -1.5, 1.8)),
    ("Q04 hall -> dining (door B)",      "doorway",    (2.0, 2.5, 1.8),   (-3.5, 1.8, 1.8)),
    ("Q05 living -> kitchen (via hall)", "multi-room", (-4.5, -1.5, 1.8), (-4.0, 2.0, 1.8)),
    ("Q06 dining -> kitchen clutter",    "clutter",    (-2.2, 3.6, 1.8),  (-5.6, 1.6, 1.8)),
    ("Q07 hall -> garden (east door)",   "doorway",    (3.0, 0.0, 1.8),   (10.5, -2.5, 1.8)),
    ("Q08 garden: around trees",         "open",       (9.0, 3.5, 1.8),   (12.5, -2.0, 1.8)),
    ("Q09 start near wall -> living",    "near-wall",  (5.5, -4.5, 1.8),  (-2.7, -3.8, 1.8)),
    ("Q10 living: corner to corner",     "clutter",    (-6.2, -4.4, 1.8), (-2.6, -4.5, 1.8)),
    ("Q11 landing -> bedroom 1",         "doorway",    (3.0, -3.6, 3.6),  (-4.4, -3.6, 3.6)),
    ("Q12 bedroom 2 -> landing",         "doorway",    (-4.4, 3.0, 3.6),  (5.5, 3.5, 3.6)),
    ("Q13 bedroom 1 -> bedroom 2",       "multi-room", (-4.4, -2.0, 3.6), (-4.4, 2.0, 3.6)),
    ("Q14 hall -> landing (atrium)",     "two-floor",  (3.0, -1.0, 1.8),  (3.0, 3.6, 3.6)),
    ("Q15 living -> bedroom 2",          "two-floor",  (-4.5, -1.5, 1.8), (-4.4, 2.5, 3.6)),
    ("Q16 landing -> garden",            "two-floor",  (6.0, -4.0, 3.6),  (10.0, -3.0, 1.8)),
    ("Q17 bedroom 1 -> kitchen",         "two-floor",  (-4.4, -3.0, 3.6), (-3.5, 1.8, 1.8)),
    ("Q18 hall -> bedroom 2 desk",       "two-floor",  (0.5, -4.0, 1.8),  (-3.5, 3.3, 3.6)),
]


# ── ground-truth map ────────────────────────────────────────────────────────
def _pose(el):
    v = [float(t) for t in (el.findtext('pose') or '0 0 0 0 0 0').split()]
    return (v + [0.0] * 6)[:6]


def boxes_from_sdf(path):
    """World-frame AABBs of every box collision (yaw-only rotation, which is
    all this world uses)."""
    out = []
    for m in ET.parse(path).getroot().iter('model'):
        if m.get('name') in SKIP:
            continue
        mp = _pose(m)
        for link in m.findall('link'):
            lp = _pose(link)
            for col in link.findall('collision'):
                size = col.find('geometry/box/size')
                if size is None:
                    continue
                sx, sy, sz = (abs(float(t)) for t in size.text.split())
                cp = _pose(col)
                # compose model -> link -> collision, yaw only
                x, y, z, yaw = 0.0, 0.0, 0.0, 0.0
                for p in (mp, lp, cp):
                    c, s = math.cos(yaw), math.sin(yaw)
                    x, y, z = x + c * p[0] - s * p[1], y + s * p[0] + c * p[1], z + p[2]
                    yaw += p[5]
                c, s = abs(math.cos(yaw)), abs(math.sin(yaw))
                hx, hy = (c * sx + s * sy) / 2, (s * sx + c * sy) / 2
                out.append((m.get('name'), x - hx, x + hx, y - hy, y + hy,
                            z - sz / 2, z + sz / 2))
    return out


def voxelise(boxes):
    keys = set()
    for _, x0, x1, y0, y1, z0, z1 in boxes:
        z0 = max(z0, 0.15)
        if z1 <= z0:
            continue
        ix = np.arange(math.floor(x0 / RES), math.ceil(x1 / RES))
        iy = np.arange(math.floor(y0 / RES), math.ceil(y1 / RES))
        iz = np.arange(math.floor(z0 / RES), math.ceil(z1 / RES))
        if not (len(ix) and len(iy) and len(iz)):
            continue
        g = np.stack(np.meshgrid(ix, iy, iz, indexing='ij'), -1).reshape(-1, 3)
        keys.update(map(tuple, g.tolist()))
    k = np.array(sorted(keys), dtype=np.int64)
    return (k.astype(np.float32) + 0.5) * RES


def make_inputs(centers):
    W, H = int(round((X1 - X0) / RES)), int(round((Y1 - Y0) / RES))
    data = np.zeros((H, W), dtype=np.int8)
    c = np.floor((centers[:, 0] - X0) / RES).astype(int)
    r = np.floor((centers[:, 1] - Y0) / RES).astype(int)
    ok = (r >= 0) & (r < H) & (c >= 0) & (c < W)
    data[r[ok], c[ok]] = 100
    grid = SimpleNamespace(
        info=SimpleNamespace(resolution=RES, width=W, height=H,
                             origin=SimpleNamespace(position=SimpleNamespace(x=X0, y=Y0))),
        data=data.reshape(-1).tolist())
    octo = OctoMap3DQuery(resolution=RES)
    octo.voxel_centers = centers
    octo.occupied_set = set(octo._world_to_key(*p) for p in centers)
    return grid, octo


# ── metrics ─────────────────────────────────────────────────────────────────
def polyline_len(p):
    return sum(math.dist(a, b) for a, b in zip(p, p[1:]))


def turning(p):
    tot = 0.0
    for a, b, c in zip(p, p[1:], p[2:]):
        u = np.subtract(b, a)[:2]
        v = np.subtract(c, b)[:2]
        nu, nv = np.linalg.norm(u), np.linalg.norm(v)
        if nu > 1e-9 and nv > 1e-9:
            tot += math.degrees(math.acos(float(np.clip(u @ v / (nu * nv), -1, 1))))
    return tot


def min_clearance(p, centers):
    pts = []
    for a, b in zip(p, p[1:]):
        n = max(1, int(math.dist(a, b) / 0.05))
        for t in np.linspace(0, 1, n, endpoint=False):
            pts.append(np.add(a, np.multiply(t, np.subtract(b, a))))
    pts.append(np.asarray(p[-1], float))
    best = np.inf
    for q in pts:
        m = ((np.abs(centers[:, 0] - q[0]) < 2.0) & (np.abs(centers[:, 1] - q[1]) < 2.0)
             & (np.abs(centers[:, 2] - q[2]) < 2.0))
        if m.any():
            best = min(best, float(np.sqrt(((centers[m] - q) ** 2).sum(1)).min()))
    return best if np.isfinite(best) else 2.0


def run(fn):
    times, path = [], None
    for _ in range(3):
        t0 = time.perf_counter()
        with contextlib.redirect_stdout(io.StringIO()):
            path = fn()
        times.append((time.perf_counter() - t0) * 1000)
    return path, statistics.median(times)


def main():
    sdf = os.path.join(REPO, 'worlds', 'house_2floor.sdf')
    boxes = boxes_from_sdf(sdf)
    centers = voxelise(boxes)
    grid, octo = make_inputs(centers)
    print(f"map: {len(boxes)} boxes -> {len(centers)} voxels, "
          f"grid {grid.info.width}x{grid.info.height} @ {RES} m")

    with contextlib.redirect_stdout(io.StringIO()):
        stock = MultiLayerPlanner(grid, octo, safety_margin=SAFETY_MARGIN,
                                  unknown_penalty=UNKNOWN_PENALTY)
        port = P.PortfolioPlanner(grid, octo, safety_margin=SAFETY_MARGIN,
                                  unknown_penalty=UNKNOWN_PENALTY)
    P.set_clock(lambda: 0.0)

    rows = []
    for name, cat, s, g in QUERIES:
        sl, gl = layer_for_z(s[2]), layer_for_z(g[2])

        p0, ms0 = run(lambda: MultiLayerPlanner.plan(stock, s[0], s[1], g[0], g[1],
                                                     start_layer=sl, goal_layer=gl))

        def ahppa():
            P.DISPATCH = HysteresisDispatcher()       # each query = fresh vehicle
            P.WATCHDOG.reset()
            return port.plan(s[0], s[1], g[0], g[1], start_layer=sl, goal_layer=gl)
        p1, ms1 = run(ahppa)
        mode = port.last_mode
        clear0 = P.FEATURE.last_clearance

        for label, p, ms, m in (("before", p0, ms0, "A*"), ("after", p1, ms1, mode)):
            ok = bool(p)
            reached = ok and math.dist(p[-1][:2], g[:2]) < 0.6 and \
                abs(p[-1][2] - Z_LAYERS[gl]) < 0.05
            rows.append(dict(
                query=name, category=cat, planner=label, mode=m,
                start_clearance=round(clear0, 3),
                ok=int(ok), reached_goal=int(reached), ms=round(ms, 1),
                length=round(polyline_len(p), 2) if ok else '',
                waypoints=len(p) if ok else 0,
                turning=round(turning(p), 0) if ok else '',
                min_clear=round(min_clearance(p, centers), 3) if ok else '',
                floors=len({round(q[2], 1) for q in p}) if ok else ''))
        r0, r1 = rows[-2], rows[-1]
        print(f"{name:34s} before ok={r0['ok']} {r0['ms']:7.1f} ms L={r0['length']!s:>6} "
              f"clr={r0['min_clear']!s:>5} | after [{mode:6s}] ok={r1['ok']} "
              f"{r1['ms']:7.1f} ms L={r1['length']!s:>6} clr={r1['min_clear']!s:>5}")

    out = os.path.join(REPO, 'benchmark', 'ahppa_ab_results.csv')
    with open(out, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    print(f"\n-> {out}")


if __name__ == '__main__':
    main()
