#!/usr/bin/env python3
"""Multi-world A/B benchmark: stock layered A* vs AHPPA, on many layouts.

One house is one data point. This runs the same comparison as
ahppa_ab_benchmark.py over every world in the third-party collection
(github.com/mertgulerx/world-collection: mazes, offices, aisles, forests,
urban, narrow passages, clutter fields...) plus this repo's own worlds, with
randomly sampled but reproducible start/goal pairs.

PLANNERS (all share one MultiLayerPlanner construction per world)
    stock     MultiLayerPlanner.plan - the "before"
    ahppa     PortfolioPlanner.plan, clearance band +/-1.0 m - the version
              deployed on 2026-11-17 (explicit, so old results reproduce)
    ahppa_h   same, clearance band +/-0.3 m. This is the DEFAULT since the
              ceiling fix (clearance_feature.BAND_Z_DEFAULT), i.e. what ships

WORLD PREPARATION
    The collection is built for ground robots: walls are 2.0 m tall, so a
    drone at 2.4 m+ would simply fly over everything. Indoor worlds are turned
    into a building - every obstacle 1.9 m or taller is raised to 2.8 m and a
    2.8-3.0 m ceiling covers the footprint, as in house_2floor. Low clutter
    (e.g. 1.2 m tables) is left as is. Outdoor families (F4 forest, F5 urban)
    keep their real heights: flying over a 3 m tree is legitimate. Only box
    and cylinder collisions exist in these worlds (checked); obstacles are
    voxelised as SURFACE shells at 0.10 m, which is what OctoMap would hold.
    RP3 (290 m x 220 m cathedral) is skipped: too large at 0.10 m.

QUERIES
    Per world, N pairs of free cells on the stock planner's 1.8 m layer, in the
    same connected component, 4-25 m apart (seeded by world name). Every
    planner gets the same pairs. A call over TIMEOUT_S counts as a failure.

  source /opt/ros/humble/setup.bash
  python3 benchmark/ahppa_multiworld.py [--n 15] [--only SC6,F2] [--out file.csv]
"""
import argparse
import contextlib
import csv
import glob
import hashlib
import io
import math
import os
import signal
import sys
import time
import xml.etree.ElementTree as ET
from collections import deque
from types import SimpleNamespace

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(REPO, 'planners', 'ahppa'))
os.environ['AHPPA_PORTFOLIO'] = '1'
os.environ.setdefault('AHPPA_LOG_DIR', '/tmp/ahppa_mw_logs')

from core.octomap_3d_query import OctoMap3DQuery            # noqa: E402
from planners.planner_3d import MultiLayerPlanner, Z_LAYERS   # noqa: E402
import portfolio as P                                         # noqa: E402
from dispatcher import HysteresisDispatcher                   # noqa: E402
from clearance_feature import ClearanceEstimator              # noqa: E402

RES = 0.10
SAFETY_MARGIN = 0.40
UNKNOWN_PENALTY = 50.0
FLY_LAYER = 1                       # 1.8 m
Z_CAP = 6.0                         # nothing above the top layer band matters
TIMEOUT_S = 30
COLLECTION = os.path.expanduser('~/world-collection/worlds')
OUTDOOR = ('F4_', 'F5_')
SKIP = ('RP3_',)


# ── world loading ───────────────────────────────────────────────────────────
def _pose(el):
    v = [float(t) for t in (el.findtext('pose') or '0 0 0 0 0 0').split()]
    return (v + [0.0] * 6)[:6]


def _rot(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p), math.sin(p),
                              math.cos(y), math.sin(y))
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def load_geometry(path):
    """(upright AABBs, surface points of TILTED shapes).

    SDF poses compose model -> link -> collision with full roll/pitch/yaw.
    Shapes with no roll or pitch become world-frame AABBs (exact for yaw too,
    via the rotated extent). Tilted ones - e.g. house_2floor's roof, pitched
    12.6 deg - cannot be an AABB without turning a thin slope into a solid
    block, so their surfaces are sampled at RES/2 and voxelised directly.
    Before 2026-11-17 this loader was yaw-only and treated that roof as flat."""
    out, tilted = [], []
    for m in ET.parse(path).getroot().iter('model'):
        mp = _pose(m)
        for link in m.findall('link') or [m]:
            lp = _pose(link) if link is not m else [0.0] * 6
            for col in link.findall('collision'):
                g = col.find('geometry')
                if g is None:
                    continue
                if g.find('box') is not None:
                    sx, sy, sz = (abs(float(t)) for t in g.find('box/size').text.split())
                elif g.find('cylinder') is not None:
                    r = float(g.findtext('cylinder/radius'))
                    sx = sy = 2 * r
                    sz = float(g.findtext('cylinder/length'))
                else:
                    continue
                cp = _pose(col)
                R, t = np.eye(3), np.zeros(3)
                for p in (mp, lp, cp):
                    t = t + R @ np.array(p[:3])
                    R = R @ _rot(p[3:6])
                h = np.array([sx, sy, sz]) / 2
                if abs(R[2, 2]) > 1 - 1e-6:            # upright: AABB is exact
                    e = np.abs(R) @ h
                    out.append([t[0] - e[0], t[0] + e[0], t[1] - e[1], t[1] + e[1],
                                t[2] - e[2], t[2] + e[2]])
                else:
                    tilted.append(_box_surface(t, R, h))
    pts = np.concatenate(tilted) if tilted else np.empty((0, 3))
    return np.array(out, dtype=float), pts


def _box_surface(t, R, h):
    step = RES / 2
    g = [np.linspace(-hi, hi, max(2, int(math.ceil(2 * hi / step)) + 1)) for hi in h]
    faces = []
    for ax in range(3):
        o = [i for i in range(3) if i != ax]
        a, b = np.meshgrid(g[o[0]], g[o[1]], indexing='ij')
        for sgn in (-1, 1):
            f = np.zeros((a.size, 3))
            f[:, o[0]], f[:, o[1]], f[:, ax] = a.ravel(), b.ravel(), sgn * h[ax]
            faces.append(f)
    return np.concatenate(faces) @ R.T + t


def load_boxes(path):
    """Upright shapes only, as world-frame AABBs (kept for old callers)."""
    return load_geometry(path)[0]


def prepare(boxes, indoor):
    """Drop floors; for indoor worlds raise walls to 2.8 m and add a ceiling."""
    b = boxes[boxes[:, 5] >= 0.15].copy()           # octomap occupancy_min_z
    b[:, 4] = np.maximum(b[:, 4], 0.15)
    xs = (b[:, 0].min(), b[:, 1].max())
    ys = (b[:, 2].min(), b[:, 3].max())
    if indoor:
        # AHPPA_BENCH_CEILING moves the ceiling for the sensitivity test
        # (default 2.8 m, as in house_2floor).
        ceil = float(os.environ.get('AHPPA_BENCH_CEILING', '2.8'))
        tall = b[:, 5] >= 1.9
        b[tall, 5] = np.maximum(b[tall, 5], ceil)
        b = np.vstack([b, [[xs[0], xs[1], ys[0], ys[1], ceil, ceil + 0.2]]])
    b[:, 5] = np.minimum(b[:, 5], Z_CAP)
    return b[b[:, 5] > b[:, 4]], xs, ys


def shell_voxels(boxes, extra_points=None):
    """Occupied voxel centres on the surface of each box (0.10 m), plus the
    voxels hit by extra_points (surfaces of tilted shapes)."""
    keys = []
    if extra_points is not None and len(extra_points):
        ep = extra_points[(extra_points[:, 2] >= 0.15) & (extra_points[:, 2] <= Z_CAP)]
        keys.append(np.floor(ep / RES).astype(np.int64))
    for x0, x1, y0, y1, z0, z1 in boxes:
        ix = np.arange(math.floor(x0 / RES), max(math.ceil(x1 / RES), math.floor(x0 / RES) + 1))
        iy = np.arange(math.floor(y0 / RES), max(math.ceil(y1 / RES), math.floor(y0 / RES) + 1))
        iz = np.arange(math.floor(z0 / RES), max(math.ceil(z1 / RES), math.floor(z0 / RES) + 1))
        for a, b_, c in ((ix[[0, -1]], iy, iz), (ix, iy[[0, -1]], iz), (ix, iy, iz[[0, -1]])):
            g = np.stack(np.meshgrid(a, b_, c, indexing='ij'), -1).reshape(-1, 3)
            keys.append(g)
    k = np.unique(np.concatenate(keys), axis=0)
    return (k.astype(np.float32) + 0.5) * RES


def make_inputs(centers, xs, ys):
    X0, X1 = math.floor(xs[0]) - 1.0, math.ceil(xs[1]) + 1.0
    Y0, Y1 = math.floor(ys[0]) - 1.0, math.ceil(ys[1]) + 1.0
    W, H = int(round((X1 - X0) / RES)), int(round((Y1 - Y0) / RES))
    data = np.zeros((H, W), dtype=np.int8)
    c = np.floor((centers[:, 0] - X0) / RES).astype(int)
    r = np.floor((centers[:, 1] - Y0) / RES).astype(int)
    ok = (r >= 0) & (r < H) & (c >= 0) & (c < W) & (centers[:, 2] < 2.75)
    data[r[ok], c[ok]] = 100
    grid = SimpleNamespace(
        info=SimpleNamespace(resolution=RES, width=W, height=H,
                             origin=SimpleNamespace(position=SimpleNamespace(x=X0, y=Y0))),
        data=data.reshape(-1).tolist())
    octo = OctoMap3DQuery(resolution=RES)
    octo.voxel_centers = centers
    octo.occupied_set = set(map(tuple, np.floor(centers / RES).astype(np.int64).tolist()))
    return grid, octo


# ── queries ─────────────────────────────────────────────────────────────────
def components(free):
    H, W = free.shape
    lab = np.zeros((H, W), dtype=np.int32)
    n = 0
    for r0, c0 in zip(*np.nonzero(free)):
        if lab[r0, c0]:
            continue
        n += 1
        lab[r0, c0] = n
        q = deque([(r0, c0)])
        while q:
            r, c = q.popleft()
            for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                rr, cc = r + dr, c + dc
                if 0 <= rr < H and 0 <= cc < W and free[rr, cc] and not lab[rr, cc]:
                    lab[rr, cc] = n
                    q.append((rr, cc))
    return lab


def sample_queries(planner, name, n, boxes, xs, ys):
    free = np.asarray(planner.layer_grids[FLY_LAYER]) == 0
    # Shell voxels leave obstacle interiors hollow: forbid cells inside any
    # obstacle that spans the flight band, and cells outside the footprint.
    H, W = free.shape
    X = planner.origin_x + (np.arange(W) + 0.5) * RES
    Y = planner.origin_y + (np.arange(H) + 0.5) * RES
    inside = np.zeros_like(free)
    z = Z_LAYERS[FLY_LAYER]
    for x0, x1, y0, y1, z0, z1 in boxes:
        if z0 <= z <= z1 and (x1 - x0) * (y1 - y0) < 0.9 * (xs[1] - xs[0]) * (ys[1] - ys[0]):
            ci = (X >= x0) & (X <= x1)
            ri = (Y >= y0) & (Y <= y1)
            inside[np.ix_(ri, ci)] = True
    inside |= ~((Y >= ys[0])[:, None] & (Y <= ys[1])[:, None]
                & (X >= xs[0])[None, :] & (X <= xs[1])[None, :])
    sample_ok = free & ~inside
    # keep cells whose 3x3 neighbourhood is free, so a start is not on an edge
    core = sample_ok.copy()
    core[1:-1, 1:-1] &= free[:-2, 1:-1] & free[2:, 1:-1] & free[1:-1, :-2] & free[1:-1, 2:]
    lab = components(free)
    cells = np.argwhere(core)
    if len(cells) < 2:
        return []
    seed = int(hashlib.md5(name.encode()).hexdigest()[:8], 16)
    rng = np.random.default_rng(seed)
    out, tries = [], 0
    while len(out) < n and tries < 20000:
        tries += 1
        a, b = cells[rng.integers(len(cells), size=2)]
        if lab[tuple(a)] != lab[tuple(b)]:
            continue
        sx, sy = planner._grid_to_world(*a)
        gx, gy = planner._grid_to_world(*b)
        d = math.hypot(gx - sx, gy - sy)
        if 4.0 <= d <= 25.0:
            out.append((sx, sy, gx, gy, d))
    return out


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
    pts = [np.asarray(p[0], float)]
    for a, b in zip(p, p[1:]):
        k = max(1, int(math.dist(a, b) / 0.05))
        t = np.linspace(0, 1, k + 1)[1:, None]
        pts.append(np.asarray(a, float) + t * (np.asarray(b, float) - np.asarray(a, float)))
    S = np.vstack(pts)
    lo, hi = S.min(0) - 2.0, S.max(0) + 2.0
    V = centers[np.all((centers >= lo) & (centers <= hi), axis=1)].astype(np.float64)
    if len(V) == 0:
        return 2.0
    best = np.inf
    V = V.astype(np.float32)
    S = S.astype(np.float32)
    for i in range(0, len(S), 8):
        d2 = ((S[i:i + 8, None, :] - V[None, :, :]) ** 2).sum(-1)
        best = min(best, float(d2.min()))
    return min(2.0, math.sqrt(best))


class _Timeout(Exception):
    pass


def _alarm(*_):
    raise _Timeout()


def timed(fn):
    signal.signal(signal.SIGALRM, _alarm)
    signal.alarm(TIMEOUT_S)
    t0 = time.perf_counter()
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            p = fn()
        to = 0
    except _Timeout:
        p, to = None, 1
    finally:
        signal.alarm(0)
    return p, (time.perf_counter() - t0) * 1000.0, to


# ── main ────────────────────────────────────────────────────────────────────
def world_list(only):
    ws = []
    for d in sorted(glob.glob(os.path.join(COLLECTION, '*', '*.sdf'))):
        name = os.path.basename(os.path.dirname(d))
        if name.startswith(SKIP):
            continue
        ws.append((name, name.split('_')[0][:2] if name[0] in 'FM' else name.split('_')[0],
                   d, not name.startswith(OUTDOOR), 'collection'))
    for w in ('house_3room', 'indoor_10x8x3', 'house_2floor'):
        ws.append((w, 'repo', os.path.join(REPO, 'worlds', w + '.sdf'), False, 'repo'))
    if only:
        keys = only.split(',')
        ws = [w for w in ws if any(w[0].startswith(k) for k in keys)]
    return ws


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--n', type=int, default=15)
    ap.add_argument('--only', default='')
    ap.add_argument('--out', default=os.path.join(REPO, 'benchmark', 'ahppa_multiworld_results.csv'))
    a = ap.parse_args()

    feat_deployed = ClearanceEstimator(band_z=1.0)   # the 2026-11-17 version
    feat_h = ClearanceEstimator(band_z=0.3)
    P.set_clock(lambda: 0.0)

    fields = ['world', 'family', 'source', 'indoor', 'qid', 'dist', 'planner', 'mode',
              'start_clearance', 'ok', 'timeout', 'ms', 'length', 'waypoints',
              'turning', 'min_clear', 'build_s', 'voxels', 'cells']
    new = not os.path.exists(a.out)
    done = set()
    if not new:
        with open(a.out) as f:
            done = {r['world'] for r in csv.DictReader(f)}
    f = open(a.out, 'a', newline='')
    w = csv.DictWriter(f, fieldnames=fields)
    if new:
        w.writeheader()

    # The repo houses already have real ceilings / floors: never 'indoorise'.
    for name, fam, path, indoor, src in world_list(a.only):
        if name in done:
            print(f"skip {name} (already in {a.out})")
            continue
        t0 = time.time()
        upright, tilted = load_geometry(path)
        boxes, xs, ys = prepare(upright, indoor)
        centers = shell_voxels(boxes, tilted)
        grid, octo = make_inputs(centers, xs, ys)
        with contextlib.redirect_stdout(io.StringIO()):
            port = P.PortfolioPlanner(grid, octo, safety_margin=SAFETY_MARGIN,
                                      unknown_penalty=UNKNOWN_PENALTY)
        build = time.time() - t0
        qs = sample_queries(port, name, a.n, boxes, xs, ys)
        cells = grid.info.width * grid.info.height
        print(f"\n{name}  [{fam}{', indoor' if indoor else ''}]  {len(centers)} voxels, "
              f"{grid.info.width}x{grid.info.height}, build {build:.1f}s, {len(qs)} queries",
              flush=True)

        for qi, (sx, sy, gx, gy, d) in enumerate(qs):
            res = {}

            def stock():
                return MultiLayerPlanner.plan(port, sx, sy, gx, gy,
                                              start_layer=FLY_LAYER, goal_layer=FLY_LAYER)

            def ahppa(feature):
                def run():
                    P.FEATURE = feature
                    P.DISPATCH = HysteresisDispatcher()
                    P.WATCHDOG.reset()
                    return port.plan(sx, sy, gx, gy, start_layer=FLY_LAYER,
                                     goal_layer=FLY_LAYER)
                return run

            for label, fn in (('stock', stock), ('ahppa', ahppa(feat_deployed)),
                              ('ahppa_h', ahppa(feat_h))):
                p, ms, to = timed(fn)
                mode = 'A*' if label == 'stock' else (port.last_mode or '')
                sc = '' if label == 'stock' else round(float(P.FEATURE.last_clearance), 3)
                ok = bool(p) and math.dist(p[-1][:2], (gx, gy)) < 0.6
                res[label] = (mode, ms, ok)
                w.writerow(dict(
                    world=name, family=fam, source=src, indoor=int(indoor), qid=qi,
                    dist=round(d, 2), planner=label, mode=mode, start_clearance=sc,
                    ok=int(ok), timeout=to, ms=round(ms, 1),
                    length=round(polyline_len(p), 3) if ok else '',
                    waypoints=len(p) if ok else 0,
                    turning=round(turning(p), 1) if ok else '',
                    min_clear=round(min_clearance(p, centers), 3) if ok else '',
                    build_s=round(build, 1), voxels=len(centers), cells=cells))
            f.flush()
            s, h, hh = res['stock'], res['ahppa'], res['ahppa_h']
            print(f"  q{qi:02d} {d:5.1f} m  stock {'ok' if s[2] else '--'} {s[1]:7.0f}ms | "
                  f"ahppa {h[0]:6s} {'ok' if h[2] else '--'} {h[1]:7.0f}ms | "
                  f"ahppa_h {hh[0]:6s} {'ok' if hh[2] else '--'} {hh[1]:7.0f}ms", flush=True)
        print(f"  world done in {time.time() - t0:.0f}s", flush=True)
    f.close()


if __name__ == '__main__':
    main()
