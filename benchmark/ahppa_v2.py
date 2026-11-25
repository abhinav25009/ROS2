#!/usr/bin/env python3
"""Candidate planners for AHPPA v2, on the same 44-layout / 1,320 queries.

Per query, in one process, one after another:
    stock     stock layered A* (dense grid path)
    ps        stock A* + greedy line-of-sight shortcutting ("post-smoothing"):
              the obvious cheap any-angle baseline a reviewer will ask for
    lt        Lazy Theta*, budget 25 s (so its real cost is visible; the 5 s
              budget can be applied afterwards from the time)
    clt<R>    corridor Lazy Theta*: Lazy Theta* restricted to cells within R m
              of the smoothed A* path. Any-angle quality, a fraction of the
              search. Falls back to the smoothed path if the corridor search
              fails. R = 1.0 and 2.0 m.
For ps / clt the time columns are the EXTRA time on top of stock A*, so the
analysis can score "plan now with A*, refine while flying" (anytime) as well
as the sequential total.

  python3 benchmark/ahppa_v2.py [--only SC6,F2] [--out file.csv]
"""
import argparse
import contextlib
import csv
import io
import math
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ahppa_multiworld as M                                   # noqa: E402

P = M.P
RADII = (1.0, 2.0)


def shortcut(lt, grids, path):
    """Greedy string pulling: from each anchor, walk forward while the
    straight segment anchor -> next point stays in free space."""
    if not path or len(path) < 3:
        return path
    cells = [lt.to_cell(*p) for p in path]
    out = [path[0]]
    i, n = 0, len(path)
    while i < n - 1:
        j = i + 1
        while j + 1 < n and lt.line_of_sight(grids, cells[i], cells[j + 1]):
            j += 1
        out.append(path[j])
        i = j
    return out


def corridor_grids(lt, grids, path, radius):
    """Block every cell farther than radius (xy) from the polyline."""
    L, H, W = grids.shape
    k = max(1, int(round(radius / lt.res)))
    keep = np.zeros((H, W), dtype=bool)
    pts = np.asarray(path, dtype=np.float64)
    for a, b in zip(pts[:-1], pts[1:]):
        d = math.hypot(b[0] - a[0], b[1] - a[1])
        m = max(2, int(d / (lt.res * 2)) + 2)
        for t in np.linspace(0.0, 1.0, m):
            x, y = a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t
            r, c = int((y - lt.oy) / lt.res), int((x - lt.ox) / lt.res)
            keep[max(0, r - k):r + k + 1, max(0, c - k):c + k + 1] = True
    return grids | ~keep[None, :, :]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--n', type=int, default=30)
    ap.add_argument('--only', default='')
    ap.add_argument('--out', default=os.path.join(M.REPO, 'benchmark', 'ahppa_v2_results.csv'))
    a = ap.parse_args()
    fields = ['world', 'family', 'indoor', 'qid', 'dist', 'planner', 'ok', 'timeout', 'ms',
              'length', 'min_clear', 'waypoints', 'gave_up', 'expansions']
    new = not os.path.exists(a.out)
    done = set() if new else {r['world'] for r in csv.DictReader(open(a.out))}
    f = open(a.out, 'a', newline='')
    w = csv.DictWriter(f, fieldnames=fields)
    if new:
        w.writeheader()
    P.set_clock(lambda: 0.0)
    for name, fam, path, indoor, src in M.world_list(a.only):
        if name in done:
            continue
        t0 = time.time()
        upright, tilted = M.load_geometry(path)
        boxes, xs, ys = M.prepare(upright, indoor)
        centers = M.shell_voxels(boxes, tilted)
        grid, octo = M.make_inputs(centers, xs, ys)
        with contextlib.redirect_stdout(io.StringIO()):
            port = P.PortfolioPlanner(grid, octo, safety_margin=M.SAFETY_MARGIN,
                                      unknown_penalty=M.UNKNOWN_PENALTY)
        lt, G = port._lt, port._grids
        qs = M.sample_queries(port, name, a.n, boxes, xs, ys)
        print(f"\n{name}  {len(qs)} queries", flush=True)
        fl = M.FLY_LAYER

        for qi, (sx, sy, gx, gy, d) in enumerate(qs):
            def ok_(p):
                return bool(p) and math.dist(p[-1][:2], (gx, gy)) < 0.6

            def row(label, p, ms, to, **kw):
                ok = ok_(p)
                w.writerow(dict(world=name, family=fam, indoor=int(indoor), qid=qi, dist=round(d, 2),
                                planner=label, ok=int(ok), timeout=to, ms=round(ms, 1),
                                length=round(M.polyline_len(p), 3) if ok else '',
                                min_clear=round(M.min_clearance(p, centers), 3) if ok else '',
                                waypoints=len(p) if ok else 0, **kw))
                return ok
            line = []
            p_st, ms, to = M.timed(lambda: M.MultiLayerPlanner.plan(port, sx, sy, gx, gy,
                                                                      start_layer=fl, goal_layer=fl))
            row('stock', p_st, ms, to)
            line.append(f"st {ms:5.0f}")
            p_ps = None
            if ok_(p_st):
                p_ps, ms, to = M.timed(lambda: shortcut(lt, G, p_st))
                row('ps', p_ps, ms, to)
                line.append(f"ps +{ms:4.0f}")
                for R in RADII:
                    def clt():
                        cg = corridor_grids(lt, G, p_ps, R)
                        lt.time_budget_s = 25.0
                        q = lt.plan(cg, (sx, sy, P.Z_LAYERS[fl]), (gx, gy, P.Z_LAYERS[fl]))
                        return q
                    p, ms, to = M.timed(clt)
                    row(f'clt{R:g}', p, ms, to, gave_up=lt.gave_up or '', expansions=lt.expansions)
                    line.append(f"clt{R:g} +{ms:5.0f}")
            lt.time_budget_s = 25.0
            p, ms, to = M.timed(lambda: lt.plan(G, (sx, sy, P.Z_LAYERS[fl]), (gx, gy, P.Z_LAYERS[fl])))
            row('lt', p, ms, to, gave_up=lt.gave_up or '', expansions=lt.expansions)
            line.append(f"lt {ms:6.0f}")
            f.flush()
            print(f"  q{qi:02d} {d:5.1f} m  " + " | ".join(line), flush=True)
        print(f"  world done in {time.time() - t0:.0f}s", flush=True)
    f.close()


if __name__ == '__main__':
    main()
