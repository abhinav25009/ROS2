#!/usr/bin/env python3
"""Upstairs in house_2floor, under the real 12.6 deg sloped roof.

The 44-layout benchmark only flew at 1.8 m. This samples first-floor queries
at the 4.2, 4.8 and 5.4 m layers - where the roof is 1.2-2.8 m, 0.6-2.2 m and
0-1.6 m above the drone - inside the house footprint, and compares
stock A*, AHPPA with the old +/-1.0 m clearance band and AHPPA with the
default +/-0.3 m band. Roof geometry is the tilted shape itself (see
ahppa_multiworld.load_geometry), not an axis-aligned approximation.

  source /opt/ros/humble/setup.bash
  python3 benchmark/upstairs_sloped_roof.py [--n 20] [--out file.csv]
"""
import argparse
import contextlib
import csv
import io
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ahppa_multiworld as M                                   # noqa: E402

LAYERS = (5, 6, 7)                       # 4.2, 4.8, 5.4 m
HOUSE = (-6.9, 6.9, -4.9, 4.9)           # inside the outer walls


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--n', type=int, default=20)
    ap.add_argument('--out', default=os.path.join(M.REPO, 'benchmark', 'upstairs_sloped_roof.csv'))
    a = ap.parse_args()
    path = os.path.join(M.REPO, 'worlds', 'house_2floor.sdf')
    upright, tilted = M.load_geometry(path)
    boxes, xs, ys = M.prepare(upright, False)
    centers = M.shell_voxels(boxes, tilted)
    grid, octo = M.make_inputs(centers, xs, ys)
    with contextlib.redirect_stdout(io.StringIO()):
        port = M.P.PortfolioPlanner(grid, octo, safety_margin=M.SAFETY_MARGIN,
                                    unknown_penalty=M.UNKNOWN_PENALTY)
    M.P.set_clock(lambda: 0.0)
    feats = {'ahppa_band1.0': M.ClearanceEstimator(band_z=1.0),
             'ahppa_band0.3': M.ClearanceEstimator()}          # = the new default
    assert feats['ahppa_band0.3'].band_z == 0.3
    rows = []
    for li in LAYERS:
        M.FLY_LAYER = li
        qs = [q for q in M.sample_queries(port, f"house_2floor_up_{li}", 25 * a.n, boxes, xs, ys)
              if all(HOUSE[0] < v < HOUSE[1] for v in (q[0], q[2]))
              and all(HOUSE[2] < v < HOUSE[3] for v in (q[1], q[3]))][:a.n]
        z = M.Z_LAYERS[li]
        print(f"\nlayer {li} (z = {z} m): {len(qs)} queries", flush=True)
        for qi, (sx, sy, gx, gy, d) in enumerate(qs):
            res = {}

            def stock():
                return M.MultiLayerPlanner.plan(port, sx, sy, gx, gy, start_layer=li, goal_layer=li)
            runs = [('stock', stock, None)] + [(k, None, f) for k, f in feats.items()]
            for lab, fn, feat in runs:
                if fn is None:
                    def fn(feat=feat):
                        M.P.FEATURE = feat
                        M.P.DISPATCH = M.HysteresisDispatcher()
                        M.P.WATCHDOG.reset()
                        return port.plan(sx, sy, gx, gy, start_layer=li, goal_layer=li)
                p, ms, to = M.timed(fn)
                ok = bool(p) and math.dist(p[-1][:2], (gx, gy)) < 0.6
                rows.append(dict(layer_z=z, qid=qi, dist=round(d, 2), planner=lab,
                                 mode='A*' if feat is None else port.last_mode,
                                 start_clearance='' if feat is None else round(feat.last_clearance, 3),
                                 ok=int(ok), timeout=to, ms=round(ms, 1),
                                 length=round(M.polyline_len(p), 3) if ok else '',
                                 waypoints=len(p) if ok else 0,
                                 min_clear=round(M.min_clearance(p, centers), 3) if ok else '',
                                 sx=round(sx, 2), sy=round(sy, 2)))
                res[lab] = rows[-1]
            s, o, n = res['stock'], res['ahppa_band1.0'], res['ahppa_band0.3']
            print(f"  q{qi:02d} start ({sx:5.1f},{sy:5.1f})  old band {o['mode']:6s} "
                  f"c={o['start_clearance']:.2f}  new band {n['mode']:6s} c={n['start_clearance']:.2f}  "
                  f"len {s['length']} / {o['length']} / {n['length']}", flush=True)
    with open(a.out, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    print(f"\n-> {a.out}")


if __name__ == '__main__':
    main()
