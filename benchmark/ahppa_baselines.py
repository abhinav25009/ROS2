#!/usr/bin/env python3
"""Single-planner baselines for the AHPPA portfolio, on the 44-layout set.

AHPPA picks one of three planners per query. To know whether the picking is
worth anything we need each member run on EVERY query, not just the ones the
dispatcher sent it:

    lt_only   Lazy Theta* (the OPEN member), always
    ca_only   clearance A* (the DENSE / ESCAPE member), always

stock A* (the MEDIUM member) and AHPPA itself are already in
ahppa_multiworld_results.csv for the same queries - sample_queries() is
seeded by world name, so the start/goal pairs are identical (checked on load
by comparing distances). From the three members per query,
analyse_baselines.py derives the oracle (best member per query, chosen in
hindsight - the ceiling for ANY selector), a uniform random selector, and
the best fixed planner.

Members are run RAW: a failure is a failure. AHPPA falls back to stock A* when
its member returns nothing; the analysis applies the same fallback to the
baselines so the comparison is like for like, and reports both.

  source /opt/ros/humble/setup.bash
  python3 benchmark/ahppa_baselines.py [--only SC6,F2] [--out file.csv]
"""
import argparse
import contextlib
import csv
import io
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ahppa_multiworld as M                                   # noqa: E402

PRIOR = os.path.join(M.REPO, 'benchmark', 'ahppa_multiworld_results.csv')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--n', type=int, default=30)
    ap.add_argument('--only', default='')
    ap.add_argument('--out', default=os.path.join(M.REPO, 'benchmark', 'ahppa_baselines_results.csv'))
    a = ap.parse_args()

    prior = {}
    for r in csv.DictReader(open(PRIOR)):
        if r['planner'] == 'stock':
            prior[(r['world'], int(r['qid']))] = float(r['dist'])

    fields = ['world', 'family', 'indoor', 'qid', 'dist', 'planner', 'ok', 'timeout', 'ms',
              'length', 'waypoints', 'turning', 'min_clear']
    new = not os.path.exists(a.out)
    done = set()
    if not new:
        done = {r['world'] for r in csv.DictReader(open(a.out))}
    f = open(a.out, 'a', newline='')
    w = csv.DictWriter(f, fieldnames=fields)
    if new:
        w.writeheader()

    for name, fam, path, indoor, src in M.world_list(a.only):
        if name in done:
            print(f"skip {name}")
            continue
        t0 = time.time()
        upright, tilted = M.load_geometry(path)
        boxes, xs, ys = M.prepare(upright, indoor)
        centers = M.shell_voxels(boxes, tilted)
        grid, octo = M.make_inputs(centers, xs, ys)
        with contextlib.redirect_stdout(io.StringIO()):
            port = M.P.PortfolioPlanner(grid, octo, safety_margin=M.SAFETY_MARGIN,
                                        unknown_penalty=M.UNKNOWN_PENALTY)
        qs = M.sample_queries(port, name, a.n, boxes, xs, ys)
        bad = [qi for qi, q in enumerate(qs)
               if (name, qi) in prior and abs(prior[(name, qi)] - round(q[4], 2)) > 0.011]
        if bad or not any((name, qi) in prior for qi in range(len(qs))):
            sys.exit(f"{name}: queries do not match {PRIOR} (qids {bad[:5]}) - "
                     f"wrong --n or the world/sampler changed")
        z = M.Z_LAYERS[M.FLY_LAYER]
        print(f"\n{name}  {len(qs)} queries (match prior run)", flush=True)
        for qi, (sx, sy, gx, gy, d) in enumerate(qs):
            line = []
            for label, member in (('lt_only', port._lt), ('ca_only', port._ca)):
                p, ms, to = M.timed(lambda m=member: m.plan(port._grids, (sx, sy, z), (gx, gy, z)))
                ok = bool(p) and math.dist(p[-1][:2], (gx, gy)) < 0.6
                w.writerow(dict(world=name, family=fam, indoor=int(indoor), qid=qi,
                                dist=round(d, 2), planner=label, ok=int(ok), timeout=to,
                                ms=round(ms, 1),
                                length=round(M.polyline_len(p), 3) if ok else '',
                                waypoints=len(p) if ok else 0,
                                turning=round(M.turning(p), 1) if ok else '',
                                min_clear=round(M.min_clearance(p, centers), 3) if ok else ''))
                line.append(f"{label} {'ok' if ok else ('TO' if to else '--')} {ms:7.0f}ms")
            f.flush()
            print(f"  q{qi:02d} {d:5.1f} m  " + " | ".join(line), flush=True)
        print(f"  world done in {time.time() - t0:.0f}s", flush=True)
    f.close()


if __name__ == '__main__':
    main()
