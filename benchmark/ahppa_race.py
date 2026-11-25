#!/usr/bin/env python3
"""Measured (not simulated) benchmark of the Lazy Theta* / stock A* race.

Per query, same process, same load:
    stock     stock A*, in-process - the reference, and the yardstick for
              fork overhead (race t_stock - stock ms)
    observe   AHPPA_RACE_OBSERVE: both children run to completion; records
              both finish times and both paths, so ANY wait setting w can be
              scored afterwards (lt taken iff t_lt <= (1 + w) * t_stock)
    race0     the live race with RACE_WAIT = 0, end to end - checks that the
              observe-derived numbers are what the live code actually does

Same queries as ahppa_multiworld_results.csv (seeded by world name).
Timing on the VM varies 2-3x run to run (host scheduling), so every query is
run --repeats times with the order rotated; analyse per-query medians.
Each race uses two cores: run at most (cores / 2) shards in parallel.

  source /opt/ros/humble/setup.bash
  python3 benchmark/ahppa_race.py [--only SC6,F2] [--out file.csv]
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

P = M.P


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--n', type=int, default=30)
    ap.add_argument('--only', default='')
    ap.add_argument('--repeats', type=int, default=3)
    ap.add_argument('--out', default=os.path.join(M.REPO, 'benchmark', 'ahppa_race_results.csv'))
    a = ap.parse_args()
    fields = ['world', 'family', 'indoor', 'qid', 'rep', 'dist', 'planner', 'ok', 'timeout', 'ms',
              'length', 'min_clear', 'winner', 't_lt', 't_stock', 'lt_gave_up',
              'lt_length', 'lt_min_clear', 'st_length', 'st_min_clear']
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
        qs = M.sample_queries(port, name, a.n, boxes, xs, ys)
        print(f"\n{name}  {len(qs)} queries", flush=True)
        fl = M.FLY_LAYER

        def ok_(p, gx, gy):
            return bool(p) and math.dist(p[-1][:2], (gx, gy)) < 0.6

        def m(p, fn):
            return round(fn(p), 3) if p else ''
        for qi, (sx, sy, gx, gy, d) in enumerate(qs):
            out = []
            labels = ('stock', 'observe', 'race0')
            for rp, label in [(r, labels[(i + r) % 3]) for r in range(a.repeats) for i in range(3)]:
                P.RACE = label != 'stock'
                P.RACE_OBSERVE = label == 'observe'
                P.RACE_WAIT = 0.0
                P.WATCHDOG.reset()
                if label == 'stock':
                    fn = lambda: M.MultiLayerPlanner.plan(port, sx, sy, gx, gy, start_layer=fl, goal_layer=fl)
                else:
                    fn = lambda: port.plan(sx, sy, gx, gy, start_layer=fl, goal_layer=fl)
                p, ms, to = M.timed(fn)
                ok = ok_(p, gx, gy)
                rc = port.last_race if label != 'stock' else None
                row = dict(world=name, family=fam, indoor=int(indoor), qid=qi, rep=rp, dist=round(d, 2),
                           planner=label, ok=int(ok), timeout=to, ms=round(ms, 1),
                           length=m(p, M.polyline_len) if ok else '',
                           min_clear=round(M.min_clearance(p, centers), 3) if ok else '')
                if rc:
                    plt, pst = rc.get('p_lt'), rc.get('p_stock')
                    row.update(winner=('lt' if p is not None and p == plt else
                                       'stock' if p is not None and p == pst else 'fallback'),
                               t_lt=rc.get('t_lt', ''), t_stock=rc.get('t_stock', ''),
                               lt_gave_up=rc.get('lt_gave_up') or '',
                               lt_length=m(plt, M.polyline_len) if ok_(plt, gx, gy) else '',
                               lt_min_clear=round(M.min_clearance(plt, centers), 3) if ok_(plt, gx, gy) else '',
                               st_length=m(pst, M.polyline_len) if ok_(pst, gx, gy) else '',
                               st_min_clear=round(M.min_clearance(pst, centers), 3) if ok_(pst, gx, gy) else '')
                w.writerow(row)
                if rp:
                    continue
                out.append(f"{label} {'ok' if ok else ('TO' if to else '--')} {ms:6.0f}ms"
                           + (f" [{row['winner']}]" if rc else ''))
            f.flush()
            print(f"  q{qi:02d} {d:5.1f} m  " + " | ".join(out), flush=True)
        print(f"  world done in {time.time() - t0:.0f}s", flush=True)
    P.RACE = False
    f.close()


if __name__ == '__main__':
    main()
