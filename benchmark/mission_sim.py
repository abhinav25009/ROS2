#!/usr/bin/env python3
"""Closed-loop mission benchmark: fly whole missions on maps built in flight.

Every earlier benchmark planned ONE path on a FULLY KNOWN map. A real flight
starts with an empty map, sees obstacles as it goes, replans when the path
ahead turns out to be blocked, hovers while it plans, and can get stuck.
This simulates exactly that loop on the same 44 layouts, without Gazebo, so
it can run hundreds of missions per planner:

    map      starts EMPTY (unknown = free, the same optimistic assumption the
             real stack makes). Every SENSE_DT seconds a 5 m sensor reveals
             obstacles: 720 rays on the drone's own layer (walls occlude), and
             everything within range on the layers above/below (ceiling and
             floor). Revealed cells are inflated by the planner's safety
             margin, exactly as the planner does.
    motion   the drone follows the current path at V m/s (kinematic).
    replan   every CHECK_DT s, if newly revealed cells block the path within
             the sensor horizon; the drone HOVERS while the planner runs
             (mission time += planning time).
    stuck    a progress watchdog (the real AHPPA DeadlockWatchdog, 6 s
             window, 0.30 m) - AHPPA plans in ESCAPE while it is tripped;
             for single planners a trip just forces a replan.
    end      success = within 0.5 m of the goal; failure = no path, a
             collision with a TRUE obstacle (drone centre within 0.15 m), or
             the time limit (60 s + 6 x straight distance / V).

Planners (any subset with --planners):
    stock   layered A*
    ps      layered A* + line-of-sight shortcutting
    lt      Lazy Theta* (5 s budget, stock fallback)
    ahppa   the shipped portfolio, real dispatcher + watchdog + ESCAPE
    v2      AHPPA v2 (see V2_POLICY below)

  python3 benchmark/mission_sim.py [--only SC6] [--n 10] [--planners stock,ps]
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

os.environ.setdefault('AHPPA_PORTFOLIO', '1')
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ahppa_multiworld as M                                   # noqa: E402
from ahppa_v2 import shortcut                                  # noqa: E402

P = M.P
from dispatcher import HysteresisDispatcher                    # noqa: E402

V = 1.0              # m/s cruise
DT = 0.1             # s motion step
SENSE_DT = 0.2       # s between sensor sweeps
RANGE = 5.0          # m sensor range
CHECK_DT = 0.2       # s between path-ahead checks (= every sweep; 1 s let corners hit)
RAYS = 720
HIT_R = 0.15         # m: drone centre this close to a TRUE obstacle = collision
GOAL_R = 0.5


class World:
    """Ground truth + the drone's growing map, wired into one planner."""

    def __init__(self, port, centers):
        self.port = port
        self.lt = port._lt
        self.res = port.resolution
        self.ox, self.oy = port.origin_x, port.origin_y
        self.H, self.W = port.height, port.width
        self.L = port.num_layers
        base = np.zeros((self.H, self.W), dtype=np.uint8)
        with contextlib.redirect_stdout(io.StringIO()):
            self.truth = np.array([port._build_layer_grid(base, z) for z in P.Z_LAYERS], dtype=bool)
        self.k = int(math.ceil(M.SAFETY_MARGIN / self.res))
        # collision test grid: true obstacles dilated by HIT_R
        kh = max(1, int(round(HIT_R / self.res)))
        self.hit = np.zeros_like(self.truth)
        for li in range(self.L):
            rr, cc = np.nonzero(self.truth[li])
            for r, c in zip(rr, cc):
                self.hit[li, max(0, r - kh):r + kh + 1, max(0, c - kh):c + kh + 1] = True
        self.centers = centers
        self.vkey = np.floor(centers / M.RES).astype(np.int64)
        ang = np.linspace(0, 2 * np.pi, RAYS, endpoint=False)
        steps = np.arange(1, int(RANGE / (self.res * 0.5)) + 1) * self.res * 0.5
        self.ray_dx = np.cos(ang)[:, None] * steps[None, :]
        self.ray_dy = np.sin(ang)[:, None] * steps[None, :]
        self.reset()

    def reset(self):
        self.seen = np.zeros_like(self.truth)
        g = np.zeros((self.H, self.W), dtype=np.uint8)
        self.port.layer_grids = [g.copy() for _ in range(self.L)]
        self.port._grids = np.zeros((self.L, self.H, self.W), dtype=bool)
        self.port.raw = np.zeros((self.H, self.W), dtype=np.int8)
        self.port.octomap_3d.voxel_centers = np.zeros((0, 3), dtype=np.float32)
        self._seen_vox = np.zeros(len(self.centers), dtype=bool)

    def sense(self, x, y, z=None):
        """Reveal obstacles; return the number of newly revealed cells.

        On the drone's own layer: 360-degree rays, first hit per ray (walls
        occlude what is behind them). On every OTHER layer: everything within
        range. Those bands are above or below the drone, where what blocks
        them indoors is the ceiling or floor - horizontal surfaces a depth
        camera sees directly. Without this the ceiling band looked open and
        planners climbed over walls into it (first mission run, 2026-11-19)."""
        own = (int(np.argmin([abs(z - zz) for zz in P.Z_LAYERS])) if z is not None else None)
        new = self._reveal_disk(x, y, own)
        X = x + self.ray_dx
        Y = y + self.ray_dy
        c = np.floor((X - self.ox) / self.res).astype(np.int64)
        r = np.floor((Y - self.oy) / self.res).astype(np.int64)
        inb = (r >= 0) & (r < self.H) & (c >= 0) & (c < self.W)
        r = np.clip(r, 0, self.H - 1)
        c = np.clip(c, 0, self.W - 1)
        for li in range(self.L):
            if own is not None and li != own:
                continue
            occ = self.truth[li][r, c] & inb
            hit = occ.any(1)
            first = occ.argmax(1)
            rows = np.nonzero(hit)[0]
            hr, hc = r[rows, first[rows]], c[rows, first[rows]]
            fresh = ~self.seen[li, hr, hc]
            if not fresh.any():
                continue
            hr, hc = hr[fresh], hc[fresh]
            new += self._mark(li, hr, hc)
        if new:
            # voxels for the clearance feature: those in seen columns
            vr = np.floor((self.centers[:, 1] - self.oy) / self.res).astype(np.int64)
            vc = np.floor((self.centers[:, 0] - self.ox) / self.res).astype(np.int64)
            ok = (vr >= 0) & (vr < self.H) & (vc >= 0) & (vc < self.W)
            any_seen = self.seen.any(0)
            m = np.zeros(len(self.centers), dtype=bool)
            m[ok] = any_seen[vr[ok], vc[ok]]
            self.port.octomap_3d.voxel_centers = self.centers[m]
        return new

    def _mark(self, li, hr, hc):
        self.seen[li, hr, hc] = True
        k = self.k
        for a, b in zip(hr, hc):
            sl = (slice(max(0, a - k), a + k + 1), slice(max(0, b - k), b + k + 1))
            self.port.layer_grids[li][sl] = 1
            self.port._grids[li][sl] = True
        return len(hr)

    def _reveal_disk(self, x, y, own):
        r0 = int((y - self.oy) / self.res)
        c0 = int((x - self.ox) / self.res)
        k = int(RANGE / self.res)
        ra, rb = max(0, r0 - k), min(self.H, r0 + k + 1)
        ca, cb = max(0, c0 - k), min(self.W, c0 + k + 1)
        rr, cc = np.mgrid[ra:rb, ca:cb]
        disk = (rr - r0) ** 2 + (cc - c0) ** 2 <= k * k
        new = 0
        for li in range(self.L):
            if li == own:
                continue
            m = self.truth[li, ra:rb, ca:cb] & disk & ~self.seen[li, ra:rb, ca:cb]
            if m.any():
                new += self._mark(li, rr[m], cc[m])
        return new

    def blocked(self, a, b):
        """Is the straight segment a -> b blocked in the drone's KNOWN map?"""
        lt = self.lt
        return not lt.line_of_sight(self.port._grids, lt.to_cell(*a), lt.to_cell(*b))

    def collided(self, x, y, z):
        li = int(np.argmin([abs(z - zz) for zz in P.Z_LAYERS]))
        r = int((y - self.oy) / self.res)
        c = int((x - self.ox) / self.res)
        if not (0 <= r < self.H and 0 <= c < self.W):
            return True
        return bool(self.hit[li, r, c])


# ── planners ────────────────────────────────────────────────────────────────
def mk_planners(w):
    port, lt = w.port, w.lt
    fl = M.FLY_LAYER

    def stock(x, y, z, gx, gy):
        li = int(np.argmin([abs(z - zz) for zz in P.Z_LAYERS]))
        return M.MultiLayerPlanner.plan(port, x, y, gx, gy, start_layer=li, goal_layer=fl), 'stock'

    def ps(x, y, z, gx, gy):
        p, _ = stock(x, y, z, gx, gy)
        return (shortcut(lt, port._grids, p) if p else p), 'ps'

    def lt_(x, y, z, gx, gy):
        lt.time_budget_s = 5.0
        p = lt.plan(port._grids, (x, y, z), (gx, gy, P.Z_LAYERS[fl]))
        if p:
            return p, 'lt'
        return stock(x, y, z, gx, gy)[0], 'lt->stock'

    def ahppa(x, y, z, gx, gy):
        li = int(np.argmin([abs(z - zz) for zz in P.Z_LAYERS]))
        p = port.plan(x, y, gx, gy, start_layer=li, goal_layer=fl)
        return p, port.last_mode or ''

    def v2(x, y, z, gx, gy):
        """AHPPA v2: smoothed A* normally; clearance A* while deadlocked."""
        if P.WATCHDOG.tripped:
            port._ca.time_budget_s = 5.0
            p = port._ca.plan(port._grids, (x, y, z), (gx, gy, P.Z_LAYERS[fl]))
            if p:
                return shortcut(lt, port._grids, p), 'ESCAPE'
        return ps(x, y, z, gx, gy)[0], 'ps'

    return dict(stock=stock, ps=ps, lt=lt_, ahppa=ahppa, v2=v2)


def fly(w, plan_fn, sx, sy, gx, gy, use_watchdog):
    """One mission. Returns a metrics dict."""
    z0 = P.Z_LAYERS[M.FLY_LAYER]
    x, y, z = sx, sy, z0
    t = 0.0
    P.set_clock(lambda: t_ref[0])
    t_ref = [0.0]
    P.DISPATCH = HysteresisDispatcher()
    P.WATCHDOG.reset()
    P._seen_trips = 0
    P._last_obs_t = None
    w.reset()
    w.sense(x, y, z)
    limit = 60.0 + 6.0 * math.hypot(gx - sx, gy - sy) / V
    plan_s, flown, replans, trips, modes = 0.0, 0.0, 0, 0, {}
    path, idx = None, 1
    need = True
    next_sense, next_check, fresh = SENSE_DT, CHECK_DT, False
    outcome, col, t_plan = 'timeout', '', 0.0
    while t < limit:
        if math.hypot(gx - x, gy - y) < GOAL_R:
            outcome = 'success'
            break
        if need:
            t0 = time.perf_counter()
            with contextlib.redirect_stdout(io.StringIO()):
                p, mode = plan_fn(x, y, z, gx, gy)
            dt_plan = time.perf_counter() - t0
            plan_s += dt_plan
            t += dt_plan                                   # hover while planning
            t_ref[0] = t
            replans += 1
            t_plan = t
            modes[mode] = modes.get(mode, 0) + 1
            if not p:
                outcome = 'no_path'
                break
            # fly to path[0] first: when the start was nudged off a blocked
            # cell, path[0] is not where the drone is
            path, need = list(p), False
            idx = 1 if math.dist(path[0], (x, y, z)) < 1e-3 else 0
            if len(path) == 1:
                path = [path[0], (gx, gy, path[0][2])]
        # move DT along the path
        rem = V * DT
        while rem > 1e-9 and idx < len(path):
            tx, ty, tz = path[idx]
            d = math.dist((x, y, z), (tx, ty, tz))
            if d <= rem:
                x, y, z = tx, ty, tz
                rem -= d
                flown += d
                idx += 1
            else:
                f = rem / d
                x, y, z = x + (tx - x) * f, y + (ty - y) * f, z + (tz - z) * f
                flown += rem
                rem = 0.0
        t += DT
        t_ref[0] = t
        if w.collided(x, y, z):
            outcome = 'collision'
            li = int(np.argmin([abs(z - zz) for zz in P.Z_LAYERS]))
            r, c = int((y - w.oy) / w.res), int((x - w.ox) / w.res)
            k = w.k + 2
            win = (slice(max(0, r - k), r + k + 1), slice(max(0, c - k), c + k + 1))
            # was the obstacle we hit already in the drone's map?
            col = (f"{x:.2f} {y:.2f} {z:.2f} L{li} seen_near={int(w.seen[li][win].any())} "
                   f"known_blocked_here={int(w.port._grids[li, r, c])} since_plan={t - t_plan:.1f}s")
            break
        if idx >= len(path):
            need = True                                    # path used up, not at goal
        if t >= next_sense:
            next_sense = t + SENSE_DT
            fresh |= bool(w.sense(x, y, z))
        if fresh and t >= next_check:
            # like the navigator: a periodic check of the path ahead, but of
            # SEGMENTS (not just waypoints) up to the sensor horizon
            next_check, fresh = t + CHECK_DT, False
            pts, acc = [(x, y, z)], 0.0
            for q in path[idx:]:
                acc += math.dist(pts[-1], q)
                pts.append(q)
                if acc > RANGE + 1.0:
                    break
            if any(w.blocked(a, b) for a, b in zip(pts, pts[1:])):
                need = True
        if use_watchdog:
            P.observe(x, y, z, True)
            if P.deadlock_edge():
                trips += 1
                need = True
    return dict(outcome=outcome, time_s=round(t, 2), plan_s=round(plan_s, 3), flown=round(flown, 2),
                replans=replans, trips=trips, straight=round(math.hypot(gx - sx, gy - sy), 2),
                modes=' '.join(f'{k}:{v}' for k, v in sorted(modes.items())), collision=col)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--n', type=int, default=10)
    ap.add_argument('--only', default='')
    ap.add_argument('--planners', default='stock,ps,lt,ahppa,v2')
    ap.add_argument('--out', default=os.path.join(M.REPO, 'benchmark', 'mission_results.csv'))
    a = ap.parse_args()
    names = a.planners.split(',')
    fields = ['world', 'family', 'indoor', 'mid', 'planner', 'outcome', 'time_s', 'plan_s', 'flown',
              'straight', 'replans', 'trips', 'modes', 'collision']
    new = not os.path.exists(a.out)
    done = set() if new else {r['world'] for r in csv.DictReader(open(a.out))}
    f = open(a.out, 'a', newline='')
    wr = csv.DictWriter(f, fieldnames=fields)
    if new:
        wr.writeheader()
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
        qs = M.sample_queries(port, name, a.n, boxes, xs, ys)   # on truth: reachable pairs
        w = World(port, centers)
        pl = mk_planners(w)
        print(f"\n{name}  {len(qs)} missions", flush=True)
        for mi, (sx, sy, gx, gy, d) in enumerate(qs):
            line = []
            for k, nm in enumerate(names):
                nm = names[(k + mi) % len(names)]               # rotate order
                r = fly(w, pl[nm], sx, sy, gx, gy, use_watchdog=nm in ('ahppa', 'v2'))
                wr.writerow(dict(world=name, family=fam, indoor=int(indoor), mid=mi, planner=nm, **r))
                line.append(f"{nm} {r['outcome'][:4]} {r['time_s']:5.1f}s")
            f.flush()
            print(f"  m{mi:02d} {d:5.1f} m  " + " | ".join(line), flush=True)
        print(f"  world done in {time.time() - t0:.0f}s", flush=True)
    f.close()


if __name__ == '__main__':
    main()
