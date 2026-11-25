#!/usr/bin/env python3
"""Stuck-scenario benchmark: obstacles the map misses.

WHY
    The 2,200-mission run (benchmark/mission_sim.py) never tripped the
    deadlock watchdog: there every obstacle is eventually sensed, so a
    kinematic drone always keeps moving. Real deadlocks come from obstacles
    the map does NOT hold. Flight test 2026-11-17 (docs/AHPPA_flight_test_report.pdf,
    run 3): the drone was pinned against 5 cm railing posts the OctoMap had
    lost; the watchdog fired after 5.8 s, ESCAPE returned the same route, the
    drone slid along the posts and flipped.

MODEL (everything else exactly as mission_sim.py)
    ghost obstacles  physically present, but the sensor sees each one only
                     with probability p per sweep, and only within
                     detect_range (glass: p = 0, never). Once seen, a ghost is
                     mapped like any obstacle.
    pinned           a motion step that would bring the drone centre within
                     HIT_R of a ghost does not happen: the drone stays where
                     it is (it is pressed against the obstacle). Seconds spent
                     like this = contact_s, the crash risk the flight showed.
    watchdog         the real DeadlockWatchdog, for EVERY planner. A trip
                     forces one replan (stock / ps / lt: same planner;
                     ahppa: shipped ESCAPE; v2: clearance A* + shortcut).
    bumped (--bump)  PX4 glass flights (2026-11-19, 20 flights): contact is
                     not a pin but a violent bump - 11-45 deg of tilt, the
                     drone thrown back ~0.5 m and down 0.3-1.7 m - and it
                     ended the flight on about half of all contacts (16 crashes
                     in 33 contacts). With --bump a contact throws the drone
                     BOUNCE_M back along its motion and costs SETTLE_S; a
                     planner with impact detection (v3i, v31) then writes the
                     barrier and holds RECOVER_S before an ESCAPE replan, as
                     the navigator does. No crash is drawn: the analysis
                     scores each mission (1 - p_crash) ** contacts, which
                     has far less variance than sampling crashes.
    v3               v2 + STUCK MEMORY: on a trip, write a virtual barrier
                     into the map across the direction the drone was trying
                     to go (BAR_W wide, BAR_D ahead), then replan in ESCAPE.
                     Memory lasts for the mission.

SCENARIOS (indoor, ceiling 2.8 m, 0.2 m walls; ghosts floor to ceiling)
    railing     a line of 5 cm posts across the room, p = 0.02 within 1.5 m
    glass       a glass pane across the room, never seen
    glass_corr  two corridors round a block; the short one has a glass door
    netting     two doorways; the near one is covered by netting
    zigzag      two glass panes in series (needs two escapes)
    visible     the railing, but always seen (control: nobody should stall)

  python3 benchmark/stuck_sim.py [--n 10] [--only railing] [--planners ...]
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
import mission_sim as S                                         # noqa: E402

M, P = S.M, S.P
from dispatcher import HysteresisDispatcher                     # noqa: E402

BAR_W = 1.2          # m: width of the virtual barrier v3 writes on a trip
BAR_D = 0.30         # m: how far ahead of the drone it is placed
FAST_S = 1.5         # s: v3f fast trigger - commanded forward but moved
FAST_MOVE = 0.10     #    less than FAST_MOVE m in FAST_S
BOUNCE_M = 0.5       # --bump: thrown back this far (PX4: 0.38-1.06 m)
SETTLE_S = 1.0       #   time lost per bump without impact detection
RECOVER_S = 5.0      #   v3i/v31: back-off + steady hold (PX4: 3.7-10 s, median ~5.5)
WALL = 0.2
CEIL = 2.8


# ── scenarios ───────────────────────────────────────────────────────────────
def room(x1, y1):
    return [[0, x1, 0, WALL, 0, CEIL], [0, x1, y1 - WALL, y1, 0, CEIL],
            [0, WALL, 0, y1, 0, CEIL], [x1 - WALL, x1, 0, y1, 0, CEIL]]


def posts(x, y0, y1, step, d=0.05):
    return [[x - d / 2, x + d / 2, y - d / 2, y + d / 2, 0, CEIL] for y in np.arange(y0, y1, step)]


def pane(x, y0, y1, t=0.02):
    return [[x - t / 2, x + t / 2, y0, y1, 0, CEIL]]


def scenarios():
    """name -> (solid boxes, [ghost groups: (boxes, p, range, per_box)], start box, goal box)
    per_box: each box is its own object (posts) or the group is one object."""
    R = room(14, 10)
    out = {}
    out['railing'] = (R, [(posts(7.0, 0.3, 7.5, 0.25), 0.02, 1.5, True)], (1.5, 1.0, 3.5, 6.0), (10.5, 1.0, 12.5, 6.0))
    out['glass'] = (R, [(pane(7.0, WALL, 7.5), 0.0, 0.0, False)], (1.5, 1.0, 3.5, 6.0), (10.5, 1.0, 12.5, 6.0))
    block = [[3.0, 11.0, 3.4, 6.6, 0, CEIL]]
    out['glass_corr'] = (R + block, [(pane(7.0, WALL, 3.4), 0.0, 0.0, False)], (1.0, 1.0, 2.5, 2.8), (11.5, 1.0, 13.0, 2.8))
    wall = [[6.9, 7.1, WALL, 1.5, 0, CEIL], [6.9, 7.1, 3.0, 7.0, 0, CEIL], [6.9, 7.1, 8.5, 10, 0, CEIL]]
    out['netting'] = (R + wall, [(posts(7.0, 1.55, 3.0, 0.2, 0.03), 0.02, 1.0, False)], (1.5, 1.0, 4.0, 4.0), (10.0, 1.0, 12.5, 4.0))
    out['zigzag'] = (R, [(pane(5.0, WALL, 6.5), 0.0, 0.0, False), (pane(9.0, 3.5, 10 - WALL), 0.0, 0.0, False)],
                     (1.0, 1.0, 3.0, 3.0), (11.0, 7.0, 13.0, 9.0))
    door = [[6.9, 7.1, WALL, 3.8, 0, CEIL], [6.9, 7.1, 6.2, 10, 0, CEIL]]
    out['post_door'] = (R + door, [(posts(7.0, 5.0, 5.01, 1.0), 0.0, 0.0, False)], (1.5, 4.8, 4.0, 5.2), (10.0, 4.8, 12.5, 5.2))
    out['visible'] = (R, [(posts(7.0, 0.3, 7.5, 0.25), 1.0, 5.0, False)], (1.5, 1.0, 3.5, 6.0), (10.5, 1.0, 12.5, 6.0))
    return out


# ── world with ghosts ───────────────────────────────────────────────────────
class GhostWorld(S.World):
    def __init__(self, port, centers, ghosts, rng):
        super().__init__(port, centers)
        self.rng = rng
        self.objs = []                    # dict(vox, cells per layer, p, range)
        for boxes, p, rng_m, per_box in ghosts:
            groups = [[b] for b in boxes] if per_box else [boxes]
            for g in groups:
                vox = M.shell_voxels(np.array(g, dtype=float))
                cells = []
                for li, zz in enumerate(P.Z_LAYERS):
                    v = vox[np.abs(vox[:, 2] - zz) <= getattr(P, 'DRONE_CLEARANCE', 0.3)]
                    r = np.floor((v[:, 1] - self.oy) / self.res).astype(np.int64)
                    c = np.floor((v[:, 0] - self.ox) / self.res).astype(np.int64)
                    ok = (r >= 0) & (r < self.H) & (c >= 0) & (c < self.W)
                    cells.append((r[ok], c[ok]))
                self.objs.append(dict(vox=vox, cells=cells, p=p, range=rng_m,
                                      xy=vox[:, :2]))
        kh = max(1, int(round(S.HIT_R / self.res)))
        self.ghit = np.zeros_like(self.truth)
        for o in self.objs:
            for li, (r, c) in enumerate(o['cells']):
                for a, b in zip(r, c):
                    self.ghit[li, max(0, a - kh):a + kh + 1, max(0, b - kh):b + kh + 1] = True
        self.reset()

    def reset(self):
        super().reset()
        for o in getattr(self, 'objs', []):
            o['seen'] = False
        self.escape = False
        self.mem = []                     # (barrier cells, contact-point cells)
        self.mem_mode = 'bar'
        self.shrinks = 0
        self.extra_vox = []

    def sense(self, x, y, z=None):
        new = super().sense(x, y, z)
        for o in self.objs:
            if o['seen'] or o['p'] <= 0:
                continue
            d = np.min(np.hypot(o['xy'][:, 0] - x, o['xy'][:, 1] - y))
            if d <= o['range'] and self.rng.random() < o['p']:
                o['seen'] = True
                for li, (r, c) in enumerate(o['cells']):
                    if len(r):
                        new += self._mark(li, r, c)
                self.extra_vox.append(o['vox'])
        if self.extra_vox:
            base = self.port.octomap_3d.voxel_centers
            self.port.octomap_3d.voxel_centers = np.vstack([base] + self.extra_vox).astype(np.float32)
        return new

    def pinned(self, x, y, z):
        li = int(np.argmin([abs(z - zz) for zz in P.Z_LAYERS]))
        r = int((y - self.oy) / self.res)
        c = int((x - self.ox) / self.res)
        return 0 <= r < self.H and 0 <= c < self.W and bool(self.ghit[li, r, c])

    def remember_stuck(self, x, y, hx, hy):
        """v3: virtual barrier across the heading (hx, hy), BAR_D ahead."""
        n = math.hypot(hx, hy)
        if n < 1e-6:
            return 0
        hx, hy = hx / n, hy / n
        cx, cy = x + hx * BAR_D, y + hy * BAR_D
        px, py = -hy, hx
        ts = np.arange(-BAR_W / 2, BAR_W / 2 + 1e-9, self.res / 2)
        r = np.floor((cy + py * ts - self.oy) / self.res).astype(np.int64)
        c = np.floor((cx + px * ts - self.ox) / self.res).astype(np.int64)
        ok = (r >= 0) & (r < self.H) & (c >= 0) & (c < self.W)
        r, c = r[ok], c[ok]
        dr = np.array([int((cy - self.oy) / self.res)])
        dc = np.array([int((cx - self.ox) / self.res)])
        self.mem.append(((r, c), (dr, dc)))
        if self.mem_mode == 'bar':
            self._mark_virtual(r, c)
        elif self.mem_mode == 'dot':
            self._mark_virtual(dr, dc)
        return len(r)

    def _mark_virtual(self, r, c):
        """Write cells into the planner grids only (not into self.seen, so a
        rebuild can take them out again)."""
        k = self.k
        for li in range(self.L):
            for a, b in zip(r, c):
                sl = (slice(max(0, a - k), a + k + 1), slice(max(0, b - k), b + k + 1))
                self.port.layer_grids[li][sl] = 1
                self.port._grids[li][sl] = True

    def shrink_memory(self):
        """Memory blocks every route: bars -> contact points -> nothing.
        Rebuild the grids from what the sensor saw plus the reduced memory.
        Returns False when there is nothing left to shrink."""
        if not self.mem or self.mem_mode == 'none':
            return False
        self.mem_mode = 'dot' if self.mem_mode == 'bar' else 'none'
        self.shrinks += 1
        k = self.k
        for li in range(self.L):
            m = self.seen[li]
            g = np.zeros_like(m)
            rr, cc = np.nonzero(m)
            for a, b in zip(rr, cc):
                g[max(0, a - k):a + k + 1, max(0, b - k):b + k + 1] = True
            self.port._grids[li] = g
            self.port.layer_grids[li] = g.astype(np.uint8)
        if self.mem_mode == 'dot':
            for _, (dr, dc) in self.mem:
                self._mark_virtual(dr, dc)
        return True


def report(*a, **k):
    """portfolio.report_stuck is a no-op unless V3; keep V3 off otherwise so
    the 'ahppa' planner (port.plan) stays the shipped one."""
    P.V3 = True
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            return P.report_stuck(*a, **k)
    finally:
        P.V3 = False


# ── planners ────────────────────────────────────────────────────────────────
def mk_planners(w):
    pl = S.mk_planners(w)
    port, lt = w.port, w.lt
    fl = M.FLY_LAYER

    def v3(x, y, z, gx, gy):
        """v2 planning; ESCAPE also after a fast (v3f) stuck event."""
        while True:
            if P.WATCHDOG.tripped or w.escape:
                port._ca.time_budget_s = 5.0
                p = port._ca.plan(port._grids, (x, y, z), (gx, gy, P.Z_LAYERS[fl]))
                if p:
                    w.escape = False
                    return S.shortcut(lt, port._grids, p), 'ESCAPE'
            p = pl['ps'](x, y, z, gx, gy)[0]
            if p:
                w.escape = False
                return p, 'ps'
            if not w.shrink_memory():          # memory sealed the way: relax it
                return None, 'ps'

    pl['v3'] = pl['v3f'] = v3

    def real_memory(fuse):
        """The SHIPPED v3 memory: portfolio.MEMORY / report_stuck /
        memory_segments / ladder / _with_memory, around v2 planning.
        fuse False = v3 as flown on 2026-11-19, True = v3.1."""
        def f(x, y, z, gx, gy):
            P.FUSE = fuse
            esc = P.WATCHDOG.tripped or w.escape
            lad = P.ladder()
            for kind in lad:
                saved = port._with_memory(kind) if kind != 'none' else None
                p, mode = None, 'ps'
                try:
                    if esc:
                        port._ca.time_budget_s = 5.0
                        q = port._ca.plan(port._grids, (x, y, z), (gx, gy, P.Z_LAYERS[fl]))
                        if q:
                            p, mode = S.shortcut(lt, port._grids, q), 'ESCAPE'
                    if not p:
                        p = pl['ps'](x, y, z, gx, gy)[0]
                finally:
                    if saved is not None:
                        port.layer_grids, port._grids = saved
                if p:
                    w.escape = False
                    if kind != lad[0]:
                        w.shrinks += 1
                        mode += ':' + kind
                    return p, mode
            return None, 'ps'
        return f

    pl['v3i'] = real_memory(False)
    pl['v31'] = real_memory(True)
    return pl


def fly(w, plan_fn, sx, sy, gx, gy, memory, fast=False, real=False, bump=False):
    z0 = P.Z_LAYERS[M.FLY_LAYER]
    x, y, z = sx, sy, z0
    t_ref = [0.0]
    P.set_clock(lambda: t_ref[0])
    P.DISPATCH = HysteresisDispatcher()
    P.WATCHDOG.reset()
    P.WATCHDOG._last_trip_t = -1e9
    P._seen_trips = P.WATCHDOG.trips
    P._last_obs_t = None
    w.reset()
    P.clear_memory()
    w.sense(x, y, z)
    t = 0.0
    limit = 60.0 + 6.0 * math.hypot(gx - sx, gy - sy) / S.V
    plan_s, flown, replans, trips, contact, modes = 0.0, 0.0, 0, 0, 0.0, {}
    path, idx, need = None, 1, True
    next_sense, next_check, fresh = S.SENSE_DT, S.CHECK_DT, False
    outcome, first_contact, first_trip = 'timeout', None, None
    hist, events = [], 0                 # v3f: positions over the last FAST_S of flight
    bumps = 0
    while t < limit:
        if math.hypot(gx - x, gy - y) < S.GOAL_R:
            outcome = 'success'
            break
        if need:
            t0 = time.perf_counter()
            with contextlib.redirect_stdout(io.StringIO()):
                p, mode = plan_fn(x, y, z, gx, gy)
            dt = time.perf_counter() - t0
            plan_s += dt
            t += dt
            t_ref[0] = t
            replans += 1
            modes[mode] = modes.get(mode, 0) + 1
            if not p:
                outcome = 'no_path'
                break
            path, need = list(p), False
            hist = []
            idx = 1 if math.dist(path[0], (x, y, z)) < 1e-3 else 0
            if len(path) == 1:
                path = [path[0], (gx, gy, path[0][2])]
        rem = S.V * S.DT
        stuck_now = bumped = False
        while rem > 1e-9 and idx < len(path):
            tx, ty, tz = path[idx]
            d = math.dist((x, y, z), (tx, ty, tz))
            step = min(d, rem)
            f = step / d if d > 0 else 1.0
            nx, ny, nz = x + (tx - x) * f, y + (ty - y) * f, z + (tz - z) * f
            if w.pinned(nx, ny, nz):
                if bump:
                    bumped = (nx - x, ny - y)
                else:
                    stuck_now = True                 # pressed against a ghost
                break
            x, y, z = nx, ny, nz
            flown += step
            rem -= step
            if step >= d - 1e-12:
                idx += 1
        if bumped:
            bumps += 1
            if first_contact is None:
                first_contact = t
            n = math.hypot(*bumped) or 1.0
            ux, uy = bumped[0] / n, bumped[1] / n
            if real:                                 # impact detected: remember, back off, hold
                report(x, y, ux, uy, source='impact')
                if first_trip is None:
                    first_trip = t
                events += 1
            bx, by = x - ux * BOUNCE_M, y - uy * BOUNCE_M
            if not w.collided(bx, by, z):
                x, y = bx, by
            hold = RECOVER_S if real else SETTLE_S
            if real:
                w.escape, need, hist = True, True, []
                P.WATCHDOG.reset()                   # the hold is a commanded hover
                P._seen_trips = P.WATCHDOG.trips
            t += hold
            t_ref[0] = t
        if stuck_now:
            contact += S.DT
            if first_contact is None:
                first_contact = t
        t += S.DT
        t_ref[0] = t
        if w.collided(x, y, z):
            outcome = 'collision'
            break
        if idx >= len(path):
            need = True
        if t >= next_sense:
            next_sense = t + S.SENSE_DT
            fresh |= bool(w.sense(x, y, z))
        if fresh and t >= next_check:
            next_check, fresh = t + S.CHECK_DT, False
            pts, acc = [(x, y, z)], 0.0
            for q in path[idx:]:
                acc += math.dist(pts[-1], q)
                pts.append(q)
                if acc > S.RANGE + 1.0:
                    break
            if any(w.blocked(a, b) for a, b in zip(pts, pts[1:])):
                need = True
        if fast and idx < len(path):
            hist.append((x, y, z))
            n = int(round(FAST_S / S.DT))
            if len(hist) > n:
                hist.pop(0)
            if len(hist) == n and math.dist(hist[0], hist[-1]) < FAST_MOVE:
                events += 1                          # commanded forward, not moving
                if first_trip is None:
                    first_trip = t
                tx, ty, _ = path[idx]
                if real:
                    report(x, y, tx - x, ty - y)
                else:
                    w.remember_stuck(x, y, tx - x, ty - y)
                w.escape, need, hist = True, True, []
        P.observe(x, y, z, True)
        if P.deadlock_edge():
            trips += 1
            if first_trip is None:
                first_trip = t
            if memory and idx < len(path):
                tx, ty, _ = path[idx]
                if real:
                    report(x, y, tx - x, ty - y, source='watchdog')
                else:
                    w.remember_stuck(x, y, tx - x, ty - y)
            need = True
    return dict(outcome=outcome, time_s=round(t, 2), plan_s=round(plan_s, 3), flown=round(flown, 2),
                replans=replans, trips=trips, fast_events=events, bumps=bumps, mem_shrinks=w.shrinks, contact_s=round(contact, 1),
                first_contact=('' if first_contact is None else round(first_contact, 1)),
                first_trip=('' if first_trip is None else round(first_trip, 1)),
                straight=round(math.hypot(gx - sx, gy - sy), 2),
                modes=' '.join(f'{k}:{v}' for k, v in sorted(modes.items())))


def build(solid, ghosts, with_ghosts):
    boxes = np.array(solid + ([b for g in ghosts for b in g[0]] if with_ghosts else []), dtype=float)
    boxes, xs, ys = M.prepare(boxes, True)
    centers = M.shell_voxels(boxes)
    grid, octo = M.make_inputs(centers, xs, ys)
    with contextlib.redirect_stdout(io.StringIO()):
        port = P.PortfolioPlanner(grid, octo, safety_margin=M.SAFETY_MARGIN,
                                  unknown_penalty=M.UNKNOWN_PENALTY)
    return port, centers


def ghost_worlds(a):
    for name, (solid, ghosts, sb, gb) in scenarios().items():
        if a.only and name not in a.only.split(','):
            continue
        truth_port, _ = build(solid, ghosts, True)        # reachability on the TRUE map
        port, centers = build(solid, ghosts, False)
        qr = np.random.default_rng(a.seed + sum(map(ord, name)))
        qs = []
        while len(qs) < a.n:
            sx, sy = qr.uniform(sb[0], sb[2]), qr.uniform(sb[1], sb[3])
            gx, gy = qr.uniform(gb[0], gb[2]), qr.uniform(gb[1], gb[3])
            with contextlib.redirect_stdout(io.StringIO()):
                o = M.MultiLayerPlanner.plan(truth_port, sx, sy, gx, gy, start_layer=M.FLY_LAYER,
                                             goal_layer=M.FLY_LAYER)
            if o:
                qs.append((sx, sy, gx, gy, M.polyline_len(o)))
        yield name, port, centers, ghosts, qs


def normal_worlds(a):
    """Same worlds and missions as mission_sim.py (same sampler and seed)."""
    for name, fam, path, indoor, src in M.world_list(a.only):
        upright, tilted = M.load_geometry(path)
        boxes, xs, ys = M.prepare(upright, indoor)
        centers = M.shell_voxels(boxes, tilted)
        grid, octo = M.make_inputs(centers, xs, ys)
        with contextlib.redirect_stdout(io.StringIO()):
            port = P.PortfolioPlanner(grid, octo, safety_margin=M.SAFETY_MARGIN,
                                      unknown_penalty=M.UNKNOWN_PENALTY)
        qs = M.sample_queries(port, name, a.n, boxes, xs, ys)
        yield name, port, centers, [], qs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--n', type=int, default=10)
    ap.add_argument('--only', default='')
    ap.add_argument('--planners', default='stock,ps,lt,ahppa,v2,v3,v3f')
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--bar-w', type=float, default=None, help='override BAR_W (m)')
    ap.add_argument('--bump', action='store_true', help='contacts bounce (PX4) instead of pinning')
    ap.add_argument('--normal', action='store_true',
                    help='the 44 ordinary layouts, no ghosts (false-trigger regression)')
    ap.add_argument('--out', default=os.path.join(M.REPO, 'benchmark', 'stuck_results.csv'))
    a = ap.parse_args()
    global BAR_W
    if a.bar_w is not None:
        BAR_W = a.bar_w
        P.BAR_W = a.bar_w
    names = a.planners.split(',')
    fields = ['scenario', 'mid', 'planner', 'outcome', 'time_s', 'plan_s', 'flown', 'oracle', 'straight',
              'replans', 'trips', 'fast_events', 'bumps', 'mem_shrinks', 'contact_s', 'first_contact', 'first_trip', 'modes']
    f = open(a.out, 'w', newline='')
    wr = csv.DictWriter(f, fieldnames=fields)
    wr.writeheader()
    for name, port, centers, ghosts, qs in (normal_worlds(a) if a.normal else ghost_worlds(a)):
        print(f"\n{name}: {len(qs)} missions", flush=True)
        w = GhostWorld(port, centers, ghosts, None)
        pl = mk_planners(w)
        for mi, (sx, sy, gx, gy, orc) in enumerate(qs):
            line = []
            for k in range(len(names)):
                nm = names[(k + mi) % len(names)]
                w.rng = np.random.default_rng(1000 * mi + 7)     # same ghost luck for every planner
                real = nm in ('v3i', 'v31')
                r = fly(w, pl[nm], sx, sy, gx, gy, memory=nm in ('v3', 'v3f') or real,
                        fast=(nm == 'v3f') or (real and not a.bump), real=real, bump=a.bump)
                wr.writerow(dict(scenario=name, mid=mi, planner=nm, oracle=round(orc, 2), **r))
                line.append(f"{nm} {r['outcome'][:4]} {r['time_s']:5.1f}s c{r['contact_s']:.0f} b{r['bumps']} t{r['trips']}+{r['fast_events']}")
            f.flush()
            print(f"  m{mi:02d} " + " | ".join(line), flush=True)
    f.close()


if __name__ == '__main__':
    main()
