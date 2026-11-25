#!/usr/bin/env python3
"""PortfolioPlanner - AHPPA as a drop-in subclass of MultiLayerPlanner.

WHY A SUBCLASS AND NOT A WRAPPER
    navigator_astar builds the planner in two places with multi-line
    constructor calls and calls .plan() from four. A subclass with the same
    constructor signature means NOTHING in the navigator changes except one
    import line - no edits to call sites, no edits to constructor calls.
    Turn it on and off with an environment variable, so A/B testing is a
    shell prefix rather than a re-patch.

WHY THE DISPATCHER IS MODULE-LEVEL
    MultiLayerPlanner is rebuilt on every replan. A dispatcher owned by the
    instance would be reconstructed with it, resetting its hysteresis and
    dwell every single time, so the mode would always start at OPEN and the
    hysteresis would never do anything. Mode is a property of the vehicle
    over time, so it lives here and survives planner rebuilds.

CLOCK
    Dwell timing defaults to time.monotonic(), which is WALL clock. The
    simulation runs at ~0.63 real-time factor, so a 0.5 s dwell becomes
    ~0.8 s of sim time. Install sim time with set_clock() from the node to
    make it exact - mixing wall and sim time has caused three separate bugs
    in this project already.

DEADLOCK -> ESCAPE (wired 2026-11-17)
    Before this, nothing in the live stack ever passed deadlock= to the
    dispatcher, so ESCAPE was reachable only in the offline selftest. Now the
    navigator calls observe() every control tick; it feeds a module-level
    DeadlockWatchdog on the installed (sim) clock. While the watchdog is
    tripped, every plan() is dispatched in ESCAPE. The navigator triggers one
    replan on each new trip (see deadlock_edge()).

LOG
    Every portfolio plan() appends a row to logs/ahppa/portfolio_<stamp>.csv
    in the repo - NOT /tmp, which the VM wipes on reboot (the October flight
    CSVs were lost that way). Override the directory with AHPPA_LOG_DIR.

USE
    export AHPPA_PORTFOLIO=1     # off unless set
"""
import csv
import datetime
import gc
import math
import os
import pickle
import select
import signal
import sys
import time

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(os.path.dirname(_HERE))
sys.path.insert(0, _HERE)
sys.path.insert(0, REPO_ROOT)

from planners.planner_3d import MultiLayerPlanner, Z_LAYERS   # noqa: E402
from clearance_feature import ClearanceEstimator              # noqa: E402
from dispatcher import (HysteresisDispatcher, OPEN, MEDIUM,   # noqa: E402
                        DENSE, ESCAPE)
from lazy_theta import LazyThetaStar                          # noqa: E402
from clearance_astar import ClearanceAStar                    # noqa: E402
from watchdog import DeadlockWatchdog                         # noqa: E402
from refine import shortcut, refine                            # noqa: E402

# -- vehicle-level state, deliberately not per planner instance ------------
FEATURE = ClearanceEstimator()
DISPATCH = HysteresisDispatcher()
# 30 samples at 0.2 s = a 6 s window of SIM time. The navigator's control
# loop runs at 20 Hz; feeding every tick would make the window 1.5 s, short
# enough to trip during an ordinary hover-turn at a waypoint.
WATCHDOG = DeadlockWatchdog(samples=30, min_displacement=0.30, rearm_s=5.0)
WATCHDOG_PERIOD_S = 0.2
# Wall-clock budget for the OPEN / DENSE / ESCAPE member on each plan(); past
# it the member gives up and the plan falls back to stock A*. Without it Lazy
# Theta* searched 17-23 s before failing on 3 of 1,320 benchmark queries, and
# the fallback then ran past the 30 s cap (the drone would hover all that
# time). 5 s keeps 93% of Lazy Theta*'s successful plans; the 6.6% it cuts
# fall back to stock A*, ~3% longer. AHPPA_MEMBER_BUDGET_S=0 disables it.
MEMBER_BUDGET_S = float(os.environ.get('AHPPA_MEMBER_BUDGET_S', '5.0'))

# RACE (AHPPA_RACE=1, off by default until flight-tested)
#   Choosing a member per query from features known before planning never beat
#   the best single member on the 44-layout benchmark (tradeoff_headroom.py):
#   which planner is fastest depends on the route, and the route is unknown
#   until someone plans it. So don't predict - run both. Lazy Theta* and stock
#   A* are forked into two child processes (they inherit the map, nothing is
#   copied; the GIL rules out threads). If Lazy Theta* finishes first its path
#   is taken and stock A* is killed; if stock finishes first at t_s, Lazy
#   Theta* gets until (1 + RACE_WAIT) * t_s, else stock's path is used.
#   RACE_WAIT trades time for length: 0 = never slower than stock (simulated
#   -1.8% length, median 199 vs 298 ms); 2 = most of Lazy Theta*'s -4%.
#   Deadlock (ESCAPE) still goes to clearance A*, sequentially. Any fork or
#   child failure falls back to the sequential path; a child that runs past
#   RACE_HARD_S is killed and stock A* is run in-process.
RACE = os.environ.get('AHPPA_RACE', '0') not in ('0', '', 'false')
RACE_WAIT = float(os.environ.get('AHPPA_RACE_WAIT', '0.0'))
RACE_HARD_S = float(os.environ.get('AHPPA_RACE_HARD_S', '30.0'))
RACE_MODE = 'RACE'
# Benchmark only: wait for BOTH children and record both finish times, so any
# RACE_WAIT can be evaluated afterwards from one run. Never set in flight.
RACE_OBSERVE = os.environ.get('AHPPA_RACE_OBSERVE', '0') not in ('0', '', 'false')
# AHPPA v2 (AHPPA_V2=1, off by default until flight-tested)
#   The 44-layout benchmark (analyse_v2.py) showed the clearance dispatcher
#   never beat a single planner, and that line-of-sight smoothing of the A*
#   path recovers ~90% of Lazy Theta*'s length saving for ~2% more time, while
#   Lazy Theta* restricted to a 1 m corridor around that smoothed path returns
#   Lazy Theta*'s own path on ~99% of queries. So v2 is:
#     normal     stock A* -> shortcut()          (hover ~ stock A* time)
#                + corridor Lazy Theta* refine() if AHPPA_V2_REFINE=sync
#     deadlock   clearance A* -> shortcut()      (ESCAPE, unchanged trigger)
#   with stock A* as the fallback for any failure.
V2 = os.environ.get('AHPPA_V2', '0') not in ('0', '', 'false')
V2_REFINE = os.environ.get('AHPPA_V2_REFINE', 'off')       # off | sync
V2_MODE = 'V2'
# AHPPA v3 (AHPPA_V3=1, implies V2; off by default until flight-tested)
#   Real deadlocks come from obstacles the MAP DOES NOT HOLD (flight test
#   2026-11-17, run 3: pinned against railing posts the OctoMap had lost;
#   ESCAPE replanned the same route). Every planner replans through such an
#   obstacle forever, because to the map it is free space. v3 adds STUCK
#   MEMORY: when the drone is stuck - the watchdog trips, or the navigator
#   reports "commanded forward but not moving" (report_stuck) - a virtual
#   barrier BAR_W wide is written BAR_D ahead of the drone, across the
#   direction it was trying to go, into every planner built afterwards, and
#   the next plan is made in ESCAPE. If the memory blocks every route (a post
#   in the middle of the only doorway), the plan is retried with each barrier
#   cut to the single contact point, then without memory.
#   stuck_sim.py (7 unseen-obstacle maps x 30 missions): 180/180 missions vs
#   60/180 for every other planner, contact time -94%; no false triggers on
#   the 44 ordinary layouts.
V3 = os.environ.get('AHPPA_V3', '0') not in ('0', '', 'false')
V2 = V2 or V3
# 4 m: a 'wall hypothesis' - the obstacles maps miss indoors (glass walls,
# railings, netting) are long and straight. stuck_sim.py sensitivity (6 maps
# x 30): stuck events per mission 1.2 m 3.41, 2 m 2.88, 4 m 2.75, 8 m 3.18
# (8 m seals real passages: 76 relaxations). In the PX4 glass flights every
# crash came from a 2nd contact just past the end of a 1.2 m barrier.
BAR_W = float(os.environ.get('AHPPA_V3_BAR_W', '4.0'))
BAR_D = 0.30
# Extra inflation around remembered barriers: where exactly the unseen
# surface is, is uncertain, and in the glass flights the rotors touched with
# the drone centre 0.30-0.39 m from the pane - inside the 0.40 m margin's
# tracking-error budget. Applies to memory only, not to sensed obstacles.
MEMORY_EXTRA_M = float(os.environ.get('AHPPA_V3_MEMORY_EXTRA_M', '0.3'))
# v3.1 SURFACE FUSION (AHPPA_V3_FUSE=1, default on): PX4 glass flights
# 2026-11-19 (20-flight batch): a barrier is drawn across the HEADING, so a
# drone that hits a flat pane at an angle gets a barrier tilted against the
# pane; the next route slipped past its tip and hit the same pane again
# (three contacts on one pane in 4 of 10 v3 flights; each contact crashed the
# drone with p ~ 0.5). Contacts are points ON the unseen surface: barriers
# whose centres lie within FUSE_D of each other on one line (residual <=
# FUSE_TOL) and whose headings run INTO that line (|heading . line| <=
# FUSE_COS) and agree with each other (FUSE_AGREE) are merged into one segment along the fitted line, spanning the
# contacts plus BAR_W / 2 at each end (the wall hypothesis, now pointed the
# right way).
#   And when memory seals every route the ladder is bar -> HULL (the measured
#   span of the contacts + HULL_EXT, which is known glass) -> dot -> none,
#   instead of dropping straight to single points: in 2 of 10 flights the
#   'dot' fallback sent the drone back into the pane it had already hit.
FUSE = os.environ.get('AHPPA_V3_FUSE', '1') not in ('0', '', 'false')
FUSE_D = 5.0
FUSE_TOL = 0.4
FUSE_COS = 0.8
FUSE_AGREE = 0.5                    # headings within 60 deg: same face, not an L corner
HULL_EXT = 0.5
MEMORY_MAX = 30                     # oldest barriers are dropped first
MEMORY = []                         # dicts: cx, cy, hx, hy (unit heading), t
_escape_request = False
CLOCK = time.monotonic
CLOCK_IS_SIM = False
STATS = {OPEN: 0, MEDIUM: 0, DENSE: 0, ESCAPE: 0, RACE_MODE: 0, V2_MODE: 0}

_last_obs_t = None
_seen_trips = 0
_log_path = None


def set_clock(fn):
    """Install the node's sim-time source: set_clock(lambda: node.get_clock()
    .now().nanoseconds / 1e9)."""
    global CLOCK, CLOCK_IS_SIM
    CLOCK = fn
    CLOCK_IS_SIM = True


def enabled():
    return os.environ.get('AHPPA_PORTFOLIO', '0') not in ('0', '', 'false')


def observe(x, y, z, goal_active):
    """Feed one pose to the deadlock watchdog. Call every control tick; it
    down-samples to WATCHDOG_PERIOD_S itself. Returns True while deadlocked."""
    global _last_obs_t
    t = CLOCK()
    if not goal_active:
        WATCHDOG.update(x, y, z, t, goal_active=False)
        _last_obs_t = None
        return False
    # 1 ms slack: four 50 ms ticks sum to 0.19999... in floating point, which
    # would otherwise stretch every sample to five ticks and the window to 7.5 s
    if _last_obs_t is not None and t - _last_obs_t < WATCHDOG_PERIOD_S - 1e-3:
        return WATCHDOG.tripped
    _last_obs_t = t
    return WATCHDOG.update(x, y, z, t, goal_active=True)


def deadlock_edge():
    """True exactly once per new watchdog trip, so the caller replans once
    per deadlock rather than every tick while it lasts."""
    global _seen_trips
    if WATCHDOG.trips > _seen_trips:
        _seen_trips = WATCHDOG.trips
        return True
    return False


def report_stuck(x, y, hx, hy, source='fast'):
    """v3: the drone is stuck at (x, y) while trying to move along (hx, hy).
    Remember a barrier there and make the next plan an ESCAPE plan. No-op
    unless AHPPA_V3=1. Returns True if a barrier was stored."""
    global _escape_request
    if not V3:
        return False
    n = (hx * hx + hy * hy) ** 0.5
    if n < 1e-6:
        return False
    MEMORY.append(dict(cx=x + hx / n * BAR_D, cy=y + hy / n * BAR_D,
                       hx=hx / n, hy=hy / n, t=CLOCK(), src=source))
    del MEMORY[:-MEMORY_MAX]
    _escape_request = True
    print(f"  🧱 [portfolio] stuck ({source}) at ({x:.2f}, {y:.2f}) heading "
          f"({hx / n:+.2f}, {hy / n:+.2f}) -> barrier #{len(MEMORY)}, next plan in ESCAPE")
    return True


def _clusters(mem):
    """Greedy grouping of barriers into straight surfaces (see FUSE)."""
    cl = []
    for i, m in enumerate(mem):
        for c in cl:
            pts = np.array([[mem[j]['cx'], mem[j]['cy']] for j in c] + [[m['cx'], m['cy']]])
            if min(np.hypot(*(pts[:-1] - pts[-1]).T)) > FUSE_D:
                continue
            ctr = pts.mean(0)
            d = np.linalg.svd(pts - ctr)[2][0]
            res = np.abs((pts - ctr) @ np.array([-d[1], d[0]]))
            heads = [(mem[j]['hx'], mem[j]['hy']) for j in c] + [(m['hx'], m['hy'])]
            agree = all(hx * m['hx'] + hy * m['hy'] >= FUSE_AGREE for hx, hy in heads[:-1])
            if agree and res.max() <= FUSE_TOL and all(abs(hx * d[0] + hy * d[1]) <= FUSE_COS for hx, hy in heads):
                c.append(i)
                break
        else:
            cl.append([i])
    return cl


def memory_segments(mem=None, kind='bar', fuse=None):
    """Remembered barriers as segments (x0, y0, x1, y1). kind: 'bar' (wall
    hypothesis), 'hull' (measured span only), 'dot' (contact points)."""
    mem = MEMORY if mem is None else mem
    fuse = FUSE if fuse is None else fuse
    if kind == 'dot':
        return [(m['cx'], m['cy'], m['cx'], m['cy']) for m in mem]
    half = BAR_W / 2 if kind == 'bar' else HULL_EXT
    segs = []
    for c in (_clusters(mem) if fuse else [[i] for i in range(len(mem))]):
        if len(c) == 1:
            m = mem[c[0]]
            px, py = -m['hy'], m['hx']                      # across the heading
            segs.append((m['cx'] - px * half, m['cy'] - py * half,
                         m['cx'] + px * half, m['cy'] + py * half))
            continue
        pts = np.array([[mem[j]['cx'], mem[j]['cy']] for j in c])
        ctr = pts.mean(0)
        d = np.linalg.svd(pts - ctr)[2][0]
        t = (pts - ctr) @ d
        a, b = ctr + d * (t.min() - half), ctr + d * (t.max() + half)
        segs.append((a[0], a[1], b[0], b[1]))
    return segs


def ladder():
    """Memory levels tried in turn when memory seals every route."""
    if not MEMORY:
        return ('none',)
    return ('bar', 'hull', 'dot', 'none') if FUSE else ('bar', 'dot', 'none')


def clear_memory():
    global _escape_request
    MEMORY.clear()
    _escape_request = False


def _log(row):
    """Append one plan record to the persistent CSV. Never raises."""
    global _log_path
    try:
        if _log_path is None:
            d = os.environ.get('AHPPA_LOG_DIR',
                               os.path.join(REPO_ROOT, 'logs', 'ahppa'))
            os.makedirs(d, exist_ok=True)
            stamp = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
            _log_path = os.path.join(d, f'portfolio_{stamp}.csv')
            with open(_log_path, 'w', newline='') as f:
                csv.writer(f).writerow(list(row))
            print(f"  [portfolio] logging plans -> {_log_path}")
        with open(_log_path, 'a', newline='') as f:
            csv.writer(f).writerow(list(row.values()))
    except Exception as e:                          # logging must never
        print(f"  [portfolio] log write failed: {e}")  # take down a plan


class PortfolioPlanner(MultiLayerPlanner):
    def __init__(self, occupancy_grid_msg, octomap_3d, safety_margin=0.25,
                 unknown_penalty=3.0):
        super().__init__(occupancy_grid_msg, octomap_3d,
                         safety_margin=safety_margin,
                         unknown_penalty=unknown_penalty)
        kw = dict(resolution=self.resolution, origin_x=self.origin_x,
                  origin_y=self.origin_y)
        self._grids = np.asarray(self.layer_grids, dtype=bool)
        self._lt = LazyThetaStar(list(Z_LAYERS), **kw)
        self._ca = ClearanceAStar(list(Z_LAYERS), d_safe=0.6, weight=3.0, **kw)
        self._lt.time_budget_s = self._ca.time_budget_s = MEMBER_BUDGET_S or None
        self.last_mode = None
        self.last_gave_up = None
        self.last_race = None
        self.last_memory = None
        self.safety_margin_m = float(safety_margin)

    # -- race ------------------------------------------------------------------
    @staticmethod
    def _spawn(fn):
        """Fork a child that runs fn() and pickles its result down a pipe.
        Returns (pid, read_fd). The child never returns into the caller."""
        r, w = os.pipe()
        pid = os.fork()
        if pid == 0:                                  # child
            code = 1
            try:
                os.close(r)
                gc.disable()                          # short-lived: never collect
                try:
                    signal.signal(signal.SIGALRM, signal.SIG_DFL)
                except Exception:
                    pass
                nul = os.open(os.devnull, os.O_WRONLY)
                os.dup2(nul, 1)                       # planners print; not here
                out = fn()
                data = pickle.dumps(out, protocol=pickle.HIGHEST_PROTOCOL)
                view = memoryview(data)
                while view:
                    view = view[os.write(w, view):]
                code = 0
            except BaseException:
                pass
            finally:
                os._exit(code)                        # no atexit, no ROS teardown
        os.close(w)
        return pid, r

    @staticmethod
    def _collect(fd):
        chunks = []
        while True:
            b = os.read(fd, 1 << 20)
            if not b:
                break
            chunks.append(b)
        try:
            return pickle.loads(b''.join(chunks)), True
        except Exception:
            return None, False                        # child crashed

    def _race(self, sx, sy, gx, gy, start_layer, goal_layer, start, goal):
        """Returns (path, winner, info). winner in {'lt', 'stock', None}."""
        t0 = time.perf_counter()
        lt, base = self._lt, MultiLayerPlanner.plan
        jobs = {'lt': lambda: (lt.plan(self._grids, start, goal), lt.gave_up),
                'stock': lambda: (base(self, sx, sy, gx, gy, start_layer, goal_layer), None)}
        kids = {}                                     # fd -> (name, pid)
        res, t_done = {}, {}
        try:
            # A collection in the child would walk - and so write to, and so
            # copy - every object the parent owns (copy-on-write). freeze()
            # parks them all in a generation the child's GC never visits.
            gc.freeze()
            try:
                for name, fn in jobs.items():
                    pid, fd = self._spawn(fn)
                    kids[fd] = (name, pid)
            finally:
                gc.unfreeze()
            hard = t0 + RACE_HARD_S
            while kids:
                now = time.perf_counter()
                lt_ok = bool(res.get('lt', (None,))[0])
                if RACE_OBSERVE:
                    limit = hard                      # benchmark: time both
                elif lt_ok:
                    break                             # Lazy Theta* path in hand
                elif 'stock' in res and res['stock'][0]:
                    if 'lt' in res:
                        break                         # lt failed, stock is it
                    limit = t0 + (1.0 + RACE_WAIT) * (t_done['stock'] - t0)
                else:
                    limit = hard                      # nothing usable yet
                if now >= limit:
                    break
                ready, _, _ = select.select(list(kids), [], [], limit - now)
                for fd in ready:
                    name, pid = kids.pop(fd)
                    out, ok = self._collect(fd)
                    os.close(fd)
                    os.waitpid(pid, 0)
                    t_done[name] = time.perf_counter()
                    res[name] = out if ok else (None, 'crashed')
        finally:
            for fd, (name, pid) in kids.items():      # losers / stragglers
                try:
                    os.kill(pid, signal.SIGKILL)
                    os.waitpid(pid, 0)
                except OSError:
                    pass
                os.close(fd)
        info = {f't_{k}': round(v - t0, 4) for k, v in t_done.items()}
        info['lt_gave_up'] = (res.get('lt') or (None, None))[1]
        p_lt = (res.get('lt') or (None,))[0]
        p_st = (res.get('stock') or (None,))[0]
        self.last_race = dict(info, p_lt=p_lt, p_stock=p_st)
        # One rule for both modes (live breaks early, so it agrees with this):
        # Lazy Theta* wins if it has a path and stock has none, or it finished
        # by (1 + RACE_WAIT) x stock's time.
        if p_lt and (not p_st or 't_lt' in info and 't_stock' not in info
                     or info.get('t_lt', 1e9) <= (1.0 + RACE_WAIT) * info.get('t_stock', 0.0)):
            return p_lt, 'lt', info
        if p_st:
            return p_st, 'stock', info
        return None, None, info

    def _mode_for(self, sx, sy, sl):
        vox = getattr(self.octomap_3d, 'voxel_centers', None)
        t = FEATURE.tightness(vox, (sx, sy, Z_LAYERS[sl]))
        # now= is a KEYWORD: the real signature is
        #   update(density, deadlock=False, now=None)
        # so passing the clock positionally made every query report a
        # deadlock (any non-zero float is truthy) and forced ESCAPE on
        # all 23 plans of the 2026-10-25 flight. deadlock= is now the
        # watchdog's real state, fed by observe().
        return DISPATCH.update(t, deadlock=WATCHDOG.tripped, now=CLOCK()), t

    def _plan_v2(self, sx, sy, gx, gy, sl, gl, deadlock):
        mode = ESCAPE if deadlock else V2_MODE
        self.last_mode = mode
        self.last_gave_up = None
        self.last_race = None
        STATS[mode] = STATS.get(mode, 0) + 1
        start = (sx, sy, Z_LAYERS[sl])
        goal = (gx, gy, Z_LAYERS[gl])
        t0 = time.perf_counter()
        p, used, err, note = None, mode, '', ''
        try:
            if deadlock:
                q = self._ca.plan(self._grids, start, goal)
                self.last_gave_up = self._ca.gave_up
                if q:
                    p, used = shortcut(self._lt, self._grids, q), 'ESCAPE:ca+ps'
            if not p:
                q = MultiLayerPlanner.plan(self, sx, sy, gx, gy, sl, gl)
                if q:
                    p, used = shortcut(self._lt, self._grids, q), 'A*+ps'
                    if V2_REFINE == 'sync':
                        t1 = time.perf_counter()
                        r = refine(self._lt, self._grids, p, radius=1.0, budget_s=MEMBER_BUDGET_S or 5.0)
                        note = f"refine {1000 * (time.perf_counter() - t1):.0f} ms"
                        if r is not p:
                            p, used = r, 'A*+ps+clt'
        except Exception as e:
            err = f"{type(e).__name__}: {e}"
            print(f"    [portfolio] v2 failed ({err}), falling back to grid A*")
            p = None
        if not p:
            used = f"{mode}->A*"
            p = MultiLayerPlanner.plan(self, sx, sy, gx, gy, sl, gl)
        ms = (time.perf_counter() - t0) * 1000.0
        if p:
            print(f"    [portfolio] {mode} {used}  {len(p)} waypoints  {ms:.0f} ms")
        _log(dict(t=round(CLOCK(), 3), sim_clock=int(CLOCK_IS_SIM), mode=mode,
                  planner=used, deadlock=int(deadlock),
                  clearance='', tightness='',
                  sx=round(sx, 2), sy=round(sy, 2), s_layer=sl,
                  gx=round(gx, 2), gy=round(gy, 2), g_layer=gl,
                  ms=round(ms, 2), n=len(p) if p else 0, ok=int(bool(p)),
                  error=err, note=note))
        return p

    # -- v3: stuck memory -------------------------------------------------------
    def _memory_cells(self, kind):
        """Grid cells of the remembered barriers at memory level `kind`
        (see memory_segments)."""
        rs, cs = [], []
        step = self.resolution / 2
        for x0, y0, x1, y1 in memory_segments(MEMORY, kind):
            n = max(1, int(math.ceil(math.hypot(x1 - x0, y1 - y0) / step)))
            ts = np.linspace(0.0, 1.0, n + 1)
            xs, ys = x0 + (x1 - x0) * ts, y0 + (y1 - y0) * ts
            rs.append(np.floor((ys - self.origin_y) / self.resolution).astype(np.int64))
            cs.append(np.floor((xs - self.origin_x) / self.resolution).astype(np.int64))
        if not rs:
            return np.zeros(0, np.int64), np.zeros(0, np.int64)
        r, c = np.concatenate(rs), np.concatenate(cs)
        ok = (r >= 0) & (r < self.height) & (c >= 0) & (c < self.width)
        return r[ok], c[ok]

    def _with_memory(self, kind):
        """Install grids = sensed map + inflated memory (every layer: an
        unseen obstacle that stopped the drone is assumed floor to ceiling).
        Returns the previous (layer_grids, _grids) to restore."""
        saved = (self.layer_grids, self._grids)
        r, c = self._memory_cells(kind)
        if len(r):
            mask = np.zeros((self.height, self.width), dtype=np.uint8)
            mask[r, c] = 1
            mask = self._inflate(mask, self.safety_margin_m + MEMORY_EXTRA_M).astype(bool)
            self.layer_grids = [np.where(mask, 1, g).astype(np.asarray(g).dtype) for g in saved[0]]
            self._grids = saved[1] | mask[None, :, :]
        return saved

    def _plan_v3(self, sx, sy, gx, gy, sl, gl, deadlock):
        """v2 planning on the map plus stuck memory; ESCAPE after a watchdog
        trip OR a navigator stuck report. Memory that seals every route is
        relaxed: bars -> (v3.1: measured hulls ->) contact points -> none."""
        global _escape_request
        esc = deadlock or _escape_request
        tried = []
        for kind in ladder():
            saved = self._with_memory(kind) if kind != 'none' else None
            try:
                p = self._plan_v2(sx, sy, gx, gy, sl, gl, esc)
            finally:
                if saved is not None:
                    self.layer_grids, self._grids = saved
            tried.append(kind)
            if p:
                break
        if p:
            _escape_request = False
            if len(tried) > 1:
                print(f"    [portfolio] v3: memory sealed every route; planned with memory '{kind}'")
        self.last_memory = kind if p else None
        return p

    def plan(self, sx, sy, gx, gy, start_layer=None, goal_layer=None):
        if not enabled():
            return super().plan(sx, sy, gx, gy, start_layer, goal_layer)
        L = len(Z_LAYERS)
        sl = 1 if start_layer is None else int(start_layer)
        sl = max(0, min(L - 1, sl))
        gl = sl if goal_layer is None else int(goal_layer)
        gl = max(0, min(L - 1, gl))
        deadlock = bool(WATCHDOG.tripped)
        if V3:
            return self._plan_v3(sx, sy, gx, gy, sl, gl, deadlock)
        if V2:
            return self._plan_v2(sx, sy, gx, gy, sl, gl, deadlock)
        if RACE and not deadlock:
            mode, tight = RACE_MODE, 0.0              # no prediction needed
        else:
            try:
                mode, tight = self._mode_for(sx, sy, sl)
            except Exception:
                mode, tight = MEDIUM, 0.0
        self.last_mode = mode
        STATS[mode] = STATS.get(mode, 0) + 1

        start = (sx, sy, Z_LAYERS[sl])
        goal = (gx, gy, Z_LAYERS[gl])
        t0 = time.perf_counter()
        p, used, err, note = None, mode, '', ''
        member = self._lt if mode == OPEN else self._ca if mode in (DENSE, ESCAPE) else None
        self.last_gave_up = None
        self.last_race = None
        try:
            if mode == RACE_MODE:
                p, winner, info = self._race(sx, sy, gx, gy, start_layer, goal_layer,
                                             start, goal)
                used = f"RACE:{winner}" if winner else used
                self.last_gave_up = info.get('lt_gave_up')
                note = ' '.join(f"{k}={v}" for k, v in info.items() if v is not None)
            elif member is not None:
                p = member.plan(self._grids, start, goal)
                self.last_gave_up = member.gave_up
                if member.gave_up:
                    err = f"gave up ({member.gave_up})"
        except Exception as e:
            err = f"{type(e).__name__}: {e}"
            print(f"    [portfolio] {mode} failed ({err}), "
                  f"falling back to grid A*")
            p = None
        else:
            if p and mode == RACE_MODE:
                print(f"    [portfolio] RACE won by {used[5:]}  ({note})  "
                      f"{len(p)} waypoints")
            elif mode == RACE_MODE:
                print(f"    [portfolio] RACE: neither child returned a path ({note}) "
                      f"-> stock A* in-process")
            elif p:
                print(f"    [portfolio] {mode}  clearance "
                      f"{FEATURE.last_clearance:.2f} m  tight {tight:.2f}  "
                      f"{len(p)} waypoints   {dict(STATS)}")
            elif mode != MEDIUM:
                # Never fall through silently: without this the log shows only
                # the stock planner's message and the portfolio looks inactive.
                why = (f"over its {MEMBER_BUDGET_S:g} s budget"
                       if self.last_gave_up == 'budget' else
                       'expansion limit' if self.last_gave_up == 'expansions' else 'NO PATH')
                print(f"    [portfolio] {mode} returned {why} "
                      f"(clearance {FEATURE.last_clearance:.2f} m, "
                      f"tight {tight:.2f}) -> stock A*")
        if not p:
            # MEDIUM is the stock planner, and so is every fallback path
            used = MEDIUM if mode == MEDIUM else f"{mode}->A*"
            p = super().plan(sx, sy, gx, gy, start_layer, goal_layer)
        ms = (time.perf_counter() - t0) * 1000.0

        _log(dict(t=round(CLOCK(), 3), sim_clock=int(CLOCK_IS_SIM), mode=mode,
                  planner=used, deadlock=int(deadlock),
                  clearance=round(float(FEATURE.last_clearance), 3),
                  tightness=round(float(tight), 4),
                  sx=round(sx, 2), sy=round(sy, 2), s_layer=sl,
                  gx=round(gx, 2), gy=round(gy, 2), g_layer=gl,
                  ms=round(ms, 2), n=len(p) if p else 0, ok=int(bool(p)),
                  error=err, note=note))
        return p
