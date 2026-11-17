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
import os
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

# -- vehicle-level state, deliberately not per planner instance ------------
FEATURE = ClearanceEstimator()
DISPATCH = HysteresisDispatcher()
# 30 samples at 0.2 s = a 6 s window of SIM time. The navigator's control
# loop runs at 20 Hz; feeding every tick would make the window 1.5 s, short
# enough to trip during an ordinary hover-turn at a waypoint.
WATCHDOG = DeadlockWatchdog(samples=30, min_displacement=0.30, rearm_s=5.0)
WATCHDOG_PERIOD_S = 0.2
CLOCK = time.monotonic
CLOCK_IS_SIM = False
STATS = {OPEN: 0, MEDIUM: 0, DENSE: 0, ESCAPE: 0}

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
        self.last_mode = None

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

    def plan(self, sx, sy, gx, gy, start_layer=None, goal_layer=None):
        if not enabled():
            return super().plan(sx, sy, gx, gy, start_layer, goal_layer)
        L = len(Z_LAYERS)
        sl = 1 if start_layer is None else int(start_layer)
        sl = max(0, min(L - 1, sl))
        gl = sl if goal_layer is None else int(goal_layer)
        gl = max(0, min(L - 1, gl))
        deadlock = bool(WATCHDOG.tripped)
        try:
            mode, tight = self._mode_for(sx, sy, sl)
        except Exception:
            mode, tight = MEDIUM, 0.0
        self.last_mode = mode
        STATS[mode] = STATS.get(mode, 0) + 1

        start = (sx, sy, Z_LAYERS[sl])
        goal = (gx, gy, Z_LAYERS[gl])
        t0 = time.perf_counter()
        p, used, err = None, mode, ''
        try:
            if mode == OPEN:
                p = self._lt.plan(self._grids, start, goal)
            elif mode in (DENSE, ESCAPE):
                p = self._ca.plan(self._grids, start, goal)
        except Exception as e:
            err = f"{type(e).__name__}: {e}"
            print(f"    [portfolio] {mode} failed ({err}), "
                  f"falling back to grid A*")
            p = None
        else:
            if p:
                print(f"    [portfolio] {mode}  clearance "
                      f"{FEATURE.last_clearance:.2f} m  tight {tight:.2f}  "
                      f"{len(p)} waypoints   {dict(STATS)}")
            elif mode != MEDIUM:
                # Never fall through silently: without this the log shows only
                # the stock planner's message and the portfolio looks inactive.
                print(f"    [portfolio] {mode} returned NO PATH "
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
                  error=err))
        return p
