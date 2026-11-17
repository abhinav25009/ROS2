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

USE
    export AHPPA_PORTFOLIO=1     # off unless set
"""
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.expanduser("~/Desktop/Drone_IP"))

from planners.planner_3d import MultiLayerPlanner, Z_LAYERS   # noqa: E402
from clearance_feature import ClearanceEstimator              # noqa: E402
from dispatcher import (HysteresisDispatcher, OPEN, MEDIUM,   # noqa: E402
                        DENSE, ESCAPE)
from lazy_theta import LazyThetaStar                          # noqa: E402
from clearance_astar import ClearanceAStar                    # noqa: E402

# -- vehicle-level state, deliberately not per planner instance ------------
FEATURE = ClearanceEstimator()
DISPATCH = HysteresisDispatcher()
CLOCK = time.monotonic
STATS = {OPEN: 0, MEDIUM: 0, DENSE: 0, ESCAPE: 0}


def set_clock(fn):
    """Install the node's sim-time source: set_clock(lambda: node.get_clock()
    .now().nanoseconds / 1e9)."""
    global CLOCK
    CLOCK = fn


def enabled():
    return os.environ.get('AHPPA_PORTFOLIO', '0') not in ('0', '', 'false')


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
        # all 23 plans of the 2026-10-25 flight.
        return DISPATCH.update(t, now=CLOCK()), t

    def plan(self, sx, sy, gx, gy, start_layer=None, goal_layer=None):
        if not enabled():
            return super().plan(sx, sy, gx, gy, start_layer, goal_layer)
        L = len(Z_LAYERS)
        sl = 1 if start_layer is None else int(start_layer)
        sl = max(0, min(L - 1, sl))
        gl = sl if goal_layer is None else int(goal_layer)
        gl = max(0, min(L - 1, gl))
        try:
            mode, tight = self._mode_for(sx, sy, sl)
        except Exception:
            mode, tight = MEDIUM, 0.0
        self.last_mode = mode
        STATS[mode] = STATS.get(mode, 0) + 1

        start = (sx, sy, Z_LAYERS[sl])
        goal = (gx, gy, Z_LAYERS[gl])
        try:
            p = None
            if mode == OPEN:
                p = self._lt.plan(self._grids, start, goal)
            elif mode in (DENSE, ESCAPE):
                p = self._ca.plan(self._grids, start, goal)
            if p:
                print(f"    [portfolio] {mode}  clearance "
                      f"{FEATURE.last_clearance:.2f} m  tight {tight:.2f}  "
                      f"{len(p)} waypoints   {dict(STATS)}")
                return p
        except Exception as e:
            print(f"    [portfolio] {mode} failed ({type(e).__name__}: {e}), "
                  f"falling back to grid A*")
        else:
            if mode != MEDIUM:
                # Never fall through silently: without this the log shows only
                # the stock planner's message and the portfolio looks inactive.
                print(f"    [portfolio] {mode} returned NO PATH "
                      f"(clearance {FEATURE.last_clearance:.2f} m, "
                      f"tight {tight:.2f}) -> stock A*")
        # MEDIUM is the stock planner, and so is every fallback path
        return super().plan(sx, sy, gx, gy, start_layer, goal_layer)
