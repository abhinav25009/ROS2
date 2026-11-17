#!/usr/bin/env python3
"""AHPPA orchestrator — Stage 1.

Holds one planner per mode and routes plan() to whichever the dispatcher
selects. Planners are injected as plain callables

    plan(sx, sy, sz, gx, gy, gz) -> list[(x, y, z)] | None

so Stage 1 runs with the planners already in this repo and each specialist can
be swapped in later without touching this file.

Stage 1 mapping (stand-ins) -> final specialists:
    OPEN   : Theta*          -> Lazy Theta*
    MEDIUM : A* (layered)    -> Kinodynamic Hybrid A*
    DENSE  : A* (layered)    -> Incremental Phi*
    ESCAPE : RRT             -> Informed RRT*
"""
import time
from dispatcher import HysteresisDispatcher, OPEN, MEDIUM, DENSE, ESCAPE
from density import DensityEstimator
from clearance_feature import ClearanceEstimator
from watchdog import DeadlockWatchdog


class AHPPA:
    def __init__(self, planners, density=None, dispatcher=None, watchdog=None,
                 feature=None):
        missing = {OPEN, MEDIUM, DENSE, ESCAPE} - set(planners)
        if missing:
            raise ValueError(f"missing planners for modes: {sorted(missing)}")
        self.planners = planners
        # The dispatch feature is local CLEARANCE as of 2026-10-25: density
        # did not transfer between environments (see clearance_feature.py).
        # `density=` still works so old call sites keep running, and passing
        # one explicitly overrides the clearance feature.
        self.density = density
        self.feature = feature or density or ClearanceEstimator()
        self._last = 0.0
        self.dispatcher = dispatcher or HysteresisDispatcher()
        self.watchdog = watchdog or DeadlockWatchdog()
        self.log = []

    def _measure(self, voxel_centers, x, y, z):
        """One scalar in [0,1], higher = tighter, whichever estimator is in
        use. ClearanceEstimator exposes tightness(); the older
        DensityEstimator exposes compute() -> (value, info)."""
        f = self.feature
        if hasattr(f, 'tightness'):
            d = f.tightness(voxel_centers, (x, y, z))
        else:
            d, _ = f.compute(voxel_centers, x, y, z)
        self._last = float(d)
        return self._last

    def tick(self, voxel_centers, x, y, z, t, goal_active):
        """Update the dispatch feature + watchdog, return (mode, value)."""
        d = self._measure(voxel_centers, x, y, z)
        stuck = self.watchdog.update(x, y, z, t, goal_active)
        # pass t through: dwell must be measured on the SAME clock as the rest
        # of the stack (sim time), never wall time.
        return self.dispatcher.update(d, deadlock=stuck, now=t), d

    def plan(self, start, goal, voxel_centers, t=None, mode=None):
        """Plan with the currently selected specialist. Returns a PlanResult."""
        t = time.monotonic() if t is None else t
        if mode is None:
            mode, _ = self.tick(voxel_centers, *start, t, goal_active=True)
        d = self._last or 0.0

        t0 = time.perf_counter()
        try:
            path = self.planners[mode](*start, *goal)
            err = None
        except Exception as e:                       # a failing specialist
            path, err = None, repr(e)                # must not kill the drone
        ms = (time.perf_counter() - t0) * 1000.0

        rec = dict(t=round(t, 3), mode=mode, density=round(d, 4),
                   tightness=round(d, 4),
                   clearance=round(float(getattr(self.feature,
                                                 'last_clearance', float('nan'))), 3),
                   ms=round(ms, 2), n=len(path) if path else 0,
                   ok=bool(path), error=err)
        self.log.append(rec)
        return rec, path

    def stats(self):
        if not self.log:
            return {}
        out = {}
        for m in (OPEN, MEDIUM, DENSE, ESCAPE):
            rows = [r for r in self.log if r['mode'] == m]
            if not rows:
                continue
            lat = sorted(r['ms'] for r in rows)
            out[m] = dict(calls=len(rows),
                          success=round(100 * sum(r['ok'] for r in rows) / len(rows), 1),
                          ms_med=round(lat[len(lat) // 2], 2),
                          ms_p95=round(lat[int(len(lat) * 0.95) - 1], 2))
        out['_switches'] = self.dispatcher.switches
        out['_dwell_pct'] = self.dispatcher.summary()
        out['_deadlocks'] = self.watchdog.trips
        return out
