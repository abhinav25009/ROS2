#!/usr/bin/env python3
"""Wrap a MultiLayerPlanner so AHPPA can dispatch between planners.

Deliberately a WRAPPER, not an edit to planner_3d.py. MultiLayerPlanner is
rebuilt on every replan and is called from four places in navigator_astar;
adding methods to it means every one of those paths has to be re-tested.
A wrapper reuses the grids it already built and changes nothing.

    planner = MultiLayerPlanner(occ_grid, octomap_3d)
    lt      = LazyThetaAdapter(planner)
    path    = lt.plan(sx, sy, gx, gy, start_layer, goal_layer)

Both expose the same signature, which is the contract ahppa.py wants for an
injected planner.
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from lazy_theta import LazyThetaStar


class LazyThetaAdapter:
    name = "LazyTheta*"

    def __init__(self, planner, z_layers=None, los_substep=0.25):
        self.planner = planner
        self.z_layers = list(z_layers if z_layers is not None
                             else _z_layers_of(planner))
        # layer_grids are uint8 with 1 = blocked, and the safety margin has
        # ALREADY been inflated in by MultiLayerPlanner.__init__ - do not
        # inflate again or doorways close up.
        self.grids = np.asarray(planner.layer_grids, dtype=bool)
        self.impl = LazyThetaStar(
            self.z_layers,
            resolution=planner.resolution,
            origin_x=planner.origin_x,
            origin_y=planner.origin_y,
            los_substep=los_substep)
        self.expansions = 0
        self.los_checks = 0

    def plan(self, sx, sy, gx, gy, start_layer=None, goal_layer=None):
        zs = self.z_layers
        sl = int(start_layer) if start_layer is not None else _default_layer(zs)
        sl = max(0, min(len(zs) - 1, sl))
        start = (sx, sy, zs[sl])

        if goal_layer is not None:
            order = [max(0, min(len(zs) - 1, int(goal_layer)))]
        else:
            # "any layer" - try nearest to the start first, which is what the
            # grid planner's heuristic effectively prefers anyway
            order = sorted(range(len(zs)), key=lambda i: abs(i - sl))

        for gl in order:
            p = self.impl.plan(self.grids, start, (gx, gy, zs[gl]))
            self.expansions = self.impl.expansions
            self.los_checks = self.impl.los_checks
            if p:
                return p
        return None


def _z_layers_of(planner):
    zs = getattr(planner, 'z_layers', None)
    if zs is not None:
        return zs
    from planner_3d import Z_LAYERS          # noqa: E402
    return Z_LAYERS


def _default_layer(zs):
    try:
        from planner_3d import DEFAULT_LAYER  # noqa: E402
        return DEFAULT_LAYER
    except Exception:
        return min(1, len(zs) - 1)


# -- three-way comparison, for the benchmark table --------------------------
def compare(planner, sx, sy, gx, gy, start_layer=None, goal_layer=None,
            label="", writer=None):
    """Run A*, the existing eager Theta*, and Lazy Theta* on one query.

    Returns a list of result dicts and, if `writer` is a csv.writer, appends
    rows in the schema of benchmark/benchmark_results.csv so the numbers sit
    beside the repo's baseline.
    """
    import math
    import time

    lt = LazyThetaAdapter(planner)
    runs = [("A* (grid)", planner.plan),
            ("Theta* (eager)", getattr(planner, 'plan_theta_star', None)),
            ("LazyTheta*", lt.plan)]
    out = []
    for name, fn in runs:
        if fn is None:
            continue
        t0 = time.perf_counter()
        try:
            p = fn(sx, sy, gx, gy, start_layer, goal_layer)
        except Exception as e:                       # a planner must not
            p, err = None, repr(e)                   # take the stack down
        else:
            err = ""
        ms = (time.perf_counter() - t0) * 1000
        if p:
            length = sum(math.dist(a, b) for a, b in zip(p, p[1:]))
            turn = _total_turning(p)
            clean = _verify(lt.impl, lt.grids, p)
        else:
            length = turn = 0.0
            clean = False
        r = dict(case=label, algo=name, ms=round(ms, 2),
                 length=round(length, 3), waypoints=len(p) if p else 0,
                 turning=round(turn, 1), ok=bool(p) and clean, error=err)
        out.append(r)
        if writer is not None:
            writer.writerow([label, name, r['ms'], r['length'], r['waypoints'],
                             '', r['turning'], str(r['ok'])])
    return out


def _total_turning(p):
    import math
    tot = 0.0
    for a, b, c in zip(p, p[1:], p[2:]):
        u = np.array(b) - np.array(a)
        v = np.array(c) - np.array(b)
        nu, nv = np.linalg.norm(u), np.linalg.norm(v)
        if nu < 1e-9 or nv < 1e-9:
            continue
        cos = float(np.clip(np.dot(u, v) / (nu * nv), -1.0, 1.0))
        tot += math.degrees(math.acos(cos))
    return tot


def _verify(impl, grids, p):
    """A shorter path that clips a wall is not a better path."""
    for a, b in zip(p, p[1:]):
        if not impl.line_of_sight(grids, impl.to_cell(*a), impl.to_cell(*b)):
            return False
        if not impl._free(grids, impl.to_cell(*b)):
            return False
    return True
