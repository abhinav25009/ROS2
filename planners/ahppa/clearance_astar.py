#!/usr/bin/env python3
"""Clearance-weighted A* - the DENSE member of the portfolio.

WHY THIS EXISTS
    A* and Lazy Theta* both minimise length. In open space that is right and
    Theta* does it better. In clutter it is wrong: the shortest path through
    a 0.95 m doorway runs along the frame, and a drone tracking it with any
    lateral error touches the frame. Two flight collisions came out of this.

    So the DENSE planner does not compete on length or on milliseconds. It
    pays length for margin, and is judged on the minimum clearance of the
    path it returns. Measured over 5 third-party worlds: 40% more margin for
    6% more path, and cheaper than Lazy Theta* while doing it.

HOW
    A Euclidean distance transform per layer gives clearance in metres.
    Edge cost is step length times a penalty rising quadratically as
    clearance falls below d_safe:

        cost = step * (1 + w * max(0, (d_safe - clear) / d_safe) ** 2)

    Quadratic, not linear, so mild narrowing is nearly free and scraping a
    wall is expensive. A linear penalty makes the planner pay a little to
    avoid every wall everywhere, lengthening corridor paths for no benefit.

    The heuristic stays Euclidean, admissible because the penalty only ever
    multiplies cost upward (>= 1).
"""
import heapq
import math
import time

import numpy as np
from edt import distance_transform_edt

from lazy_theta import LazyThetaStar, NB8


class ClearanceAStar:
    name = "Clearance A*"

    def __init__(self, z_layers, resolution=0.25, origin_x=-10.0,
                 origin_y=-8.0, d_safe=0.6, weight=3.0, max_expansions=400000):
        self.z_layers = list(z_layers)
        self.res = float(resolution)
        self.ox = float(origin_x)
        self.oy = float(origin_y)
        self.d_safe = float(d_safe)
        self.weight = float(weight)
        self.max_expansions = int(max_expansions)
        # Wall-clock cap per plan() call, None = no cap. The portfolio sets
        # it: on long routes in big maps the search can run 17-23 s before
        # hitting max_expansions, and only THEN would the portfolio fall
        # back to stock A* (2026-11-17 baseline benchmark, SC5 / F5).
        self.time_budget_s = None
        self.gave_up = None          # None | 'budget' | 'expansions'
        self._zmid = np.array([(self.z_layers[i] + self.z_layers[i + 1]) / 2.0
                               for i in range(len(self.z_layers) - 1)])
        self.expansions = 0
        self.los_checks = 0
        self._clear = None
        self._key = None

    layer_of = LazyThetaStar.layer_of
    to_cell = LazyThetaStar.to_cell
    to_world = LazyThetaStar.to_world
    _free = LazyThetaStar._free
    _neighbours = LazyThetaStar._neighbours
    line_of_sight = LazyThetaStar.line_of_sight
    _nudge = LazyThetaStar._nudge

    def clearance(self, grids):
        """Metres to the nearest blocked cell, per layer. Cached: the EDT is
        the expensive part and the grid rarely changes between queries."""
        key = (grids.shape, int(grids.sum()))
        if self._clear is not None and self._key == key:
            return self._clear
        out = np.empty(grids.shape, dtype=np.float32)
        for li in range(grids.shape[0]):
            out[li] = distance_transform_edt(~grids[li]) * self.res
        self._clear = out
        self._key = key
        return out

    def penalty(self, clear):
        if clear >= self.d_safe:
            return 1.0
        t = (self.d_safe - clear) / self.d_safe
        return 1.0 + self.weight * t * t

    def plan(self, grids, start_world, goal_world):
        self.expansions = 0
        self.los_checks = 0
        self.gave_up = None
        deadline = (time.perf_counter() + self.time_budget_s
                    if self.time_budget_s else None)
        grids = np.asarray(grids, dtype=bool)
        cl = self.clearance(grids)

        s = self.to_cell(*start_world)
        g = self.to_cell(*goal_world)
        if not self._free(grids, s):
            s = self._nudge(grids, s)
        if not self._free(grids, g):
            g = self._nudge(grids, g)
        if s is None or g is None:
            return None
        if s == g:
            return [self.to_world(s)]
        gw = self.to_world(g)

        def h(n):
            return math.dist(self.to_world(n), gw)

        gscore = {s: 0.0}
        parent = {}
        pq = [(h(s), s)]
        closed = set()
        while pq:
            _, n = heapq.heappop(pq)
            if n in closed:
                continue
            closed.add(n)
            self.expansions += 1
            if n == g:
                out, k = [], n
                while k in parent:
                    out.append(self.to_world(k))
                    k = parent[k]
                out.append(self.to_world(s))
                out.reverse()
                return out
            if self.expansions > self.max_expansions:
                self.gave_up = 'expansions'
                return None
            if (deadline is not None and not self.expansions & 1023
                    and time.perf_counter() > deadline):
                self.gave_up = 'budget'
                return None
            nw = self.to_world(n)
            for m in self._neighbours(grids, n):
                if m in closed:
                    continue
                step = math.dist(nw, self.to_world(m))
                cand = gscore[n] + step * self.penalty(float(cl[m]))
                if cand < gscore.get(m, math.inf):
                    gscore[m] = cand
                    parent[m] = n
                    heapq.heappush(pq, (cand + h(m), m))
        return None

    def path_clearance(self, grids, path, step=None):
        """Minimum and mean clearance along a path, sampled densely so a long
        any-angle segment is not judged only by its endpoints."""
        cl = self.clearance(grids)
        step = step or self.res * 0.5
        vals = []
        for a, b in zip(path, path[1:]):
            d = math.dist(a, b)
            n = max(int(d / step), 1)
            for i in range(n + 1):
                t = i / n
                p = (a[0] + (b[0] - a[0]) * t,
                     a[1] + (b[1] - a[1]) * t,
                     a[2] + (b[2] - a[2]) * t)
                li, r, c = self.to_cell(*p)
                L, H, W = grids.shape
                if 0 <= li < L and 0 <= r < H and 0 <= c < W:
                    vals.append(float(cl[li, r, c]))
        if not vals:
            return 0.0, 0.0
        return min(vals), float(np.mean(vals))
