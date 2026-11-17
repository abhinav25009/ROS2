#!/usr/bin/env python3
"""Lazy Theta* over the layered (row, col, layer) grid.

WHY THIS PLANNER EXISTS
    Grid A* can only ever move along the 8 compass directions, so a straight
    6 m run down the hall comes back as 24 staircase waypoints with a 45 deg
    zigzag baked in. The drone then flies the zigzag. Theta* removes the
    constraint: a node's parent may be ANY earlier node, not just a
    neighbour, so the path is made of long straight segments at arbitrary
    angles - which is what the hall, the garden and the atrium actually want.

WHY *LAZY*
    Plain Theta* checks line-of-sight every time it relaxes an edge. Most of
    those checks are wasted on nodes that never get expanded. Lazy Theta*
    (Nash, Koenig & Tovey) assumes the line of sight is clear when relaxing,
    and verifies it once, at expansion. If the assumption was wrong it
    repairs the node from its already-closed neighbours. One LOS check per
    expanded node instead of one per edge.

CONTRACT
    plan(grids, start, goal) -> [(x, y, z), ...] in world coordinates, or
    None. `grids` is a (L, H, W) bool array, True = blocked, one layer per
    entry of z_layers. Start/goal are world (x, y, z) tuples. Nothing here
    touches ROS or OctoMap, which is what makes it testable offline.
"""
import heapq
import math

import numpy as np

NB8 = ((-1, 0), (1, 0), (0, -1), (0, 1),
       (-1, -1), (-1, 1), (1, -1), (1, 1))


class LazyThetaStar:
    name = "LazyTheta*"

    def __init__(self, z_layers, resolution=0.25, origin_x=-10.0, origin_y=-8.0,
                 los_substep=0.25, max_expansions=200000):
        self.z_layers = list(z_layers)
        self.res = float(resolution)
        self.ox = float(origin_x)
        self.oy = float(origin_y)
        # LOS is sampled every los_substep cells
        self.los_substep = float(los_substep)
        # midpoints between adjacent layers: searchsorted on these maps an
        # altitude to its nearest layer in one vectorised step
        self._zmid = np.array([(self.z_layers[i] + self.z_layers[i + 1]) / 2.0
                               for i in range(len(self.z_layers) - 1)])
        self.max_expansions = int(max_expansions)
        self.expansions = 0
        self.los_checks = 0

    # -- coordinate helpers --------------------------------------------------
    def layer_of(self, z):
        return min(range(len(self.z_layers)),
                   key=lambda i: abs(self.z_layers[i] - z))

    def to_cell(self, x, y, z):
        return (self.layer_of(z),
                int((y - self.oy) / self.res),
                int((x - self.ox) / self.res))

    def to_world(self, n):
        li, r, c = n
        return (self.ox + (c + 0.5) * self.res,
                self.oy + (r + 0.5) * self.res,
                self.z_layers[li])

    def _free(self, grids, n):
        li, r, c = n
        L, H, W = grids.shape
        if not (0 <= li < L and 0 <= r < H and 0 <= c < W):
            return False
        return not grids[li, r, c]

    # -- the any-angle primitive ---------------------------------------------
    def line_of_sight(self, grids, a, b):
        """Straight segment between two cell centres, in world space.

        Sampling in world space rather than Bresenham on the grid is what
        lets a segment cross layers: the drone climbing through the atrium
        moves in x, y and z at once, and each sample lands in whichever
        layer is nearest that sample's altitude.
        """
        self.los_checks += 1
        wa, wb = self.to_world(a), self.to_world(b)
        d = math.dist(wa, wb)
        steps = max(int(d / (self.res * self.los_substep)), 1)
        if steps < 2:
            return True
        # vectorised: sampling the whole segment at once turns the hot loop
        # from Python arithmetic into three numpy ops
        t = np.arange(1, steps, dtype=np.float64) / steps
        x = wa[0] + (wb[0] - wa[0]) * t
        y = wa[1] + (wb[1] - wa[1]) * t
        z = wa[2] + (wb[2] - wa[2]) * t
        li = np.searchsorted(self._zmid, z)
        r = ((y - self.oy) / self.res).astype(np.int64)
        c = ((x - self.ox) / self.res).astype(np.int64)
        L, H, W = grids.shape
        if (r < 0).any() or (r >= H).any() or (c < 0).any() or (c >= W).any():
            return False
        np.clip(li, 0, L - 1, out=li)
        return not grids[li, r, c].any()

    def cost(self, a, b):
        return math.dist(self.to_world(a), self.to_world(b))

    def _neighbours(self, grids, n):
        """8-connected in-layer plus one layer up/down.

        A diagonal step is only allowed when both orthogonal cells beside it
        are free. Without that test the path squeezes through the shared
        corner of two diagonally-placed obstacles - geometrically zero-width,
        and the drone does not fit through it. This bites plain grid A* too.
        """
        li, r, c = n
        out = []
        for dr, dc in NB8:
            m = (li, r + dr, c + dc)
            if not self._free(grids, m):
                continue
            if dr and dc and not (self._free(grids, (li, r + dr, c))
                                  and self._free(grids, (li, r, c + dc))):
                continue
            out.append(m)
        for dl in (-1, 1):
            m = (li + dl, r, c)
            if self._free(grids, m):
                out.append(m)
        return out

    # -- search --------------------------------------------------------------
    def plan(self, grids, start_world, goal_world):
        self.expansions = 0
        self.los_checks = 0
        grids = np.asarray(grids, dtype=bool)
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
        parent = {s: s}
        open_q = [(h(s), s)]
        closed = set()

        while open_q:
            _, n = heapq.heappop(open_q)
            if n in closed:
                continue
            # --- the lazy part: the optimistic assumption is checked here ---
            p = parent[n]
            if p != n and not self.line_of_sight(grids, p, n):
                best, bestg = None, math.inf
                for m in self._neighbours(grids, n):
                    if m in closed:
                        cand = gscore[m] + self.cost(m, n)
                        if cand < bestg:
                            best, bestg = m, cand
                if best is None:
                    continue              # unreachable by any real path
                parent[n] = best
                gscore[n] = bestg
            closed.add(n)
            self.expansions += 1
            if n == g:
                return self._trace(parent, s, g)
            if self.expansions > self.max_expansions:
                return None

            pn = parent[n]
            for m in self._neighbours(grids, n):
                if m in closed:
                    continue
                # optimistic relaxation: pretend LOS(parent(n), m) holds
                cand = gscore[pn] + self.cost(pn, m)
                if cand < gscore.get(m, math.inf):
                    gscore[m] = cand
                    parent[m] = pn
                    heapq.heappush(open_q, (cand + h(m), m))
        return None

    def _nudge(self, grids, n):
        """Pull a start/goal that landed in an obstacle to the nearest free cell."""
        li, r, c = n
        for rad in range(1, 9):
            for dl in (0, -1, 1):
                for dr in range(-rad, rad + 1):
                    for dc in range(-rad, rad + 1):
                        if max(abs(dr), abs(dc)) != rad:
                            continue
                        m = (li + dl, r + dr, c + dc)
                        if self._free(grids, m):
                            return m
        return None

    def _trace(self, parent, s, g):
        out, n = [], g
        seen = set()
        while True:
            out.append(self.to_world(n))
            if n == s:
                break
            if n in seen:                 # parent cycle, should not happen
                return None
            seen.add(n)
            n = parent[n]
        out.reverse()
        return out


class GridAStar:
    """The incumbent, for comparison: 8-connected per layer + vertical moves."""
    name = "A* (grid)"

    def __init__(self, z_layers, resolution=0.25, origin_x=-10.0, origin_y=-8.0):
        self.z_layers = list(z_layers)
        self.res = float(resolution)
        self.ox = float(origin_x)
        self.oy = float(origin_y)
        self._zmid = np.array([(self.z_layers[i] + self.z_layers[i + 1]) / 2.0
                               for i in range(len(self.z_layers) - 1)])
        self.expansions = 0
        self.los_checks = 0

    layer_of = LazyThetaStar.layer_of
    to_cell = LazyThetaStar.to_cell
    to_world = LazyThetaStar.to_world
    _free = LazyThetaStar._free
    _neighbours = LazyThetaStar._neighbours
    cost = LazyThetaStar.cost
    line_of_sight = LazyThetaStar.line_of_sight
    _nudge = LazyThetaStar._nudge

    def plan(self, grids, start_world, goal_world):
        self.expansions = 0
        self.los_checks = 0
        grids = np.asarray(grids, dtype=bool)
        s = self.to_cell(*start_world)
        g = self.to_cell(*goal_world)
        if not self._free(grids, s):
            s = self._nudge(grids, s)
        if not self._free(grids, g):
            g = self._nudge(grids, g)
        if s is None or g is None:
            return None
        gw = self.to_world(g)

        def h(n):
            return math.dist(self.to_world(n), gw)

        gscore = {s: 0.0}
        parent = {}
        open_q = [(h(s), s)]
        closed = set()
        while open_q:
            _, n = heapq.heappop(open_q)
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
            for m in self._neighbours(grids, n):
                if m in closed:
                    continue
                cand = gscore[n] + self.cost(n, m)
                if cand < gscore.get(m, math.inf):
                    gscore[m] = cand
                    parent[m] = n
                    heapq.heappush(open_q, (cand + h(m), m))
        return None
