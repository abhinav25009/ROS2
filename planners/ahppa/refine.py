#!/usr/bin/env python3
"""Path refinement for AHPPA v2: line-of-sight shortcutting and corridor
Lazy Theta*.

WHY (44-layout benchmark, benchmark/analyse_v2.py)
    Lazy Theta* gives ~3.5% shorter paths than layered A* but costs ~3x the
    planning time, and the drone hovers while it plans. Two cheaper routes to
    the same paths:
      shortcut()        greedy string pulling on the A* path: a few ms,
                        recovers ~90% of Lazy Theta*'s length saving
      corridor_grids()  block everything farther than R from the smoothed
                        path; Lazy Theta* inside that corridor returns the
                        SAME path as unrestricted Lazy Theta* on ~99% of
                        queries, and can run while the drone is already
                        flying the smoothed path (anytime refinement)
Both use the same blocked-cell grids (True = blocked, inflated by the safety
margin) and the same world-space line-of-sight test as Lazy Theta*, so a
shortcut is exactly as safe as a Lazy Theta* segment.
"""
import math

import numpy as np


def shortcut(lt, grids, path):
    """Greedy string pulling: from each anchor, keep extending to the next
    point while the straight segment stays in free space. lt supplies
    to_cell() and line_of_sight() (a LazyThetaStar instance)."""
    if not path or len(path) < 3:
        return path
    cells = [lt.to_cell(*p) for p in path]
    out = [path[0]]
    i, n = 0, len(path)
    while i < n - 1:
        j = i + 1
        while j + 1 < n and lt.line_of_sight(grids, cells[i], cells[j + 1]):
            j += 1
        out.append(path[j])
        i = j
    return out


def corridor_grids(lt, grids, path, radius=1.0):
    """Copy of grids with every cell farther than radius (in x-y, every
    layer) from the polyline marked blocked."""
    L, H, W = grids.shape
    k = max(1, int(round(radius / lt.res)))
    keep = np.zeros((H, W), dtype=bool)
    pts = np.asarray(path, dtype=np.float64)
    if len(pts) == 1:
        pts = np.vstack([pts, pts])
    for a, b in zip(pts[:-1], pts[1:]):
        d = math.hypot(b[0] - a[0], b[1] - a[1])
        m = max(2, int(d / (lt.res * 2)) + 2)
        for t in np.linspace(0.0, 1.0, m):
            x, y = a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t
            r, c = int((y - lt.oy) / lt.res), int((x - lt.ox) / lt.res)
            keep[max(0, r - k):r + k + 1, max(0, c - k):c + k + 1] = True
    return grids | ~keep[None, :, :]


def path_length(p):
    return sum(math.dist(a, b) for a, b in zip(p, p[1:])) if p else math.inf


def refine(lt, grids, path, radius=1.0, budget_s=5.0):
    """Corridor Lazy Theta* from path[0] to path[-1]. Returns the refined
    path if it is valid and shorter, else the input path."""
    if not path or len(path) < 3:
        return path
    old = lt.time_budget_s
    lt.time_budget_s = budget_s
    try:
        q = lt.plan(corridor_grids(lt, grids, path, radius), path[0], path[-1])
    finally:
        lt.time_budget_s = old
    if q and path_length(q) < path_length(path) - 1e-6:
        return q
    return path


def segments_clear(lt, grids, path, start=0, horizon_m=None):
    """True if every SEGMENT of path from index start is free (optionally
    only the first horizon_m metres). Checking segments, not just
    waypoints, matters for any-angle paths whose waypoints are metres apart."""
    acc = 0.0
    for a, b in zip(path[start:], path[start + 1:]):
        if not lt.line_of_sight(grids, lt.to_cell(*a), lt.to_cell(*b)):
            return False
        acc += math.dist(a, b)
        if horizon_m is not None and acc > horizon_m:
            break
    return True
