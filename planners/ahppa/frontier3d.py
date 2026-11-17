#!/usr/bin/env python3
"""Per-layer 3D frontier extraction for a multi-storey map.

WHY THIS EXISTS
    The stock frontier explorer reads /projected_map, a flatten of the whole
    vertical column. On two floors every ground-floor cell has the first-floor
    slab above it, so the flatten marks everything occupied and exploration
    dies after one scan. Frontiers must be found per altitude layer instead.

THE MISSING THIRD STATE
    Layer grids built from OctoMap voxels only know "occupied". Everything
    unobserved looks free, so "free cell beside unknown" has nothing to find.
    VisibilityGrid supplies the missing state: cells the drone has actually
    been within sensor range of. Frontier = observed-free beside unobserved.

WHY GOALS ARE REACHED IN STEPS
    The planner in navigator_astar treats unobserved space as FREE, so given a
    far goal it routes straight through walls it has never seen - which is how
    the drone crashed. A hard distance cap stops that but throws away every
    distant frontier, so exploration quits around 60% coverage.

    The fix is geodesic, not Euclidean. This module runs its own Dijkstra over
    OBSERVED-FREE space only. A frontier is a candidate only if a path to it
    exists entirely inside mapped territory, and the goal published is a
    waypoint max_step metres along that path, in clear line of sight. Distant
    frontiers are kept, not discarded: they take several hops, and every hop
    stays in known air.
"""
import heapq
import math
import numpy as np

NB8 = ((-1, 0), (1, 0), (0, -1), (0, 1),
       (-1, -1), (-1, 1), (1, -1), (1, 1))


class VisibilityGrid:
    """Dense 3D bool grid: has the drone been within sensor range of this cell?"""

    def __init__(self, bounds=(-10.0, 16.0, -8.0, 8.0, 0.0, 7.0),
                 res=0.25, sensor_range=4.0, move_thresh=0.4):
        self.x0, self.x1, self.y0, self.y1, self.z0, self.z1 = bounds
        self.res = float(res)
        self.range = float(sensor_range)
        self.move_thresh = float(move_thresh)

        self.nx = int(round((self.x1 - self.x0) / self.res))
        self.ny = int(round((self.y1 - self.y0) / self.res))
        self.nz = int(round((self.z1 - self.z0) / self.res))
        self.g = np.zeros((self.ny, self.nx, self.nz), dtype=bool)

        r = int(math.ceil(self.range / self.res))
        o = np.arange(-r, r + 1)
        dy, dx, dz = np.meshgrid(o, o, o, indexing='ij')
        keep = (dx * dx + dy * dy + dz * dz) <= r * r
        self.off = np.stack([dy[keep], dx[keep], dz[keep]], axis=1)

        self._last = None
        self.marks = 0

    def idx(self, x, y, z):
        return (int((y - self.y0) / self.res),
                int((x - self.x0) / self.res),
                int((z - self.z0) / self.res))

    def mark(self, x, y, z, force=False):
        """Mark everything within sensor range of this pose as observed."""
        if not force and self._last is not None:
            if math.dist((x, y, z), self._last) < self.move_thresh:
                return False
        self._last = (x, y, z)
        iy, ix, iz = self.idx(x, y, z)
        c = self.off + np.array([iy, ix, iz])
        ok = ((c[:, 0] >= 0) & (c[:, 0] < self.ny) &
              (c[:, 1] >= 0) & (c[:, 1] < self.nx) &
              (c[:, 2] >= 0) & (c[:, 2] < self.nz))
        c = c[ok]
        self.g[c[:, 0], c[:, 1], c[:, 2]] = True
        self.marks += 1
        return True

    def slice_observed(self, z_lo, z_hi):
        """2D mask: observed anywhere in this altitude band."""
        a = max(0, int((z_lo - self.z0) / self.res))
        b = min(self.nz, int(math.ceil((z_hi - self.z0) / self.res)))
        if b <= a:
            return np.zeros((self.ny, self.nx), dtype=bool)
        return self.g[:, :, a:b].any(axis=2)

    @property
    def observed_fraction(self):
        return float(self.g.mean())


class LayeredFrontierFinder:
    """Find frontiers per altitude layer, reachable through observed space."""

    def __init__(self, vis, z_layers, clearance=0.3,
                 min_cluster=4, blacklist_radius=1.0,
                 max_step=3.0, inflate=1, band=2.0):
        self.vis = vis
        self.z_layers = list(z_layers)
        self.clearance = float(clearance)
        self.min_cluster = int(min_cluster)
        self.blacklist_radius = float(blacklist_radius)
        # how far a single published goal may be, measured along the path
        self.max_step = float(max_step)
        # grow obstacles by this many cells so corridors keep drone clearance
        self.inflate = int(inflate)
        # frontiers within the same distance band compete on size, not distance
        self.band = float(band)
        self.last_reachable = 0
        self.last_blocked = 0

    # -- masks ---------------------------------------------------------------
    def _occupied(self, pts, z):
        m = np.zeros((self.vis.ny, self.vis.nx), dtype=bool)
        if pts is None or len(pts) == 0:
            return m
        p = np.asarray(pts, dtype=np.float32)
        band = p[(p[:, 2] >= z - self.clearance) & (p[:, 2] <= z + self.clearance)]
        if len(band) == 0:
            return m
        ix = np.floor((band[:, 0] - self.vis.x0) / self.vis.res).astype(np.int64)
        iy = np.floor((band[:, 1] - self.vis.y0) / self.vis.res).astype(np.int64)
        ok = (ix >= 0) & (ix < self.vis.nx) & (iy >= 0) & (iy < self.vis.ny)
        m[iy[ok], ix[ok]] = True
        return m

    @staticmethod
    def _neighbour_any(mask):
        """True where any 4-connected neighbour is True."""
        out = np.zeros_like(mask)
        out[1:, :] |= mask[:-1, :]
        out[:-1, :] |= mask[1:, :]
        out[:, 1:] |= mask[:, :-1]
        out[:, :-1] |= mask[:, 1:]
        return out

    @classmethod
    def _grow(cls, mask, n):
        for _ in range(max(n, 0)):
            mask = mask | cls._neighbour_any(mask)
        return mask

    def _cluster(self, mask):
        """Connected components over a sparse bool mask, no scipy."""
        cells = set(map(tuple, np.argwhere(mask)))
        out = []
        while cells:
            seed = cells.pop()
            comp, stack = [seed], [seed]
            while stack:
                r, c = stack.pop()
                for n in ((r+1, c), (r-1, c), (r, c+1), (r, c-1),
                          (r+1, c+1), (r-1, c-1), (r+1, c-1), (r-1, c+1)):
                    if n in cells:
                        cells.remove(n); comp.append(n); stack.append(n)
            if len(comp) >= self.min_cluster:
                out.append(comp)
        return out

    # -- reachability through observed-free space -----------------------------
    def _start_node(self, free, drone_xyz):
        """Nearest free cell to the drone, searched outward a little."""
        dx, dy, dz = drone_xyz
        li = min(range(len(self.z_layers)),
                 key=lambda i: abs(self.z_layers[i] - dz))
        r0, c0, _ = self.vis.idx(dx, dy, dz)
        L, ny, nx = free.shape
        for rad in range(0, 9):
            for lo in (li, li - 1, li + 1):
                if not 0 <= lo < L:
                    continue
                for dr in range(-rad, rad + 1):
                    for dc in range(-rad, rad + 1):
                        if max(abs(dr), abs(dc)) != rad:
                            continue
                        r, c = r0 + dr, c0 + dc
                        if 0 <= r < ny and 0 <= c < nx and free[lo, r, c]:
                            return lo, r, c
        return None

    def _dijkstra(self, free, start):
        """Shortest path from start over free cells, 8-connected per layer
        plus vertical transitions between adjacent layers."""
        L, ny, nx = free.shape
        plane = ny * nx
        flat = free.reshape(-1)
        dist = np.full(flat.size, np.inf, dtype=np.float64)
        par = np.full(flat.size, -1, dtype=np.int64)
        res = self.vis.res
        diag = res * math.sqrt(2.0)
        zs = self.z_layers

        si = (start[0] * ny + start[1]) * nx + start[2]
        dist[si] = 0.0
        pq = [(0.0, si)]
        pop, push = heapq.heappop, heapq.heappush
        while pq:
            d, u = pop(pq)
            if d > dist[u]:
                continue
            li, rem = divmod(u, plane)
            r, c = divmod(rem, nx)
            for dr, dc in NB8:
                rr, cc = r + dr, c + dc
                if not (0 <= rr < ny and 0 <= cc < nx):
                    continue
                v = u + dr * nx + dc
                if not flat[v]:
                    continue
                nd = d + (diag if dr and dc else res)
                if nd < dist[v]:
                    dist[v] = nd
                    par[v] = u
                    push(pq, (nd, v))
            for dl in (-1, 1):
                ll = li + dl
                if not 0 <= ll < L:
                    continue
                v = u + dl * plane
                if not flat[v]:
                    continue
                nd = d + abs(zs[ll] - zs[li])
                if nd < dist[v]:
                    dist[v] = nd
                    par[v] = u
                    push(pq, (nd, v))
        return dist, par

    def _world(self, li, r, c):
        return (self.vis.x0 + (c + 0.5) * self.vis.res,
                self.vis.y0 + (r + 0.5) * self.vis.res,
                self.z_layers[li])

    def _los(self, free, a, b):
        """Is the straight line a->b entirely inside observed-free space?

        This matters because the navigator's A* does NOT know about observed
        space: given a goal it takes the shortest optimistic route, which can
        cut a corner through a wall it has never seen even when the goal
        itself is in mapped air. Only hand it goals it can reach in a line.
        """
        d = math.dist(a, b)
        n = max(int(d / (self.vis.res * 0.5)), 1)
        L = free.shape[0]
        for i in range(n + 1):
            t = i / n
            x = a[0] + (b[0] - a[0]) * t
            y = a[1] + (b[1] - a[1]) * t
            z = a[2] + (b[2] - a[2]) * t
            li = min(range(L), key=lambda j: abs(self.z_layers[j] - z))
            r = int((y - self.vis.y0) / self.vis.res)
            c = int((x - self.vis.x0) / self.vis.res)
            if not (0 <= r < self.vis.ny and 0 <= c < self.vis.nx):
                return False
            if not free[li, r, c]:
                return False
        return True

    def _step_goal(self, free, par, start_i, goal_i, nx, plane):
        """Furthest point along the mapped path that is within max_step AND
        in clear line of sight from the drone."""
        chain = []
        i = goal_i
        while i != -1:
            chain.append(i)
            if i == start_i:
                break
            i = par[i]
        chain.reverse()
        pts = []
        for i in chain:
            li, rem = divmod(int(i), plane)
            r, c = divmod(rem, nx)
            pts.append(self._world(li, r, c))
        if len(pts) < 2:
            return (pts[-1] if pts else None), 0.0
        total = sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))
        acc, keep = 0.0, None
        for a, b in zip(pts, pts[1:]):
            acc += math.dist(a, b)
            if acc > self.max_step:
                break
            if self._los(free, pts[0], b):
                keep = b
        if keep is None or math.dist(keep, pts[0]) < 0.6:
            return None, total
        return keep, total

    # -- main ----------------------------------------------------------------
    def find(self, voxel_centers, drone_xyz, blacklist=()):
        """Frontiers sorted best-first.

        x/y/z is the goal to publish (a bounded hop inside mapped space);
        tx/ty/tz is the frontier itself; dist is the geodesic path length.
        """
        L = len(self.z_layers)
        ny, nx = self.vis.ny, self.vis.nx
        plane = ny * nx
        free = np.zeros((L, ny, nx), dtype=bool)
        fronts = []

        for li, z in enumerate(self.z_layers):
            occ = self._occupied(voxel_centers, z)
            obs = self.vis.slice_observed(z - self.clearance, z + self.clearance)
            free[li] = obs & ~self._grow(occ, self.inflate)
            fr = (obs & ~occ) & self._neighbour_any(~obs)
            fr &= ~self._neighbour_any(occ)
            if fr.any():
                fronts.append((li, z, fr))

        start = self._start_node(free, drone_xyz)
        if start is None or not fronts:
            self.last_reachable = self.last_blocked = 0
            return []
        start_i = (start[0] * ny + start[1]) * nx + start[2]
        dist, par = self._dijkstra(free, start)

        found, blocked = [], 0
        for li, z, fr in fronts:
            for comp in self._cluster(fr):
                a = np.array(comp, dtype=np.float64)
                mr, mc = a[:, 0].mean(), a[:, 1].mean()
                # A frontier ring around the drone has its centroid AT the
                # drone, which is useless as a goal. Snap to the nearest cell
                # genuinely in the cluster, preferring one Dijkstra reached.
                order = np.argsort((a[:, 0] - mr) ** 2 + (a[:, 1] - mc) ** 2)
                pick = None
                for k in order[:40]:
                    r, c = int(a[k, 0]), int(a[k, 1])
                    gi = (li * ny + r) * nx + c
                    if np.isfinite(dist[gi]):
                        pick = (r, c, gi)
                        break
                if pick is None:
                    blocked += 1
                    continue
                r, c, gi = pick
                wx, wy, wz = self._world(li, r, c)
                if any(math.dist((wx, wy, wz), b) < self.blacklist_radius
                       for b in blacklist):
                    continue
                step, g = self._step_goal(free, par, start_i, gi, nx, plane)
                if step is None:
                    continue
                if any(math.dist(step, b) < self.blacklist_radius
                       for b in blacklist):
                    continue
                g = max(g, 0.3)
                found.append(dict(
                    x=round(step[0], 2), y=round(step[1], 2), z=round(step[2], 2),
                    tx=round(wx, 2), ty=round(wy, 2), tz=round(wz, 2),
                    layer=li, size=len(comp), dist=round(g, 2),
                    score=round(len(comp) / g ** 1.5, 2)))

        self.last_reachable = len(found)
        self.last_blocked = blocked
        # near-first by distance band, biggest frontier wins inside a band.
        # Nothing is discarded for being far: it just waits its turn.
        found.sort(key=lambda f: (int(f['dist'] // self.band), -f['size']))
        return found
