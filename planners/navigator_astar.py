#!/usr/bin/env python3
"""
=============================================================================
 AUTONOMOUS A* NAVIGATOR v3 — Sensor-Based Mapping (Real-World Ready)
 File: autonomous_navigator.py
=============================================================================

 This navigator does NOT know the room layout in advance.
 Instead, it uses the depth camera + OctoMap to discover obstacles,
 then plans collision-free paths on the SENSOR-DERIVED map.

 Pipeline:
   1. Drone takes off to 1.8m
   2. EXPLORATION PHASE: Drone flies to multiple scan positions and
      rotates 360° at each, allowing OctoMap to build a complete 3D map
      from depth camera data.
   3. READY PHASE: Map is built. User clicks "2D Goal Pose" in RViz.
   4. A* runs on OctoMap's /projected_map (real sensor data).
   5. Drone flies the obstacle-free path.

 Coordinate System:
   Gazebo ENU:  X = East (+right), Y = North (+forward)
   PX4 NED:     X = North,         Y = East
   Conversion:  PX4_X += Gazebo_delta_Y,  PX4_Y += Gazebo_delta_X

 Why this matters for real-world:
   In real-world deployment, you won't have an SDF file.
   This approach uses only sensor data — swap Gazebo for a real depth
   camera and localization system, and the same code works.

=============================================================================
"""

import math
import time
import heapq
import numpy as np
import sys
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PointStamped
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from tf2_msgs.msg import TFMessage
from px4_msgs.msg import (
    OffboardControlMode, TrajectorySetpoint,
    VehicleCommand, VehicleOdometry
)

# Add project root to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from core.octomap_3d_query import OctoMap3DQuery
from planners.planner_3d import (MultiLayerPlanner, Z_LAYERS,
                                 DEFAULT_LAYER, layer_for_z, DRONE_CLEARANCE)
# [portfolio] override placed AFTER the import it shadows: an unconditional
# `from planners.planner_3d import MultiLayerPlanner` after this would rebind
# the name straight back to the stock class, and the log would still claim the
# portfolio was active. Import the MODULE, not the package: ahppa/__init__.py
# does `from .ahppa import AHPPA` and ahppa.py imports `dispatcher` bare.
import os as _os
import sys as _sys
if _os.environ.get('AHPPA_PORTFOLIO', '0') not in ('0', '', 'false'):
    _sys.path.insert(0, _os.path.join(
        _os.path.dirname(_os.path.abspath(__file__)), 'ahppa'))
    from portfolio import PortfolioPlanner as MultiLayerPlanner
    import portfolio as _ahppa
    print('  [portfolio] AHPPA planner portfolio ACTIVE')
else:
    _ahppa = None


def densify_path(path, spacing=0.1):
    """Insert points every `spacing` m along each segment (2D or 3D tuples).

    The follower in _navigate_step / _explore_step takes the closest waypoint
    as progress and then the first waypoint >= LOOKAHEAD_DIST away. On a
    SPARSE path (Lazy Theta* returns 2-7 points) the closest waypoint stays
    the start until the drone is half way, so as soon as it drifts 0.3 m out
    the start itself becomes the target and pulls it back: it oscillates in
    place forever. Seen in flight 2026-11-17 on the first OPEN plan. Dense
    A* paths hid this. Densifying here keeps the any-angle geometry and gives
    the follower and the obstacle checker the spacing they assume."""
    if not path or len(path) < 2:
        return path
    out = [tuple(path[0])]
    for a, b in zip(path, path[1:]):
        d = math.dist(a[:2], b[:2])
        if len(a) == 3:
            d = max(d, abs(b[2] - a[2]))
        n = max(1, int(math.ceil(d / spacing)))
        for k in range(1, n + 1):
            t = k / n
            out.append(tuple(a[i] + t * (b[i] - a[i]) for i in range(len(a))))
    return out


# ─────────────────────────────────────────────────────────────────────────────
#  FRONTIER-BASED EXPLORATION
# ─────────────────────────────────────────────────────────────────────────────
# Instead of hardcoded waypoints, the drone autonomously discovers where to
# fly next by analyzing the OctoMap for boundaries between explored (free)
# and unexplored (unknown) space.
#
# Algorithm:
#   1. Initial 360° scan at spawn to seed the map
#   2. Extract frontier cells from /projected_map (free cells adjacent to unknown)
#   3. Cluster frontiers into groups (connected components)
#   4. Score each cluster: bigger + closer = better
#   5. Plan A* path to the best frontier centroid
#   6. Fly there, do 360° scan
#   7. Repeat until no frontiers remain → room is fully mapped
#
# ─────────────────────────────────────────────────────────────────────────────


class FrontierExtractor:
    """
    Extracts, clusters, and scores frontier cells from an OccupancyGrid.

    A frontier cell is a FREE cell (value == 0) that has at least one
    UNKNOWN neighbor (value == -1). These are the edges of explored space.
    """

    MIN_CLUSTER_SIZE = 3       # Catch small doorway frontiers (was 5)
    MIN_DISTANCE = 0.5          # Don't ignore close frontiers (was 1.5)
    MIN_OBSTACLE_CLEARANCE = 3  # Allow frontiers in corners (~0.3m)
    SCORE_SIZE_WEIGHT = 1.5     # Heavily prioritize large frontiers
    SCORE_DIST_WEIGHT = 0.2     # Very light distance penalty (encourages crossing the room)

    @staticmethod
    def _obstacle_distance_at(r, c, obstacle_grid, max_check=8):
        """
        Compute the minimum distance (in cells) from (r, c) to the nearest
        obstacle cell, searching up to max_check cells away.
        Returns max_check if no obstacle found within range.
        """
        h, w = obstacle_grid.shape
        for radius in range(1, max_check + 1):
            for dr in range(-radius, radius + 1):
                for dc in range(-radius, radius + 1):
                    if abs(dr) != radius and abs(dc) != radius:
                        continue  # Only check the outer ring
                    nr, nc = r + dr, c + dc
                    if 0 <= nr < h and 0 <= nc < w and obstacle_grid[nr, nc] == 1:
                        return radius
        return max_check

    @staticmethod
    def extract(occupancy_grid_msg, drone_x, drone_y, unreachable=None):
        """
        Extract, cluster, and rank frontiers from the OccupancyGrid.

        Args:
            occupancy_grid_msg: nav_msgs/OccupancyGrid from OctoMap
            drone_x, drone_y: Current drone position (Gazebo ENU)
            unreachable: List of (x,y) tuples of known unreachable frontiers

        Returns:
            List of (centroid_x, centroid_y, score, size) tuples,
            sorted by score descending. Empty list if no frontiers.
        """
        if unreachable is None:
            unreachable = []

        info = occupancy_grid_msg.info
        w, h = info.width, info.height
        res = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y

        # Parse the raw occupancy data into a 2D grid
        raw = np.array(occupancy_grid_msg.data, dtype=np.int8).reshape((h, w))

        # Build obstacle grid for clearance checks
        obstacle_grid = np.where(raw > 50, 1, 0).astype(np.uint8)

        # ── Step 1: Find all frontier cells ──
        # Free cells: value == 0
        # Unknown cells: value == -1
        free = (raw == 0)
        unknown = (raw == -1)

        # A frontier cell is free AND has at least one unknown 4-neighbor
        # Pad unknown with True border because anything outside the grid bounding box is also unknown!
        padded = np.pad(unknown, 1, constant_values=True)
        has_unknown_neighbor = (
            padded[:-2, 1:-1] |   # up
            padded[2:, 1:-1]  |   # down
            padded[1:-1, :-2] |   # left
            padded[1:-1, 2:]      # right
        )
        frontier_mask = free & has_unknown_neighbor

        frontier_cells = np.argwhere(frontier_mask)  # Nx2 array of (row, col)
        if len(frontier_cells) == 0:
            return []

        # ── Step 2: Cluster frontier cells (connected components via BFS) ──
        clusters = []
        visited = np.zeros((h, w), dtype=bool)

        for r, c in frontier_cells:
            if visited[r, c]:
                continue

            # BFS flood fill
            cluster = []
            queue = [(r, c)]
            visited[r, c] = True

            while queue:
                cr, cc = queue.pop(0)
                cluster.append((cr, cc))

                for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                    nr, nc = cr + dr, cc + dc
                    if (0 <= nr < h and 0 <= nc < w
                            and not visited[nr, nc]
                            and frontier_mask[nr, nc]):
                        visited[nr, nc] = True
                        queue.append((nr, nc))

            if len(cluster) >= FrontierExtractor.MIN_CLUSTER_SIZE:
                clusters.append(cluster)

        if not clusters:
            return []

        # ── Step 3: Score each cluster ──
        results = []
        for cluster in clusters:
            # Centroid in world coordinates
            rows = [c[0] for c in cluster]
            cols = [c[1] for c in cluster]
            center_r = sum(rows) / len(rows)
            center_c = sum(cols) / len(cols)

            # Safety-aware target selection: find the frontier cell that is
            # both close to the centroid AND far from obstacles.
            # Score each cell: closeness_to_centroid * obstacle_clearance
            best_score = -1.0
            best_r, best_c = cluster[0]
            for r, c in cluster:
                centroid_dist = math.sqrt((r - center_r)**2 + (c - center_c)**2)
                obs_dist = FrontierExtractor._obstacle_distance_at(r, c, obstacle_grid)
                # Prefer cells close to centroid but far from obstacles
                cell_score = obs_dist / (1.0 + centroid_dist * 0.3)
                if cell_score > best_score:
                    best_score = cell_score
                    best_r, best_c = r, c

            # Check obstacle clearance — skip targets too close to walls
            obs_clearance = FrontierExtractor._obstacle_distance_at(
                best_r, best_c, obstacle_grid,
                max_check=FrontierExtractor.MIN_OBSTACLE_CLEARANCE + 1)
            if obs_clearance < FrontierExtractor.MIN_OBSTACLE_CLEARANCE:
                # Try to find ANY cell in the cluster with sufficient clearance
                found_safe = False
                for r, c in cluster:
                    d = FrontierExtractor._obstacle_distance_at(
                        r, c, obstacle_grid,
                        max_check=FrontierExtractor.MIN_OBSTACLE_CLEARANCE + 1)
                    if d >= FrontierExtractor.MIN_OBSTACLE_CLEARANCE:
                        best_r, best_c = r, c
                        found_safe = True
                        break
                if not found_safe:
                    continue  # Skip this entire cluster — too close to walls

            cx = ox + (best_c + 0.5) * res
            cy = oy + (best_r + 0.5) * res

            # Distance from drone
            dist = math.hypot(cx - drone_x, cy - drone_y)
            if dist < FrontierExtractor.MIN_DISTANCE:
                continue

            # Check if this frontier is close to a known unreachable one
            is_unreachable = False
            for ux, uy in unreachable:
                if math.hypot(cx - ux, cy - uy) < 1.0:
                    is_unreachable = True
                    break
            if is_unreachable:
                continue

            # Score: prefer big frontiers, penalize far ones
            size = len(cluster)
            score = (size ** FrontierExtractor.SCORE_SIZE_WEIGHT
                     / (dist ** FrontierExtractor.SCORE_DIST_WEIGHT))

            results.append((cx, cy, score, size))

        # Sort by score descending (best frontier first)
        results.sort(key=lambda x: -x[2])
        return results


# ─────────────────────────────────────────────────────────────────────────────
#  A* PLANNER — Plans on OctoMap's /projected_map (sensor-derived)
# ─────────────────────────────────────────────────────────────────────────────
class SensorMapPlanner:
    """
    A* path planner that operates on an OccupancyGrid from OctoMap.
    This is the REAL sensor data, not a hardcoded map.
    """

    def __init__(self, occupancy_grid_msg, safety_margin=0.7, unknown_penalty=3.0):
        info = occupancy_grid_msg.info
        self.unknown_penalty = unknown_penalty
        self.resolution = info.resolution
        self.origin_x = info.origin.position.x
        self.origin_y = info.origin.position.y
        self.width = info.width
        self.height = info.height

        # Convert 1D occupancy data → 2D numpy grid
        # OccupancyGrid values: 0=free, 100=occupied, -1=unknown
        raw = np.array(occupancy_grid_msg.data).reshape((self.height, self.width))

        # Only cells with probability > 50 are treated as obstacles.
        # Unknown (-1) and free (0) cells are passable — the drone can
        # explore through unknown space.
        self.raw = raw
        self.grid = np.where(raw > 50, 1, 0).astype(np.uint8)

        # Inflate obstacles for safety
        self._inflate(safety_margin)

    def _inflate(self, margin_m):
        """Grow obstacle cells by margin_m meters in all directions."""
        cells = int(math.ceil(margin_m / self.resolution))
        if cells <= 0:
            return

        inflated = np.copy(self.grid)
        obstacles = np.argwhere(self.grid == 1)

        for r, c in obstacles:
            r_lo = max(0, r - cells)
            r_hi = min(self.height, r + cells + 1)
            c_lo = max(0, c - cells)
            c_hi = min(self.width, c + cells + 1)
            inflated[r_lo:r_hi, c_lo:c_hi] = 1

        self.grid = inflated

    def world_to_grid(self, x, y):
        c = int((x - self.origin_x) / self.resolution)
        r = int((y - self.origin_y) / self.resolution)
        return r, c

    def grid_to_world(self, r, c):
        x = self.origin_x + (c + 0.5) * self.resolution
        y = self.origin_y + (r + 0.5) * self.resolution
        return x, y

    def is_free(self, r, c):
        return (0 <= r < self.height and
                0 <= c < self.width and
                self.grid[r, c] == 0)

    def plan(self, sx, sy, gx, gy):
        """A* from (sx, sy) to (gx, gy) in world coordinates."""
        start = self.world_to_grid(sx, sy)
        goal = self.world_to_grid(gx, gy)

        # Clamp to grid bounds
        goal = (
            max(0, min(self.height - 1, goal[0])),
            max(0, min(self.width - 1, goal[1]))
        )
        start = (
            max(0, min(self.height - 1, start[0])),
            max(0, min(self.width - 1, start[1]))
        )

        # If start or goal lands in an obstacle, nudge to nearest free cell
        if not self.is_free(*start):
            print(f"  ⚠️  Start ({sx:.2f},{sy:.2f}) is in obstacle — finding nearest free cell")
            start = self._nearest_free(start)
            if start is None:
                print("  ❌  Cannot find free start cell!")
                return None

        if not self.is_free(*goal):
            print(f"  ⚠️  Goal ({gx:.2f},{gy:.2f}) is in obstacle — finding nearest free cell")
            goal = self._nearest_free(goal)
            if goal is None:
                print("  ❌  Cannot find free goal cell!")
                return None

        # Pre-compute wall-proximity cost map: cells close to obstacles
        # cost more to traverse, pushing paths toward corridor centers.
        wall_cost_map = self._compute_wall_proximity_cost()

        # A* with 8-connectivity
        open_set = []
        heapq.heappush(open_set, (0.0, start))
        came_from = {start: None}
        g = {start: 0.0}
        dirs = [(0, 1), (1, 0), (0, -1), (-1, 0),
                (1, 1), (1, -1), (-1, 1), (-1, -1)]

        while open_set:
            _, cur = heapq.heappop(open_set)
            if cur == goal:
                break
            for dr, dc in dirs:
                nxt = (cur[0] + dr, cur[1] + dc)
                if not self.is_free(*nxt):
                    continue

                # Penalty for unknown space (low during exploration, high during user navigation)
                unknown_penalty = self.unknown_penalty if self.raw[nxt[0], nxt[1]] == -1 else 0.0

                # Wall-proximity penalty: cells near obstacles cost more,
                # naturally centering paths in corridors.
                wall_penalty = wall_cost_map[nxt[0], nxt[1]]

                step = (1.414 if dr and dc else 1.0) + unknown_penalty + wall_penalty
                ng = g[cur] + step
                if nxt not in g or ng < g[nxt]:
                    g[nxt] = ng
                    f = ng + math.hypot(goal[0] - nxt[0], goal[1] - nxt[1])
                    heapq.heappush(open_set, (f, nxt))
                    came_from[nxt] = cur

        if goal not in came_from:
            print("  ❌  No path found!")
            return None

        path, cur = [], goal
        while cur is not None:
            path.append(self.grid_to_world(*cur))
            cur = came_from[cur]
        path.reverse()
        return path

    def _nearest_free(self, cell):
        """BFS outward to find the nearest free cell."""
        visited = {cell}
        queue = [cell]
        while queue:
            next_queue = []
            for r, c in queue:
                for dr in range(-1, 2):
                    for dc in range(-1, 2):
                        nb = (r + dr, c + dc)
                        if nb not in visited:
                            if self.is_free(*nb):
                                return nb
                            visited.add(nb)
                            next_queue.append(nb)
            queue = next_queue
        return None

    def _simplify(self, path):
        """Remove collinear intermediate waypoints."""
        if len(path) < 3:
            return path
        result = [path[0]]
        for i in range(1, len(path) - 1):
            dx1 = path[i][0] - path[i - 1][0]
            dy1 = path[i][1] - path[i - 1][1]
            dx2 = path[i + 1][0] - path[i][0]
            dy2 = path[i + 1][1] - path[i][1]
            if abs(dx1 * dy2 - dx2 * dy1) > 1e-4:
                result.append(path[i])
        result.append(path[-1])
        return result

    def _line_of_sight(self, r0, c0, r1, c1):
        """
        Bresenham line-of-sight check between (r0,c0) and (r1,c1).
        Returns True if all cells on the line are free.
        """
        dr = abs(r1 - r0)
        dc = abs(c1 - c0)
        sr = 1 if r1 > r0 else -1
        sc = 1 if c1 > c0 else -1

        r, c = r0, c0

        if dc > dr:
            err = dc // 2
            while c != c1:
                if not self.is_free(r, c):
                    return False
                err -= dr
                if err < 0:
                    r += sr
                    err += dc
                c += sc
        else:
            err = dr // 2
            while r != r1:
                if not self.is_free(r, c):
                    return False
                err -= dc
                if err < 0:
                    c += sc
                    err += dr
                r += sr

        return self.is_free(r1, c1)

    def plan_theta_star(self, sx, sy, gx, gy):
        """
        Theta* path planning — any-angle A* with line-of-sight shortcuts.
        Produces smoother paths than A* by allowing straight-line connections
        between non-adjacent cells when there's clear line-of-sight.

        The sparse Theta* path is then interpolated to produce dense waypoints
        needed for collision checking during flight.
        """
        start = self.world_to_grid(sx, sy)
        goal = self.world_to_grid(gx, gy)

        # Clamp to grid bounds
        goal = (
            max(0, min(self.height - 1, goal[0])),
            max(0, min(self.width - 1, goal[1]))
        )
        start = (
            max(0, min(self.height - 1, start[0])),
            max(0, min(self.width - 1, start[1]))
        )

        if not self.is_free(*start):
            print(f"  \u26a0\ufe0f  Start ({sx:.2f},{sy:.2f}) is in obstacle \u2014 finding nearest free cell")
            start = self._nearest_free(start)
            if start is None:
                print("  \u274c  Cannot find free start cell!")
                return None

        if not self.is_free(*goal):
            print(f"  \u26a0\ufe0f  Goal ({gx:.2f},{gy:.2f}) is in obstacle \u2014 finding nearest free cell")
            goal = self._nearest_free(goal)
            if goal is None:
                print("  \u274c  Cannot find free goal cell!")
                return None

        wall_cost_map = self._compute_wall_proximity_cost()

        open_set = []
        heapq.heappush(open_set, (0.0, start))
        came_from = {start: None}
        g = {start: 0.0}
        dirs = [(0, 1), (1, 0), (0, -1), (-1, 0),
                (1, 1), (1, -1), (-1, 1), (-1, -1)]

        while open_set:
            _, cur = heapq.heappop(open_set)
            if cur == goal:
                break
            for dr, dc in dirs:
                nxt = (cur[0] + dr, cur[1] + dc)
                if not self.is_free(*nxt):
                    continue

                unknown_penalty = 3.0 if self.raw[nxt[0], nxt[1]] == -1 else 0.0
                wall_penalty = wall_cost_map[nxt[0], nxt[1]]

                # Theta* shortcut: check if grandparent has line-of-sight
                parent = came_from.get(cur)
                if (parent is not None
                        and self._line_of_sight(parent[0], parent[1],
                                                nxt[0], nxt[1])):
                    # Direct connection from grandparent to neighbor
                    new_g = g[parent] + math.hypot(
                        nxt[0] - parent[0], nxt[1] - parent[1]
                    ) + unknown_penalty + wall_penalty
                    use_parent = parent
                else:
                    # Standard A* connection through cur
                    step = (1.414 if dr and dc else 1.0) + unknown_penalty + wall_penalty
                    new_g = g[cur] + step
                    use_parent = cur

                if nxt not in g or new_g < g[nxt]:
                    g[nxt] = new_g
                    f = new_g + math.hypot(goal[0] - nxt[0], goal[1] - nxt[1])
                    heapq.heappush(open_set, (f, nxt))
                    came_from[nxt] = use_parent

        if goal not in came_from:
            print("  \u274c  No Theta* path found!")
            return None

        # Reconstruct sparse path
        sparse_path, cur = [], goal
        while cur is not None:
            sparse_path.append(self.grid_to_world(*cur))
            cur = came_from[cur]
        sparse_path.reverse()

        # Interpolate for dense collision checking
        return self._interpolate_path(sparse_path, spacing=0.1)

    def _interpolate_path(self, path, spacing=0.1):
        """
        Interpolate a sparse path to add intermediate points at `spacing` metre
        intervals. This gives the dynamic obstacle checker enough granularity
        while preserving Theta*'s smooth straight-line segments.
        """
        if len(path) < 2:
            return path

        dense = [path[0]]
        for i in range(1, len(path)):
            x0, y0 = path[i - 1]
            x1, y1 = path[i]
            seg_len = math.hypot(x1 - x0, y1 - y0)
            if seg_len < 1e-6:
                continue
            n_steps = max(1, int(seg_len / spacing))
            for s in range(1, n_steps + 1):
                t = s / n_steps
                dense.append((x0 + t * (x1 - x0), y0 + t * (y1 - y0)))

        return dense

    def _compute_wall_proximity_cost(self):
        """
        Build a cost map where cells near obstacles have extra traversal cost.
        This makes A* naturally prefer corridor centers over wall-grazing paths.

        Cells at distance d from nearest obstacle get penalty:
          d=1: 4.0, d=2: 2.0, d=3: 0.5, d>=4: 0.0
        """
        cost_map = np.zeros((self.height, self.width), dtype=np.float32)
        obstacles = np.argwhere(self.grid == 1)

        if len(obstacles) == 0:
            return cost_map

        # Distance penalties: index = distance in cells, value = cost
        penalties = {1: 4.0, 2: 2.0, 3: 0.5}
        max_dist = 3

        for r, c in obstacles:
            for d in range(1, max_dist + 1):
                r_lo = max(0, r - d)
                r_hi = min(self.height, r + d + 1)
                c_lo = max(0, c - d)
                c_hi = min(self.width, c + d + 1)
                # Apply penalty to the ring at distance d
                # (np.maximum ensures we keep the highest penalty if overlapping)
                cost_map[r_lo:r_hi, c_lo:c_hi] = np.maximum(
                    cost_map[r_lo:r_hi, c_lo:c_hi], penalties[d])

        # Zero out costs on obstacle cells themselves (they're impassable anyway)
        cost_map[self.grid == 1] = 0.0
        return cost_map

    def nearest_obstacle_world_dist(self, wx, wy):
        """
        Return the approximate distance (in metres) from world position
        (wx, wy) to the nearest REAL obstacle cell (raw grid, NOT inflated).
        Used for reactive safety checks — must measure from actual walls,
        not from inflated zones which would cause false positives.
        """
        r, c = self.world_to_grid(wx, wy)
        if not (0 <= r < self.height and 0 <= c < self.width):
            return 0.0  # Out of bounds = unsafe

        # Build raw obstacle mask (not inflated)
        raw_obstacles = (self.raw > 50)

        # BFS outward to find nearest real obstacle
        for radius in range(0, 15):
            for dr in range(-radius, radius + 1):
                for dc in range(-radius, radius + 1):
                    if abs(dr) != radius and abs(dc) != radius:
                        continue
                    nr, nc = r + dr, c + dc
                    if (0 <= nr < self.height and 0 <= nc < self.width
                            and raw_obstacles[nr, nc]):
                        return radius * self.resolution
        return 15.0 * self.resolution  # Far from obstacles


# ─────────────────────────────────────────────────────────────────────────────
#  AUTONOMOUS NAVIGATOR NODE
# ─────────────────────────────────────────────────────────────────────────────
class AutonomousNavigator(Node):
    """
    State Machine:
      INIT       → waiting for PX4 + Gazebo data
      TAKEOFF    → arming and ascending to flight altitude
      EXPLORING  → flying scan waypoints + rotating at each
      READY      → map built, hovering, waiting for user goal
      NAVIGATING → following A* path to goal
    """

    def __init__(self):
        super().__init__('autonomous_navigator')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5
        )

        # World name selects the Gazebo pose topic. launch_sim.sh bridges
        # house_3room, launch_sim_360.sh bridges house_2floor:
        #   --ros-args -p world:=house_3room
        self.declare_parameter('world', 'house_2floor')
        world = str(self.get_parameter('world').value)
        self._viz_world = world

        # ── Subscribers ──
        self.gz_sub = self.create_subscription(
            TFMessage, f'/world/{world}/dynamic_pose/info',
            self.gz_pose_cb, qos)
        self.odom_sub = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry',
            self.odom_cb, qos)
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose',
            self.goal_cb, 10)
        # RViz "Publish Point" raycasts against rendered geometry, so the
        # click carries a real z — unlike "2D Goal Pose", which is always z=0.
        self.click_sub = self.create_subscription(
            PointStamped, '/clicked_point',
            self.clicked_cb, 10)
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/projected_map',
            self.map_cb, 10)
        self.pc2_sub = self.create_subscription(
            PointCloud2, '/octomap_point_cloud_centers',
            self.pc2_cb, 10)

        # ── Publishers ──
        self.path_pub = self.create_publisher(Path, '/plan', 10)
        # Latched state topic so late-joining nodes (e.g. semantic pipeline)
        # immediately learn the current phase
        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1)
        self.nav_state_pub = self.create_publisher(String, '/nav_state', state_qos)
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos)

        # ── State ──
        self.gz_pos = None       # Gazebo ENU [x, y, z]
        self.px4_pos = None      # PX4 NED  [x, y, z]
        self.occupancy_grid = None  # Latest OccupancyGrid from OctoMap
        self.planner_cache = None   # Latest SensorMapPlanner instance for quick collision checks
        self.octomap_3d = OctoMap3DQuery(resolution=0.10)  # 3D voxel query

        self._set_state('INIT')  # Also broadcasts on latched /nav_state
        self.waypoints = []      # Current path waypoints: [(x,y,z), ...]
        self.wp_idx = 0
        self.goal_xy = None      # Store goal for replanning
        self.goal_layer = None   # ...and its layer, so a replan stays on the right floor
        self.current_alt = 1.8   # Current target altitude (smoothed)

        # Frontier exploration state
        self.scan_yaw = 0.0           # Current rotation angle during 360° scan
        self.scan_rotating = False
        self.frontier_target = None   # Current frontier goal (x, y) or None
        self.frontier_path = []       # A* path to current frontier
        self.frontier_wp_idx = 0      # Index into frontier_path
        self.frontier_scans = 0       # Total 360° scans completed
        # Frontier exploration is now OPT-IN. Default: take off, hover, and
        # wait for a clicked goal. Enable with:
        #   --ros-args -p explore:=true
        self.declare_parameter('explore', False)
        self.EXPLORE_ON_START = bool(self.get_parameter('explore').value)
        self.frontier_phase = 'INITIAL_SCAN'  # INITIAL_SCAN → FINDING → FLYING → SCANNING → RETREATING
        self.last_frontier_time = 0.0 # Throttle frontier extraction (expensive)
        self.FRONTIER_INTERVAL = 3.0  # Re-extract frontiers every N seconds
        self.no_frontier_count = 0    # Consecutive times no frontier found
        self.NO_FRONTIER_LIMIT = 3    # Declare done after N consecutive misses
        self.unreachable_frontiers = [] # List of (x,y) that A* failed to reach

        # Reactive safety / stuck detection state
        self.retreat_target = None          # (px4_x, px4_y) to retreat to
        self.retreat_start_time = 0.0       # When retreat started
        self.RETREAT_DURATION = 2.0         # Seconds to hold retreat position
        self.last_position_time = self._now()  # For stuck detection (sim time)
        self.last_position_xy = None        # Last recorded position (Gazebo ENU)
        self.STUCK_TIME_LIMIT = 40.0
        self.STUCK_MOVE_THRESHOLD = 0.15
        self.replan_timestamps = []         # Timestamps of recent replans
        self.MAX_REPLANS_WINDOW = 3         # Max replans within REPLAN_WINDOW_SEC
        self.REPLAN_WINDOW_SEC = 10.0       # Window for counting replans

        # Parameters
        self.ALTITUDE = 1.8          # Flight altitude (metres)
        # Altitude to hover at in READY. Starts at ALTITUDE and follows the
        # last goal reached, so after flying upstairs the drone stays there
        # instead of sinking back to 1.8 m (fixed 2026-11-17).
        self.hold_alt = self.ALTITUDE
        # Crash response (2026-11-17 flight: flipped against the atrium railing
        # at 380 s and tumbled with motors on for 45 s while this node kept
        # sending setpoints and replanning). Tilt above CRASH_TILT_DEG for
        # CRASH_HOLD_S of sim time -> force-disarm and stop commanding.
        self.tilt_deg = 0.0
        self.CRASH_TILT_DEG = 60.0
        self.CRASH_HOLD_S = 0.25
        self._tilt_since = None
        self.WP_REACH_DIST = 0.25    # Slightly more forgiving (was 0.15)
        self.SCAN_WP_REACH = 0.5     # Distance to consider scan position reached
        self.YAW_STEP = 0.03         # Slower rotation = better scan quality (was 0.04)
        self.SAFETY_MARGIN = 0.40     # Obstacle inflation for A* (was 0.20)
        # Doorway clearance: 2.5m - 2*(0.075 wall + 0.40 inflate) = 1.55m gap

        # Stability: velocity clamping + yaw smoothing
        self.MAX_NAV_DELTA = 1.5     # Max target distance from drone (metres)
        self.current_yaw = 0.0       # Smoothed yaw angle (radians)
        self.YAW_SMOOTH = 0.15       # Yaw interpolation factor (0=no turn, 1=snap)
        self.ALT_SMOOTH = 0.10       # Altitude interpolation factor
        self.MAX_ALT_DELTA = 0.5     # Max altitude change per tick (metres)

        # Obstacle avoidance: check path every N ticks
        self.AVOIDANCE_CHECK_INTERVAL = 40  # Check every 40 ticks (2 seconds at 20Hz)
        self.avoidance_tick = 0

        # ── AHPPA: dwell + deadlock timing on SIM time, same as the rest ──
        if _ahppa is not None:
            _ahppa.set_clock(self._now)
            print(f"  [portfolio] clock: "
                  f"{'sim' if self._uses_sim_time() else 'WALL (use_sim_time is false)'}")

        # ── AHPPA v3: fast "commanded forward, not moving" trigger ──
        self._stuck_hist = []         # (t, x, y, z) since the current path was set
        self._stuck_wps = None        # the waypoint list the history belongs to
        self._tilt_hist = []          # (t, tilt_deg, x, y, z), last IMPACT_HIST_S
        self._recover = None          # dict(target ENU, until) while backing off

        # ── Control timer: 20 Hz ──
        self.create_timer(0.05, self.control_loop)

        # ── Demo overlay (AHPPA_VIZ=1): drone, trail, unseen obstacles and the
        #    contact memory as RViz markers. Display only: nothing here feeds
        #    back into planning or control. ──
        self._viz_kind, self._viz_trail = None, []
        if os.environ.get('AHPPA_VIZ', '0') == '1':
            self.viz_pub = self.create_publisher(MarkerArray, '/ahppa/viz', 10)
            self.create_timer(0.2, self._viz_tick)

        print("\n╔══════════════════════════════════════════════════════════╗")
        print("║   🧠 AUTONOMOUS NAVIGATOR v6 — SAFE FRONTIER EXPLORER  ║")
        print("╠══════════════════════════════════════════════════════════╣")
        print("║  Phase 1: Takeoff → 1.8m                               ║")
        print("║  Phase 2: Frontier-based exploration (autonomous)       ║")
        print("║           Reactive safety + stuck detection + retreat   ║")
        print("║           Stops when no frontiers remain (room mapped)  ║")
        print("║  Phase 3: Click '2D Goal Pose' in RViz                  ║")
        print("║  Phase 4: A* plans obstacle-free path → flies!          ║")
        print("╚══════════════════════════════════════════════════════════╝\n")

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def gz_pose_cb(self, msg):
        """Get drone's ground-truth position from Gazebo."""
        for tf in msg.transforms:
            if 'x500' in tf.child_frame_id.lower():
                t = tf.transform.translation
                self.gz_pos = np.array([t.x, t.y, t.z])
                return
        if msg.transforms:
            t = msg.transforms[0].transform.translation
            self.gz_pos = np.array([t.x, t.y, t.z])

    def odom_cb(self, msg):
        self.px4_pos = np.array([msg.position[0], msg.position[1], msg.position[2]])
        w, x, y, z = (float(v) for v in msg.q)
        if not math.isnan(w):
            # angle between the body z axis and the world vertical
            c = max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y)))
            self.tilt_deg = math.degrees(math.acos(c))

    def map_cb(self, msg):
        """Store the latest projected OccupancyGrid from OctoMap and update planner cache."""
        self.occupancy_grid = msg
        try:
            self.planner_cache = SensorMapPlanner(msg, safety_margin=self.SAFETY_MARGIN)
        except Exception:
            pass

    def pc2_cb(self, msg):
        """Update the 3D OctoMap query from PointCloud2 voxel centers."""
        self.octomap_3d.update_from_pointcloud2(msg)

    def clicked_cb(self, msg):
        """RViz Publish Point -> 3D goal. Click a voxel on the floor you want."""
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose.position.x = msg.point.x
        ps.pose.position.y = msg.point.y
        ps.pose.position.z = msg.point.z
        ps.pose.orientation.w = 1.0
        lyr = layer_for_z(msg.point.z)
        print(f"\n  3D goal clicked: ({msg.point.x:.2f}, {msg.point.y:.2f}, "
              f"z={msg.point.z:.2f})  ->  layer {lyr} (z={Z_LAYERS[lyr]:.1f} m)")
        self.goal_cb(ps)

    def goal_cb(self, msg):
        """Handle user clicking '2D Goal Pose' in RViz."""
        if self.state != 'READY':
            print("⚠️  Drone is not ready yet! Wait for exploration to complete.")
            return

        if self.occupancy_grid is None:
            print("⚠️  No sensor map available! The OctoMap hasn't built yet.")
            return

        if self.gz_pos is None:
            print("⚠️  No Gazebo position received.")
            return

        gx = msg.pose.position.x
        gy = msg.pose.position.y
        g_layer = None

        print(f"\n🎯  Goal received: ({gx:.2f}, {gy:.2f})")
        print(f"   Drone is at: ({self.gz_pos[0]:.2f}, {self.gz_pos[1]:.2f}, z={self.gz_pos[2]:.2f})")

        # Try 2.5D planning if 3D data is available, else fall back to 2D
        path = None
        if self.octomap_3d.is_ready:
            print(f"   Planning with 2.5D multi-layer planner (3D data: {self.octomap_3d.num_voxels} voxels)...")
            self.octomap_3d.debug_summary()
            try:
                planner = MultiLayerPlanner(self.occupancy_grid, self.octomap_3d,
                                            safety_margin=self.SAFETY_MARGIN, unknown_penalty=50.0)
                # Use standard A* (dense path) instead of Theta* (sparse path)
                gz = msg.pose.position.z
                s_layer = layer_for_z(self.gz_pos[2])
                g_layer = layer_for_z(gz) if gz > 0.3 else None
                if g_layer is not None:
                    print(f"   start layer {s_layer} (z={Z_LAYERS[s_layer]:.1f}) "
                          f"-> goal layer {g_layer} (z={Z_LAYERS[g_layer]:.1f})")
                path = self._plan_layers(planner, self.gz_pos[0], self.gz_pos[1], s_layer,
                                         gx, gy, g_layer)
                self._viz_kind = getattr(planner, 'last_memory', None)
            except Exception as e:
                print(f"   ⚠️  2.5D planner failed ({e}), falling back to 2D...")
                path = None

        if path is None and not self._fallback_ok(g_layer):
            print("   ❌ No 2.5D path, and the 2D fallback is refused above the ground floor. "
                  "Hovering.")
        elif path is None:
            print(f"   Planning with 2D A* (fallback)...")
            planner_2d = SensorMapPlanner(self.occupancy_grid, safety_margin=self.SAFETY_MARGIN, unknown_penalty=50.0)
            path_2d = planner_2d.plan(self.gz_pos[0], self.gz_pos[1], gx, gy)
            if path_2d:
                # Convert 2D path to 3D by adding default altitude
                path = [(x, y, self.ALTITUDE) for x, y in path_2d]

        if path:
            self.waypoints = densify_path(path)
            self.wp_idx = 0
            self.goal_xy = (gx, gy)
            self.goal_layer = g_layer if self.octomap_3d.is_ready else None
            self._set_state('NAVIGATING')
            self._publish_rviz_path(path)
            # Count altitude changes
            alt_changes = sum(1 for i in range(1, len(path))
                              if abs(path[i][2] - path[i-1][2]) > 0.1)
            print(f"   ✅ Path found — {len(path)} waypoints, {alt_changes} altitude changes. Flying!")
        else:
            print("   ❌ No path found. Try a different target.")

    # ── Control Loop ──────────────────────────────────────────────────────────────────

    def _uses_sim_time(self):
        try:
            return bool(self.get_parameter('use_sim_time').value)
        except Exception:
            return False

    def _now(self):
        """Seconds on the node clock: SIM time when use_sim_time:=true.
        Every timer in the navigator goes through this so stuck detection,
        retreat and frontier throttling all scale with the real-time factor.
        Before the first /clock message sim time reads 0; fall back to wall
        time only for that startup window so nothing waits forever."""
        t = self.get_clock().now().nanoseconds / 1e9
        return t if t > 0.0 else time.monotonic()

    def _set_state(self, new_state):
        """Change navigator state and broadcast it on /nav_state (latched)."""
        self.state = new_state
        msg = String()
        msg.data = new_state
        self.nav_state_pub.publish(msg)

    def _check_crash(self):
        """True (and the vehicle killed) once tilt stays past the limit."""
        if self.state in ('INIT', 'CRASHED'):
            return self.state == 'CRASHED'
        if self.tilt_deg < self.CRASH_TILT_DEG:
            self._tilt_since = None
            return False
        now = self._now()
        if self._tilt_since is None:
            self._tilt_since = now
            return False
        if now - self._tilt_since < self.CRASH_HOLD_S:
            return False
        print(f"\n  💥 CRASH: tilt {self.tilt_deg:.0f}° for {now - self._tilt_since:.2f} s "
              f"- force-disarming, no further setpoints or replans.")
        # MAV_CMD_COMPONENT_ARM_DISARM, param1 = 0 (disarm), param2 = 21196 (force)
        self._send_command(400, param1=0.0, param2=21196.0)
        self.waypoints = []
        self.goal_xy = None
        self.goal_layer = None
        self._set_state('CRASHED')
        return True

    def control_loop(self):
        if self.px4_pos is None or self.gz_pos is None:
            return
        if self._check_crash():
            return            # CRASHED: no heartbeat, no setpoints, no replans

        if self.state == 'INIT':
            self._takeoff()
            return

        # Always send heartbeat when airborne
        self._heartbeat()

        if self.state == 'EXPLORING':
            self._explore_step()

        elif self.state == 'NAVIGATING':
            if _ahppa is not None:
                if self._recovering():
                    return                 # backing off after an impact
                if self._impact():
                    return
                _ahppa.observe(*self.gz_pos, goal_active=True)
                if _ahppa.deadlock_edge():
                    print(f"  🔒 [portfolio] DEADLOCK: moved < "
                          f"{_ahppa.WATCHDOG.min_disp:.2f} m in "
                          f"{_ahppa.WATCHDOG.window_s:.1f} s — replanning in ESCAPE")
                    h = self._stuck_heading()
                    if h is not None and self.gz_pos[2] >= self.STUCK_MIN_Z:
                        _ahppa.report_stuck(self.gz_pos[0], self.gz_pos[1], *h, source='watchdog')
                    self._replan()
                    return
                if self._fast_stuck():
                    self._replan()
                    return
            self._navigate_step()

        elif self.state == 'READY':
            if _ahppa is not None:
                _ahppa.observe(*self.gz_pos, goal_active=False)
            # Hover in place, at the altitude of the last goal reached
            self._send_position(self.px4_pos[0], self.px4_pos[1], -self.hold_alt)

    # ── Exploration Phase ─────────────────────────────────────────────────────

    def _explore_step(self):
        """Frontier-based autonomous exploration.

        State machine:
          INITIAL_SCAN → 360° at spawn to seed the OctoMap
          FINDING      → extract frontiers, pick best, plan A* path
          FLYING       → follow A* path to frontier centroid
          SCANNING     → 360° rotation at the frontier
          → back to FINDING (or DONE if no frontiers left)
        """

        # ── INITIAL_SCAN: 360° rotation at spawn to seed the map ──
        if self.frontier_phase == 'INITIAL_SCAN':
            self.scan_yaw += self.YAW_STEP
            self._send_position(
                self.px4_pos[0], self.px4_pos[1], -self.ALTITUDE,
                yaw=self.scan_yaw
            )
            if self.scan_yaw >= 2 * math.pi:
                self.scan_rotating = False
                self.frontier_scans = 1
                self.frontier_phase = 'FINDING'
                print("  ✅ Initial scan complete. Looking for frontiers...")
            return

        # ── FINDING: extract frontiers and plan a path ──
        if self.frontier_phase == 'FINDING':
            # Throttle: don't re-extract every tick
            now = self._now()
            if now - self.last_frontier_time < self.FRONTIER_INTERVAL:
                # Hover while waiting
                self._send_position(self.px4_pos[0], self.px4_pos[1], -self.ALTITUDE)
                return
            self.last_frontier_time = now

            if self.occupancy_grid is None:
                print("  ⏳ Waiting for OctoMap /projected_map...")
                self._send_position(self.px4_pos[0], self.px4_pos[1], -self.ALTITUDE)
                return

            # Extract and rank frontiers (ignoring known unreachable ones)
            frontiers = FrontierExtractor.extract(
                self.occupancy_grid, self.gz_pos[0], self.gz_pos[1],
                unreachable=self.unreachable_frontiers)

            if not frontiers:
                self.no_frontier_count += 1
                print(f"  🔍 No frontiers found ({self.no_frontier_count}/{self.NO_FRONTIER_LIMIT})")

                if self.no_frontier_count >= self.NO_FRONTIER_LIMIT:
                    # Exploration complete!
                    self._exploration_complete()
                    return
                else:
                    # Wait and try again
                    return

            # Reset miss counter on success
            self.no_frontier_count = 0

            print(f"\n  🗺️  Found {len(frontiers)} frontier clusters")
            
            # Try frontiers in order of score until we find one with a valid path
            path = None
            planner = SensorMapPlanner(self.occupancy_grid,
                                      safety_margin=self.SAFETY_MARGIN)
            
            for rank, (fx, fy, score, size) in enumerate(frontiers):
                print(f"     Option {rank+1}: ({fx:.1f}, {fy:.1f}) — "
                      f"{size} cells, score={score:.1f}")
                
                try:
                    path = planner.plan(
                        self.gz_pos[0], self.gz_pos[1],
                        fx, fy)
                except Exception as e:
                    print(f"       ❌ A* planning failed: {e}")
                    path = None

                if path and len(path) >= 2:
                    self.frontier_target = (fx, fy)
                    break
                else:
                    print(f"       ❌ No path to frontier. Blacklisting it.")
                    self.unreachable_frontiers.append((fx, fy))
                    path = None

            if path:
                self.frontier_path = densify_path(path)
                self.frontier_wp_idx = 1  # Skip the start point
                self.frontier_phase = 'FLYING'
                self._publish_rviz_path(path)
                print(f"     ✅ Path planned — {len(path)} waypoints. Flying!")
            else:
                print(f"     ❌ All frontiers unreachable. Skipping tick.")
                self.last_frontier_time = self._now() - self.FRONTIER_INTERVAL + 1.0
            return

        # ── FLYING: follow A* path to the frontier centroid ──
        if self.frontier_phase == 'FLYING':
            # 🛡️ REACTIVE SAFETY CHECK: Is the drone itself too close to a REAL obstacle?
            # Uses raw (uninflated) grid so we measure from actual walls, not inflated zones.
            if self.planner_cache is not None:
                obs_dist = self.planner_cache.nearest_obstacle_world_dist(
                    self.gz_pos[0], self.gz_pos[1])
                if obs_dist < 0.25:  # Within 25cm of a REAL wall!
                    print(f"  🚨 REACTIVE SAFETY: Drone is {obs_dist:.2f}m from real obstacle! "
                          f"Blacklisting frontier and retreating...")
                    # Blacklist THIS frontier so we don't keep returning to it
                    if self.frontier_target:
                        self.unreachable_frontiers.append(self.frontier_target)
                    self._initiate_retreat()
                    return

            # ── Stuck detection: if drone hasn't moved enough, retreat + new frontier
            now = self._now()
            if self.last_position_xy is not None:
                moved = math.hypot(
                    self.gz_pos[0] - self.last_position_xy[0],
                    self.gz_pos[1] - self.last_position_xy[1])
                if moved > self.STUCK_MOVE_THRESHOLD:
                    self.last_position_xy = (self.gz_pos[0], self.gz_pos[1])
                    self.last_position_time = now
                elif now - self.last_position_time > self.STUCK_TIME_LIMIT:
                    print(f"  🔒 STUCK DETECTED: No movement for {self.STUCK_TIME_LIMIT}s! "
                          f"Blacklisting frontier and retreating...")
                    if self.frontier_target:
                        self.unreachable_frontiers.append(self.frontier_target)
                    self._initiate_retreat()
                    return
            else:
                self.last_position_xy = (self.gz_pos[0], self.gz_pos[1])
                self.last_position_time = now

            # 1. Advance the actual path progress index to the closest waypoint ahead
            min_d = float('inf')
            best_idx = self.frontier_wp_idx
            for i in range(self.frontier_wp_idx, min(len(self.frontier_path), self.frontier_wp_idx + 20)):
                wp = self.frontier_path[i]
                d = math.hypot(wp[0] - self.gz_pos[0], wp[1] - self.gz_pos[1])
                if d < min_d:
                    min_d = d
                    best_idx = i
            self.frontier_wp_idx = best_idx

            # Check if we reached the end of the path
            if self.frontier_wp_idx >= len(self.frontier_path) - 1:
                final_wp = self.frontier_path[-1]
                d_final = math.hypot(final_wp[0] - self.gz_pos[0], final_wp[1] - self.gz_pos[1])
                if d_final < self.WP_REACH_DIST:
                    # Arrived at frontier — start scanning
                    self.frontier_phase = 'SCANNING'
                    self.scan_rotating = True
                    self.scan_yaw = 0.0
                    self.frontier_scans += 1
                    tx, ty = self.frontier_target
                    print(f"  📍 Frontier #{self.frontier_scans} reached "
                          f"at ({tx:.1f}, {ty:.1f}) — rotating 360°...")
                    return

            # 🛡️ Dynamic safety check: Ensure the path ahead is still free!
            if self.planner_cache is not None:
                end_idx = min(len(self.frontier_path), self.frontier_wp_idx + 10)
                for check_wp in self.frontier_path[self.frontier_wp_idx : end_idx]:
                    r, c = self.planner_cache.world_to_grid(check_wp[0], check_wp[1])
                    if not self.planner_cache.is_free(r, c):
                        print(f"  ⚠️ Path blocked by new obstacle! Aborting and replanning...")
                        # Track replans — too many in a short window = blacklist frontier
                        self.replan_timestamps.append(now)
                        self.replan_timestamps = [
                            t for t in self.replan_timestamps
                            if now - t < self.REPLAN_WINDOW_SEC]
                        if len(self.replan_timestamps) >= self.MAX_REPLANS_WINDOW:
                            print(f"  🚫 Too many replans ({len(self.replan_timestamps)} in "
                                  f"{self.REPLAN_WINDOW_SEC}s)! Blacklisting frontier.")
                            if self.frontier_target:
                                self.unreachable_frontiers.append(self.frontier_target)
                            self.replan_timestamps.clear()
                        self.frontier_phase = 'FINDING'
                        return

            # 2. Pure Pursuit: Look ahead for a stable yaw/velocity target
            LOOKAHEAD_DIST = 0.3  # Short lookahead prevents corner-cutting (was 0.8)
            target_wp = self.frontier_path[self.frontier_wp_idx]
            for i in range(self.frontier_wp_idx, len(self.frontier_path)):
                wp = self.frontier_path[i]
                d = math.hypot(wp[0] - self.gz_pos[0], wp[1] - self.gz_pos[1])
                target_wp = wp
                if d >= LOOKAHEAD_DIST:
                    break

            wx, wy = target_wp[0], target_wp[1]
            dx_enu = wx - self.gz_pos[0]
            dy_enu = wy - self.gz_pos[1]
            dist = math.hypot(dx_enu, dy_enu)

            # Velocity clamping with deceleration near frontier target
            final_wp = self.frontier_path[-1]
            dist_to_goal = math.hypot(
                final_wp[0] - self.gz_pos[0],
                final_wp[1] - self.gz_pos[1])

            # Base speed: 0.3 m/s (safe for indoor corridors, was 0.5)
            # Decelerate to 0.15 m/s when within 1.0m of the frontier target
            if dist_to_goal < 1.0:
                max_speed = 0.15
            else:
                max_speed = 0.3

            if dist > max_speed:
                scale = max_speed / dist
                dx_enu *= scale
                dy_enu *= scale

            fly_yaw = math.atan2(dx_enu, dy_enu)
            tgt_px4_x = self.px4_pos[0] + dy_enu
            tgt_px4_y = self.px4_pos[1] + dx_enu
            self._send_position(tgt_px4_x, tgt_px4_y, -self.ALTITUDE, yaw=fly_yaw)
            return

        # ── RETREATING: back away from danger, then replan ──
        if self.frontier_phase == 'RETREATING':
            now = self._now()
            if self.retreat_target is not None:
                # Hold the retreat position for RETREAT_DURATION seconds
                self._send_position(
                    self.retreat_target[0], self.retreat_target[1], -self.ALTITUDE)
                if now - self.retreat_start_time >= self.RETREAT_DURATION:
                    print(f"  ✅ Retreat complete. Finding new frontier...")
                    self.retreat_target = None
                    self.frontier_phase = 'FINDING'
                    self.last_position_xy = None  # Reset stuck detector
                    self.last_position_time = now
            else:
                # Shouldn't happen, but fallback
                self.frontier_phase = 'FINDING'
            return

        # ── SCANNING: 360° rotation at the frontier ──
        if self.frontier_phase == 'SCANNING':
            # Hold position at the safe end of the A* path (NOT the un-nudged frontier target!)
            safe_wp = self.frontier_path[-1]
            tx, ty = safe_wp[0], safe_wp[1]
            dx_enu = tx - self.gz_pos[0]
            dy_enu = ty - self.gz_pos[1]
            tgt_px4_x = self.px4_pos[0] + dy_enu
            tgt_px4_y = self.px4_pos[1] + dx_enu

            self.scan_yaw += self.YAW_STEP
            self._send_position(
                tgt_px4_x, tgt_px4_y, -self.ALTITUDE,
                yaw=self.scan_yaw
            )

            if self.scan_yaw >= 2 * math.pi:
                # Scan done — go find the next frontier
                self.scan_rotating = False
                self.frontier_phase = 'FINDING'
                self.frontier_target = None
                self.frontier_path = []
                print(f"  ✅ Scan #{self.frontier_scans} complete. "
                      f"Looking for next frontier...")
            return

    def _initiate_retreat(self):
        """Back away from danger by retreating along the last safe path segment."""
        # Find a safe position to retreat to: go backward along the path
        retreat_wp = None
        if self.frontier_path and self.frontier_wp_idx > 2:
            # Go back 3-5 waypoints along the path we came from
            retreat_idx = max(0, self.frontier_wp_idx - 5)
            retreat_wp = self.frontier_path[retreat_idx]
        
        if retreat_wp is not None:
            # Convert to PX4 NED
            dx_enu = retreat_wp[0] - self.gz_pos[0]
            dy_enu = retreat_wp[1] - self.gz_pos[1]
            self.retreat_target = (
                self.px4_pos[0] + dy_enu,
                self.px4_pos[1] + dx_enu)
        else:
            # No path history — just hover in place
            self.retreat_target = (self.px4_pos[0], self.px4_pos[1])

        self.frontier_phase = 'RETREATING'
        self.retreat_start_time = self._now()
        self.frontier_target = None
        self.frontier_path = []

    def _exploration_complete(self):
        """Called when no frontiers remain — the room is fully mapped."""
        self._set_state('READY')
        has_map = self.occupancy_grid is not None
        if has_map:
            info = self.occupancy_grid.info
            occupied = sum(1 for v in self.occupancy_grid.data if v > 50)
            print(f"\n  ✅ FRONTIER EXPLORATION COMPLETE!")
            print(f"     {self.frontier_scans} autonomous scans performed")
            print(f"     Sensor map: {info.width}x{info.height} cells, "
                  f"{occupied} obstacles detected")
        else:
            print(f"\n  ⚠️  Exploration done but no map received from OctoMap!")
            print(f"     Check that Terminal 8 (OctoMap Server) is running.")
        print(f"     Click '2D Goal Pose' in RViz to set a target.\n")

    # ── Navigation Phase ──────────────────────────────────────────────────────

    def _plan_layers(self, planner, sx, sy, sl, gx, gy, gl):
        """Plan, doing any change of floor as a vertical leg in a clear column.

        Climbing: rise at the start, then fly level at the goal altitude.
        Descending: fly level at the current altitude, then drop at the goal.
        Either way the horizontal leg is flown at the HIGHER of the two
        altitudes, so the drone never crosses a railing, sill or atrium edge
        at railing height when it does not have to. On 2026-11-17 the layered
        planner put the climb near the goal and crossed the atrium railing at
        3.6 m (railing top 4.05 m) - into posts that had dropped out of the
        map. Falls back to the planner's own transitions when the column is
        not clear or the planner had to move the start/goal."""
        if getattr(self, 'viz_pub', None) is not None:
            self._viz_stock_path(planner, sx, sy, sl, gx, gy, gl)
        if gl is None or gl == sl:
            return planner.plan(sx, sy, gx, gy, start_layer=sl, goal_layer=gl)
        z0, z1 = Z_LAYERS[sl], Z_LAYERS[gl]
        lo, hi = min(z0, z1) - DRONE_CLEARANCE, max(z0, z1) + DRONE_CLEARANCE
        try:
            if gl > sl and self.octomap_3d.is_column_clear(sx, sy, lo, hi, xy_radius=0.4):
                leg = planner.plan(sx, sy, gx, gy, start_layer=gl, goal_layer=gl)
                if leg and math.hypot(leg[0][0] - sx, leg[0][1] - sy) < 0.25:
                    print(f"   climb first: {z0:.1f} -> {z1:.1f} m at the start, then level")
                    return [(sx, sy, z0)] + [tuple(p) for p in leg]
            elif gl < sl and self.octomap_3d.is_column_clear(gx, gy, lo, hi, xy_radius=0.4):
                leg = planner.plan(sx, sy, gx, gy, start_layer=sl, goal_layer=sl)
                if leg and math.hypot(leg[-1][0] - gx, leg[-1][1] - gy) < 0.25:
                    print(f"   descend last: level at {z0:.1f} m, then {z0:.1f} -> {z1:.1f} m at the goal")
                    return [tuple(p) for p in leg] + [(gx, gy, z1)]
        except Exception as e:
            print(f"   (vertical-leg planning failed: {e}; using layered transitions)")
        return planner.plan(sx, sy, gx, gy, start_layer=sl, goal_layer=gl)

    def _fallback_ok(self, goal_layer):
        """The 2D fallback plans at ALTITUDE (1.8 m) on the PROJECTED map, which
        flattens both floors. Upstairs it would fly into the slab (420.9 s,
        2026-11-17), so it is only allowed on the ground floor."""
        on_ground_floor = self.gz_pos is not None and self.gz_pos[2] < 2.6
        goal_down = goal_layer is None or Z_LAYERS[goal_layer] < 2.8
        return on_ground_floor and goal_down

    def _at_goal(self):
        """At the goal: within WP_REACH_DIST horizontally and 0.3 m vertically,
        and progress is within the last 0.3 m of the (densified) path.
        Requiring progress == LAST waypoint failed on a vertical descent:
        hovering 7 cm above the goal (normal PX4 settling) the closest point
        is the one 0.1 m above it, so the goal was never declared and the
        deadlock watchdog fired instead (re-fly, 2026-11-17, 210-214 s)."""
        if not self.waypoints:
            return False
        if self.wp_idx < len(self.waypoints) - 4:
            return False
        g = self.waypoints[-1]
        d = math.hypot(g[0] - self.gz_pos[0], g[1] - self.gz_pos[1])
        dz = abs(g[2] - self.gz_pos[2]) if len(g) == 3 else 0.0
        return d < self.WP_REACH_DIST and dz < 0.3

    def _wp_dist(self, wp):
        """Distance from the drone to a waypoint: 3D for (x, y, z) waypoints.
        Horizontal-only distance made a goal straight above the drone count as
        reached at once, and made every point of a vertical climb tie as
        'closest', so progress and lookahead never advanced up the climb."""
        if len(wp) == 3:
            return math.dist(wp, self.gz_pos[:3])
        return math.hypot(wp[0] - self.gz_pos[0], wp[1] - self.gz_pos[1])

    FAST_STUCK_S = 1.5       # v3: commanded forward but moved less than
    FAST_STUCK_M = 0.10      # FAST_STUCK_M (3D) in FAST_STUCK_S of sim time
    STUCK_MIN_Z = 0.5        # v3: no stuck barrier below this altitude (on the ground)

    IMPACT_TILT_DEG = 10.0   # v3: tilt past this ...
    CALM_TILT_DEG = 5.0      # ... after IMPACT_CALM_S below this = a bump
    IMPACT_CALM_S = 1.0
    IMPACT_HIST_S = 1.5
    RECOVER_BACK_M = 0.6     # back off this far, along the surface normal
    RECOVER_MIN_S = 2.0      # and hold at least this long, until level again
    RECOVER_STEADY_S = 1.0   # "level again" = tilt < CALM, speed < RECOVER_MAX_SPEED
    RECOVER_MAX_SPEED = 0.2  #   and at the hold altitude, for RECOVER_STEADY_S
    RECOVER_MAX_S = 10.0     # replan anyway after this long

    def _impact(self):
        """AHPPA v3: a sudden tilt while cruising is a collision with
        something the map does not hold. Glass flight test (glass_test.sdf,
        2026-11-19): at 0.3 m/s the tilt went 0.3 -> 18.7 deg in 0.1 s on
        contact, normal cruise stayed below 3.4 deg; the drone survived the
        first bump and crashed on the second, after replanning the same way.
        So on the FIRST bump: remember a barrier where it hit, back off, level
        out, then replan in ESCAPE. Returns True when it fired."""
        if not getattr(_ahppa, 'V3', False) or self.gz_pos is None:
            return False
        now = self._now()
        x, y, z = (float(v) for v in self.gz_pos[:3])
        self._tilt_hist.append((now, float(self.tilt_deg), x, y, z))
        while self._tilt_hist and now - self._tilt_hist[0][0] > self.IMPACT_HIST_S:
            self._tilt_hist.pop(0)
        if self.tilt_deg < self.IMPACT_TILT_DEG:
            return False
        calm = [h for h in self._tilt_hist if now - self.IMPACT_CALM_S - 0.3 <= h[0] <= now - 0.3]
        if not calm or calm[0][0] > now - self.IMPACT_CALM_S or max(h[1] for h in calm) >= self.CALM_TILT_DEG:
            return False                    # not after calm flight: not a fresh bump
        pre = calm[-1]                      # last calm pose, ~0.3 s before
        old = calm[0]
        hx, hy = pre[2] - old[2], pre[3] - old[3]
        if math.hypot(hx, hy) < 0.05:       # was hovering: use the path direction
            h = self._stuck_heading()
            if h is None:
                return False
            hx, hy = h
        n = math.hypot(hx, hy)
        print(f"  💢 [v3] IMPACT: tilt {pre[1]:.1f} -> {self.tilt_deg:.1f}° near "
              f"({pre[2]:.2f}, {pre[3]:.2f}) moving ({hx / n:+.2f}, {hy / n:+.2f}) — "
              f"something the map does not hold")
        # phase 1 (BOUNCE_S): do not fight the bounce - hold wherever the
        # drone is - and measure which way it is thrown: that is the surface
        # normal. Glass flight 3: grazing the pane while flying ALONG it, the
        # heading was +y but the bounce went mostly -x, away from the glass;
        # backing off opposite the heading kept it scraping and it flipped.
        self._recover = dict(phase='bounce', t0=now, until=now + self.BOUNCE_S,
                             pre=pre, heading=(hx / n, hy / n),
                             start=(x, y), target=(x, y, pre[4]))
        self._tilt_hist = []
        self._recovering()
        return True

    BOUNCE_S = 0.3           # v3: watch the bounce this long to find the surface normal

    def _bounce_done(self, r, now):
        """End of the bounce phase: barrier across the HEADING, back off
        opposite it. (Taking the bounce direction as the surface normal was
        tried and was 36-45 deg off in two glass flights: the post-impact
        swing dominates it. Every first contact in the flights was head-on,
        where the heading is exact.)"""
        pre, (hx, hy) = r['pre'], r['heading']
        nx, ny = -hx, -hy
        b = math.hypot(self.gz_pos[0] - r['start'][0], self.gz_pos[1] - r['start'][1])
        print(f"  [v3] bounced {b:.2f} m; barrier across the heading, backing off "
              f"{self.RECOVER_BACK_M:.1f} m")
        _ahppa.report_stuck(pre[2], pre[3], -nx, -ny, source='impact')
        r.update(phase='back', until=now + self.RECOVER_MIN_S,
                 target=(pre[2] + nx * self.RECOVER_BACK_M, pre[3] + ny * self.RECOVER_BACK_M, pre[4]))

    def _recovering(self):
        """After an impact: bounce phase, then hold the back-off point; once
        RECOVER_MIN_S has passed and the drone is level, replan (ESCAPE, with
        the new barrier). Returns True while recovery is in progress."""
        r = self._recover
        if r is None:
            return False
        now = self._now()
        if r['phase'] == 'bounce':
            if now < r['until']:
                self._send_position(self.px4_pos[0], self.px4_pos[1], -r['target'][2],
                                    yaw=self.current_yaw)
                return True
            self._bounce_done(r, now)
        tx, ty, tz = r['target']
        # "level" must HOLD: in the first glass flight the tilt passed through
        # 5 deg for one tick mid-oscillation (18 -> 5.3 -> 28 deg) and the
        # replan started while the drone was still swinging
        prev = r.get('prev')
        speed = (math.dist(prev[1:], self.gz_pos[:3]) / max(1e-3, now - prev[0])) if prev else 9.9
        r['prev'] = (now, float(self.gz_pos[0]), float(self.gz_pos[1]), float(self.gz_pos[2]))
        steady = (self.tilt_deg < self.CALM_TILT_DEG and speed < self.RECOVER_MAX_SPEED
                  and abs(self.gz_pos[2] - tz) < 0.3)
        r['steady_since'] = (r.get('steady_since') or now) if steady else None
        held = r['steady_since'] is not None and now - r['steady_since'] >= self.RECOVER_STEADY_S
        if now >= r['until'] and (held or now - r['t0'] >= self.RECOVER_MAX_S):
            self._recover = None
            print(f"  [v3] {'steady again' if held else 'recovery timed out'} after "
                  f"{now - r['t0']:.1f} s — replanning around the remembered barrier")
            self._replan()
            return True
        dx, dy = tx - self.gz_pos[0], ty - self.gz_pos[1]
        d = math.hypot(dx, dy)
        if d > self.MAX_NAV_DELTA:                 # same clamp as the follower
            dx, dy = dx * self.MAX_NAV_DELTA / d, dy * self.MAX_NAV_DELTA / d
        # hold the CURRENT yaw: the default yaw=0 would add a turn to north
        # on top of the upset (glass flight 2, 2026-11-19)
        self._send_position(self.px4_pos[0] + dy, self.px4_pos[1] + dx, -tz, yaw=self.current_yaw)
        return True

    def _stuck_heading(self):
        """Direction the follower is trying to go: toward the first waypoint
        at least 0.3 m ahead (the pure-pursuit target). None at the end."""
        if not self.waypoints or self.gz_pos is None:
            return None
        # start from the CLOSEST waypoint, as _navigate_step does: wp_idx can
        # lag behind the drone (right after a replan it is 0), and a target
        # behind the drone would put the barrier on the wrong side
        hi = min(len(self.waypoints), self.wp_idx + 20)
        k = min(range(self.wp_idx, hi), key=lambda i: self._wp_dist(self.waypoints[i]), default=self.wp_idx)
        for wp in self.waypoints[k:]:
            dx, dy = wp[0] - self.gz_pos[0], wp[1] - self.gz_pos[1]
            if math.hypot(dx, dy) >= 0.3:
                return dx, dy
        return None

    def _fast_stuck(self):
        """AHPPA v3: True (once, then the history restarts) when the drone has
        been COMMANDED along a path for FAST_STUCK_S but moved less than
        FAST_STUCK_M. Unlike the 6 s watchdog this cannot fire while the
        drone hovers to plan (the history restarts with every new path) or
        settles at the goal (not checked within 0.5 m of the last waypoint).
        Pressed against an obstacle the map does not hold (glass, thin posts),
        every planner would keep sending it into the same spot; reporting it
        writes a barrier into the planner's stuck memory."""
        if _ahppa is None or not getattr(_ahppa, 'V3', False) or not self.waypoints:
            return False
        if self._stuck_wps is not self.waypoints:            # new path: restart
            self._stuck_wps, self._stuck_hist = self.waypoints, []
        if self.gz_pos[2] < self.STUCK_MIN_Z:                # on the ground (landed, or
            self._stuck_hist = []                            # knocked down): not a blockage
            return False
        g = self.waypoints[-1]
        if math.hypot(g[0] - self.gz_pos[0], g[1] - self.gz_pos[1]) < 0.5:
            self._stuck_hist = []
            return False
        h = self._stuck_heading()
        if h is None:
            return False
        now = self._now()
        self._stuck_hist.append((now, float(self.gz_pos[0]), float(self.gz_pos[1]), float(self.gz_pos[2])))
        while self._stuck_hist and now - self._stuck_hist[0][0] > self.FAST_STUCK_S:
            self._stuck_hist.pop(0)
        if now - self._stuck_hist[0][0] < self.FAST_STUCK_S - 0.1:
            return False                                     # window not full yet
        a, b = self._stuck_hist[0], self._stuck_hist[-1]
        moved = math.dist(a[1:], b[1:])
        span = max(math.dist(p[1:], a[1:]) for p in self._stuck_hist)
        if span >= self.FAST_STUCK_M:
            return False
        print(f"  🧱 [v3] commanded toward a waypoint {math.hypot(*h):.2f} m ahead but moved "
              f"{moved:.2f} m in {b[0] - a[0]:.1f} s — stuck on something the map does not hold")
        self._stuck_hist = []
        return _ahppa.report_stuck(self.gz_pos[0], self.gz_pos[1], *h, source='fast')

    def _navigate_step(self):
        """Follow path waypoints to the goal with velocity + altitude clamping."""

        # 1. Advance path progress index to the closest waypoint
        min_d = float('inf')
        best_idx = self.wp_idx
        for i in range(self.wp_idx, min(len(self.waypoints), self.wp_idx + 20)):
            wp = self.waypoints[i]
            d = self._wp_dist(wp)
            if d < min_d:
                min_d = d
                best_idx = i
        self.wp_idx = best_idx

        # Check if we reached the final destination
        if self._at_goal():
            final_wp = self.waypoints[-1]
            if len(final_wp) == 3:
                self.hold_alt = float(final_wp[2])
            print(f"  🎉 DESTINATION REACHED at z={self.gz_pos[2]:.2f} m! Set a new goal in RViz.")
            self.waypoints = []
            self.goal_xy = None
            self.goal_layer = None
            self._set_state('READY')
            return

        # Obstacle avoidance: periodically check if path is still clear
        self.avoidance_tick += 1
        if (self.avoidance_tick % self.AVOIDANCE_CHECK_INTERVAL == 0
                and self.goal_xy is not None
                and self.occupancy_grid is not None):
            if self._path_blocked():
                print("  ⚠️  Obstacle detected on path! Replanning...")
                self._replan()
                return

        # 2. Pure pursuit lookahead for smooth steering
        LOOKAHEAD_DIST = 0.3  # Short lookahead prevents corner-cutting (was 0.6)
        target_wp = self.waypoints[self.wp_idx]
        for i in range(self.wp_idx, len(self.waypoints)):
            wp = self.waypoints[i]
            d = self._wp_dist(wp)
            target_wp = wp
            if d >= LOOKAHEAD_DIST:
                break

        # Support both 2D (x,y) and 3D (x,y,z) waypoints
        if len(target_wp) == 3:
            wx, wy, wz = target_wp
        else:
            wx, wy = target_wp
            wz = self.ALTITUDE

        # Relative delta in Gazebo ENU
        dx_enu = wx - self.gz_pos[0]
        dy_enu = wy - self.gz_pos[1]
        dist = math.hypot(dx_enu, dy_enu)

        # Velocity clamping: limit target distance so drone moves smoothly
        if dist > self.MAX_NAV_DELTA:
            scale = self.MAX_NAV_DELTA / dist
            dx_enu *= scale
            dy_enu *= scale

        # Altitude smoothing: interpolate toward target altitude
        alt_diff = wz - self.current_alt
        if abs(alt_diff) > self.MAX_ALT_DELTA:
            alt_diff = math.copysign(self.MAX_ALT_DELTA, alt_diff)
        self.current_alt += self.ALT_SMOOTH * alt_diff

        # Convert ENU delta → NED target
        tgt_px4_x = self.px4_pos[0] + dy_enu
        tgt_px4_y = self.px4_pos[1] + dx_enu
        tgt_px4_z = -self.current_alt   # Smoothed altitude (NED = negative up)

        # Yaw smoothing: interpolate toward target instead of snapping
        target_yaw = math.atan2(wx - self.gz_pos[0], wy - self.gz_pos[1])
        yaw_diff = target_yaw - self.current_yaw
        yaw_diff = (yaw_diff + math.pi) % (2 * math.pi) - math.pi
        self.current_yaw += self.YAW_SMOOTH * yaw_diff
        self._send_position(tgt_px4_x, tgt_px4_y, tgt_px4_z, yaw=self.current_yaw)

        # 3. Print progress if we reached the lookahead waypoint
        if dist < self.WP_REACH_DIST:
            alt_str = f", alt={wz:.1f}m" if len(target_wp) == 3 else ""
            print(f"  📍 WP {self.wp_idx}/{len(self.waypoints)} — {dist:.2f}m{alt_str}")

    PATH_CHECK_HORIZON_M = 5.0    # how far ahead the segment check looks
    PATH_CHECK_STEP_M = 0.10      # sample spacing along each segment

    def _path_blocked(self):
        """Check if the path ahead collides with latest OctoMap (3D-aware)."""
        if self.occupancy_grid is None:
            return False
        if self.octomap_3d.is_ready and self._segments_blocked():
            return True

        # Check next 5 waypoints
        check_end = min(self.wp_idx + 5, len(self.waypoints))

        for i in range(self.wp_idx, check_end):
            wp = self.waypoints[i]
            wx, wy = wp[0], wp[1]
            wz = wp[2] if len(wp) == 3 else self.ALTITUDE

            # 3D check using OctoMap3DQuery
            if self.octomap_3d.is_ready:
                if not self.octomap_3d.is_column_clear(wx, wy,
                        wz - 0.3, wz + 0.3, xy_radius=0.3):
                    return True
            else:
                # Fallback: 2D check on projected map
                info = self.occupancy_grid.info
                raw = np.array(self.occupancy_grid.data).reshape(
                    (info.height, info.width))
                margin_cells = int(math.ceil(self.SAFETY_MARGIN / info.resolution))
                c = int((wx - info.origin.position.x) / info.resolution)
                r = int((wy - info.origin.position.y) / info.resolution)
                for dr in range(-margin_cells, margin_cells + 1):
                    for dc in range(-margin_cells, margin_cells + 1):
                        cr, cc = r + dr, c + dc
                        if (0 <= cr < info.height and 0 <= cc < info.width
                                and raw[cr, cc] > 50):
                            return True
        return False

    def _segments_blocked(self):
        """Sample the STRAIGHT SEGMENTS from the drone through the upcoming
        waypoints, up to PATH_CHECK_HORIZON_M ahead.

        The waypoint check below only looks at the next 5 waypoints. On the
        dense stock A* path that is ~0.5 m ahead; on an any-angle or smoothed
        path (AHPPA v2, Lazy Theta*) waypoints can be metres apart, and a
        wall the map learns about BETWEEN two waypoints was never checked -
        the drone would fly the segment into it. Found by the mission
        simulator (benchmark/mission_sim.py), 2026-11-19."""
        if self.gz_pos is None or not self.waypoints:
            return False
        prev = (float(self.gz_pos[0]), float(self.gz_pos[1]), float(self.gz_pos[2]))
        acc = 0.0
        for wp in self.waypoints[self.wp_idx:]:
            cur = (wp[0], wp[1], wp[2] if len(wp) == 3 else self.ALTITUDE)
            d = math.dist(prev, cur)
            n = max(1, int(math.ceil(d / self.PATH_CHECK_STEP_M)))
            for k in range(1, n + 1):
                f = k / n
                x = prev[0] + (cur[0] - prev[0]) * f
                y = prev[1] + (cur[1] - prev[1]) * f
                z = prev[2] + (cur[2] - prev[2]) * f
                if not self.octomap_3d.is_column_clear(x, y, z - 0.3, z + 0.3, xy_radius=0.3):
                    return True
                if acc + d * f > self.PATH_CHECK_HORIZON_M:
                    return False
            acc += d
            prev = cur
        return False

    def _replan(self):
        """Replan from current position to stored goal (3D-aware)."""
        if self.goal_xy is None or self.occupancy_grid is None:
            return

        # Hover while replanning. Keep the current yaw: the default yaw=0 made
        # PX4 turn the drone toward north for the whole planning time
        # (found in the glass flights, 2026-11-19).
        self._send_position(self.px4_pos[0], self.px4_pos[1], -self.current_alt,
                            yaw=self.current_yaw)

        path = None
        # Try 2.5D replan first
        if self.octomap_3d.is_ready:
            try:
                planner = MultiLayerPlanner(self.occupancy_grid, self.octomap_3d,
                                            safety_margin=self.SAFETY_MARGIN, unknown_penalty=50.0)
                # Use standard A* (dense path) instead of Theta* (sparse path)
                path = self._plan_layers(planner, self.gz_pos[0], self.gz_pos[1],
                                         layer_for_z(self.gz_pos[2]),
                                         self.goal_xy[0], self.goal_xy[1], self.goal_layer)
                self._viz_kind = getattr(planner, 'last_memory', None)
            except Exception:
                path = None

        # Fallback to 2D - ground floor only
        if path is None and not self._fallback_ok(self.goal_layer):
            print("  ❌ Replan: no 2.5D path; 2D fallback refused above the ground floor.")
        elif path is None:
            planner_2d = SensorMapPlanner(self.occupancy_grid, safety_margin=self.SAFETY_MARGIN, unknown_penalty=50.0)
            path_2d = planner_2d.plan(self.gz_pos[0], self.gz_pos[1],
                                      self.goal_xy[0], self.goal_xy[1])
            if path_2d:
                path = [(x, y, self.ALTITUDE) for x, y in path_2d]

        if path:
            self.waypoints = densify_path(path)
            self.wp_idx = 0
            self._publish_rviz_path(path)
            print(f"  ✅ Replanned — {len(path)} waypoints. Resuming!")
        else:
            print("  ❌ Replan failed — hovering. Try a new goal.")
            self.waypoints = []
            self.goal_xy = None
            self.goal_layer = None
            self._set_state('READY')

    # ── Takeoff ───────────────────────────────────────────────────────────────

    def _takeoff(self):
        print("  🚀 Takeoff sequence starting...")

        hover_z = -self.ALTITUDE
        print("  ⏳ Sending warmup heartbeats...")
        for _ in range(30):
            self._heartbeat()
            self._send_position(self.px4_pos[0], self.px4_pos[1], hover_z)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)

        print("  ⚙️  OFFBOARD mode...")
        self._send_command(176, param1=1.0, param2=6.0)
        time.sleep(0.5)

        for _ in range(10):
            self._heartbeat()
            self._send_position(self.px4_pos[0], self.px4_pos[1], hover_z)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.1)

        print("  🔋 Arming motors...")
        self._send_command(400, param1=1.0)
        time.sleep(0.5)

        self.frontier_phase = 'INITIAL_SCAN'
        self.scan_rotating = True
        self.scan_yaw = 0.0
        print(f"\n  ✅ Airborne at {self.ALTITUDE}m!")
        if self.EXPLORE_ON_START:
            self._set_state('EXPLORING')
            print(f"  🔄 Starting FRONTIER-BASED exploration...")
            print(f"     Initial 360° scan at spawn to seed OctoMap...\n")
        else:
            self._set_state('READY')
            print(f"  ⏸  Holding position — frontier exploration disabled.")
            print(f"     Use RViz 'Publish Point' to send a 3D goal.")
            print(f"     Enable exploring with: --ros-args -p explore:=true\n")

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _heartbeat(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_pub.publish(msg)

    def _send_position(self, x, y, z, yaw=0.0):
        msg = TrajectorySetpoint()
        msg.position = [float(x), float(y), float(z)]
        msg.yaw = float(yaw)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.setpoint_pub.publish(msg)

    def _send_command(self, cmd_id, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command = cmd_id
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.command_pub.publish(msg)

    # Unseen obstacles per world, for the demo overlay only: (cx, cy, cz, sx, sy, sz, label)
    VIZ_GHOSTS = {'glass_test': [(4.0, 0.85, 1.4, 0.05, 7.3, 2.8,
                                  'glass pane: invisible to the depth cameras')]}
    VIZ_KIND_RGB = {'bar': (1.0, 0.35, 0.1), 'hull': (1.0, 0.8, 0.1), 'dot': (0.9, 0.2, 0.9)}

    def _viz_stock_path(self, planner, sx, sy, sl, gx, gy, gl):
        """Demo overlay only: plan the same request with the STOCK layered A*
        (the planner AHPPA replaces) and publish it, so RViz can show both."""
        import contextlib, io
        try:
            with contextlib.redirect_stdout(io.StringIO()):
                sp = MultiLayerPlanner.plan(planner, sx, sy, gx, gy, start_layer=sl, goal_layer=gl)
        except Exception:
            sp = None
        self._viz_stock = [tuple(p) for p in sp] if sp else None
        if not hasattr(self, 'stock_pub'):
            self.stock_pub = self.create_publisher(Path, '/ahppa/stock_path', 10)
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        for wp in (self._viz_stock or []):
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x, ps.pose.position.y = float(wp[0]), float(wp[1])
            ps.pose.position.z = float(wp[2]) if len(wp) == 3 else self.ALTITUDE
            msg.poses.append(ps)
        self.stock_pub.publish(msg)

    def _viz_tick(self):
        if self.gz_pos is None:
            return
        now = self.get_clock().now().to_msg()
        ms = []

        def mk(ns, i, typ, rgba, scale):
            m = Marker()
            m.header.frame_id, m.header.stamp = 'map', now
            m.ns, m.id, m.type, m.action = ns, i, typ, Marker.ADD
            m.pose.orientation.w = 1.0
            m.color.r, m.color.g, m.color.b, m.color.a = (float(v) for v in rgba)
            m.scale.x, m.scale.y, m.scale.z = (float(v) for v in scale)
            ms.append(m)
            return m

        def pt(x, y, z):
            q = PointStamped().point
            q.x, q.y, q.z = float(x), float(y), float(z)
            return q

        x, y, z = (float(v) for v in self.gz_pos[:3])
        d = mk('drone', 0, Marker.SPHERE, (0.1, 0.9, 0.3, 1.0), (0.45, 0.45, 0.2))
        d.pose.position = pt(x, y, z)
        if not self._viz_trail or math.dist(self._viz_trail[-1], (x, y, z)) > 0.05:
            self._viz_trail.append((x, y, z))
        if len(self._viz_trail) > 1:
            t = mk('trail', 0, Marker.LINE_STRIP, (0.1, 0.9, 0.3, 0.8), (0.04, 0, 0))
            t.points = [pt(*p) for p in self._viz_trail[-3000:]]

        for i, (cx, cy, cz, sx, sy, sz, lab) in enumerate(self.VIZ_GHOSTS.get(self._viz_world, [])):
            g = mk('unseen obstacle', i, Marker.CUBE, (0.35, 0.75, 1.0, 0.25), (sx, sy, sz))
            g.pose.position = pt(cx, cy, cz)
            tx = mk('unseen obstacle label', i, Marker.TEXT_VIEW_FACING, (0.6, 0.85, 1.0, 1.0), (0, 0, 0.3))
            tx.pose.position, tx.text = pt(cx, cy + sy / 2 - 0.8, cz + sz / 2 + 0.35), lab

        mem = list(getattr(_ahppa, 'MEMORY', [])) if _ahppa is not None else []
        kind = self._viz_kind if self._viz_kind in self.VIZ_KIND_RGB else 'bar'
        if mem:
            r, g_, b = self.VIZ_KIND_RGB[kind]
            for i, (x0, y0, x1, y1) in enumerate(_ahppa.memory_segments(mem, kind)):
                L = math.hypot(x1 - x0, y1 - y0)
                w = mk('contact memory', i, Marker.CUBE, (r, g_, b, 0.55), (max(L, 0.1), 0.12, 2.4))
                w.pose.position = pt((x0 + x1) / 2, (y0 + y1) / 2, 1.3)
                a = math.atan2(y1 - y0, x1 - x0)
                w.pose.orientation.z, w.pose.orientation.w = math.sin(a / 2), math.cos(a / 2)
            c = mk('contacts', 0, Marker.SPHERE_LIST, (1.0, 0.15, 0.15, 1.0), (0.22, 0.22, 0.22))
            bd = getattr(_ahppa, 'BAR_D', 0.3)
            c.points = [pt(m_['cx'] - m_['hx'] * bd, m_['cy'] - m_['hy'] * bd, 1.8) for m_ in mem]
        # clear barrier slots no longer used (fusion merges barriers)
        for i in range(len([m_ for m_ in ms if m_.ns == 'contact memory']), 30):
            m = Marker()
            m.header.frame_id, m.header.stamp, m.ns, m.id, m.action = 'map', now, 'contact memory', i, Marker.DELETE
            ms.append(m)

        def plen(P_):
            return sum(math.dist(P_[i][:2], P_[i + 1][:2]) for i in range(len(P_) - 1)) if P_ and len(P_) > 1 else 0.0
        sp, ap = getattr(self, '_viz_stock', None), getattr(self, '_viz_ahppa', None)
        if sp and ap:
            cm = mk('comparison', 0, Marker.TEXT_VIEW_FACING, (1.0, 1.0, 1.0, 1.0), (0, 0, 0.32))
            cm.pose.position = pt(x, y, z + 1.4)
            cm.text = (f"stock A*: {plen(sp):.1f} m, {len(sp)} waypoints   |   "
                       f"AHPPA: {plen(ap):.1f} m, {len(ap)} waypoints")
        v = ('v2 (no contact memory)' if not getattr(_ahppa, 'V3', False)
             else 'v3.1 (fusion)' if getattr(_ahppa, 'FUSE', False) else 'v3')
        st = mk('status', 0, Marker.TEXT_VIEW_FACING, (1.0, 1.0, 1.0, 1.0), (0, 0, 0.35))
        st.pose.position = pt(x, y, z + 0.8)
        st.text = (f"AHPPA {v} | contacts remembered: {len(mem)}"
                   + (f" | memory level: {kind}" if mem else ""))
        arr = MarkerArray()
        arr.markers = ms
        self.viz_pub.publish(arr)

    def _publish_rviz_path(self, coords):
        """Draw the planned path as a line in RViz (supports 2D and 3D coords)."""
        self._viz_ahppa = [tuple(c) for c in coords]
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        for wp in coords:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = wp[0]
            ps.pose.position.y = wp[1]
            ps.pose.position.z = wp[2] if len(wp) == 3 else self.ALTITUDE
            msg.poses.append(ps)
        self.path_pub.publish(msg)


# ─────────────────────────────────────────────────────────────────────────────
#  MAIN
# ─────────────────────────────────────────────────────────────────────────────
def main():
    rclpy.init()
    node = AutonomousNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n  🛬 Landing...")
        node._send_command(21)  # MAV_CMD_NAV_LAND
        time.sleep(2)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("  ✅ Navigator stopped.")


if __name__ == '__main__':
    main()
