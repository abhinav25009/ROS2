#!/usr/bin/env python3
"""Autonomous multi-floor exploration by generating 3D goals.

Rather than replacing the navigator, this drives it. The navigator already
accepts a 3D clicked goal and plans across floors with the layered planner,
so this node just decides WHERE to go next and publishes it on /clicked_point.

    explorer_3d  --(/clicked_point)-->  navigator_astar  --> flight
         ^                                     |
         +--------- /nav_state ----------------+

Start the navigator with explore:=false (the default) so it takes off and
holds READY; this node then feeds it frontiers until the house is mapped.

    python3 explorer_3d.py --ros-args -p use_sim_time:=true
"""
import math
import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker, MarkerArray

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from frontier3d import VisibilityGrid, LayeredFrontierFinder   # noqa: E402

sys.path.insert(0, os.path.expanduser("~/Desktop/Drone_IP"))
from core.octomap_3d_query import OctoMap3DQuery                # noqa: E402
from planners.planner_3d import Z_LAYERS                        # noqa: E402


class Explorer3D(Node):
    def __init__(self):
        super().__init__('explorer_3d')
        # Exploration volume. Outdoors there are no obstacles, so every
        # direction is a frontier and the drone wanders off for ever.
        for _k, _v in (('xmin', -7.5), ('xmax', 7.5),
                       ('ymin', -5.5), ('ymax', 5.5),
                       ('zmin', 0.3), ('zmax', 6.5)):
            self.declare_parameter(_k, _v)
        # Re-entry route. The nearest in-bounds point is NOT reachable from
        # outside: in house_2floor the only opening in the east wall is at
        # (7.1, 0), and a drone that drifted out at y = -4 flies into solid
        # wall trying to go straight back. Approach the door, then step in.
        for _k, _v in (('use_door', True),
                       ('door_out_x', 8.2), ('door_out_y', 0.0),
                       ('door_in_x', 5.6), ('door_in_y', 0.0),
                       ('door_z', 1.8)):
            self.declare_parameter(_k, _v)
        p = self.declare_parameter
        p('world', 'house_2floor')
        p('sensor_range', 4.0)
        p('min_cluster', 4)
        p('reach_tol', 1.0)        # within this of the goal = reached
        p('settle_s', 3.0)         # wait after READY before the next goal
        p('max_fail', 3)           # consecutive empty scans before done

        world = self.get_parameter('world').value
        self.reach_tol = float(self.get_parameter('reach_tol').value)
        self.settle_s = float(self.get_parameter('settle_s').value)
        self.max_fail = int(self.get_parameter('max_fail').value)

        self.octo = OctoMap3DQuery(resolution=0.10)
        gp = lambda k: float(self.get_parameter(k).value)
        bounds = (gp('xmin'), gp('xmax'), gp('ymin'), gp('ymax'),
                  gp('zmin'), gp('zmax'))
        self.bounds = bounds
        self.out_ticks = 0
        print(f"     exploring x{bounds[0]:.1f}..{bounds[1]:.1f} "
              f"y{bounds[2]:.1f}..{bounds[3]:.1f} z{bounds[4]:.1f}..{bounds[5]:.1f}")
        self.vis = VisibilityGrid(
            bounds=bounds, res=0.25,
            sensor_range=float(self.get_parameter('sensor_range').value))
        self.ff = LayeredFrontierFinder(
            self.vis, Z_LAYERS,
            min_cluster=int(self.get_parameter('min_cluster').value))

        self.pos = None
        self.nav_state = None
        self.goal = None
        self.target = None
        self.blacklist = []
        self.sent = 0
        self.fails = 0
        self.done = False
        self._ready_since = None

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        latched = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)

        self.create_subscription(PointCloud2, '/octomap_point_cloud_centers',
                                 self.octo.update_from_pointcloud2, qos)
        self.create_subscription(TFMessage, f'/world/{world}/dynamic_pose/info',
                                 self.pose_cb, qos)
        self.create_subscription(String, '/nav_state', self.state_cb, latched)

        self.goal_pub = self.create_publisher(PointStamped, '/clicked_point', 10)
        self.mk_pub = self.create_publisher(MarkerArray, '/frontiers_3d', 1)
        self.create_timer(1.0, self.tick)

        print("\n  3D frontier explorer")
        print(f"     layers: {Z_LAYERS}")
        print(f"     sensor range {self.vis.range} m, cluster >= "
              f"{self.ff.min_cluster} cells")
        print("     start navigator_astar.py WITHOUT explore:=true\n")

    # ── inputs ──────────────────────────────────────────────────────────────
    def pose_cb(self, msg):
        for tr in msg.transforms:
            if tr.child_frame_id.startswith('x500'):
                t = tr.transform.translation
                self.pos = (t.x, t.y, t.z)
                if t.z > 0.5:                  # only mark once airborne
                    self.vis.mark(t.x, t.y, t.z)
                return

    def state_cb(self, msg):
        if msg.data != self.nav_state:
            self.nav_state = msg.data
            self._ready_since = (self.get_clock().now().nanoseconds / 1e9
                                 if msg.data == 'READY' else None)

    # ── main loop ───────────────────────────────────────────────────────────
    def tick(self):
        if self.done or self.pos is None or not self.octo.is_ready:
            return
        if self.nav_state != 'READY' or self._ready_since is None:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self._ready_since < self.settle_s:
            return

        # did the previous goal actually get reached?
        if self.goal is not None:
            d = math.dist(self.pos, self.goal)
            if d > self.reach_tol:
                # blacklist the frontier, not the stepping stone
                self.blacklist.append(self.target or self.goal)
                print(f"  x unreached ({d:.1f} m away) - frontier blacklisted "
                      f"({len(self.blacklist)} total)")
            self.goal = None
            self.target = None

        # GEOFENCE. Bounding the frontier grid limits where goals may be;
        # it does nothing to stop the aircraft leaving. Once outside, the
        # start cell clamps to the boundary and every goal we publish has no
        # safe route home - which is how it ended up against the outside of
        # the east wall. Fly it back in before resuming exploration.
        x0, x1, y0, y1, z0, z1 = self.bounds
        px, py, pz = self.pos
        if not (x0 <= px <= x1 and y0 <= py <= y1 and z0 <= pz <= z1):
            self.out_ticks += 1
            gp2 = lambda k: float(self.get_parameter(k).value)
            if bool(self.get_parameter('use_door').value):
                dz = gp2('door_z')
                out = (gp2('door_out_x'), gp2('door_out_y'), dz)
                ins = (gp2('door_in_x'), gp2('door_in_y'), dz)
                # line up outside the opening first, then step through it
                if math.dist(self.pos, out) > 1.5:
                    rx, ry, rz = out
                else:
                    rx, ry, rz = ins
            else:
                rx = min(max(px, x0 + 1.0), x1 - 1.0)
                ry = min(max(py, y0 + 1.0), y1 - 1.0)
                rz = min(max(pz, 1.2), 2.4)
            print(f"  !! OUTSIDE the exploration box at "
                  f"({px:.2f},{py:.2f},{pz:.2f}) - returning to "
                  f"({rx:.2f},{ry:.2f},{rz:.2f})  [{self.out_ticks}]")
            if self.out_ticks > 20:
                print("  !! cannot get back inside - stopping")
                self.finish()
                return
            m = PointStamped()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.point.x, m.point.y, m.point.z = float(rx), float(ry), float(rz)
            self.goal_pub.publish(m)
            self.goal = (rx, ry, rz)
            self.target = None
            self._ready_since = None
            return
        self.out_ticks = 0

        fr = self.ff.find(self.octo.voxel_centers, self.pos, self.blacklist)
        self.publish_markers(fr)

        if not fr:
            self.fails += 1
            print(f"  no frontiers ({self.fails}/{self.max_fail})")
            if self.fails >= self.max_fail:
                self.finish()
            return

        self.fails = 0
        best = fr[0]
        self.goal = (best['x'], best['y'], best['z'])
        self.target = (best['tx'], best['ty'], best['tz'])
        self.sent += 1
        print(f"  [{self.sent:3d}] hop ({best['x']:6.2f},{best['y']:6.2f},"
              f"{best['z']:4.1f}) -> frontier ({best['tx']:6.2f},"
              f"{best['ty']:6.2f},{best['tz']:4.1f})  L{best['layer']}"
              f"  size {best['size']:3d}  path {best['dist']:5.2f} m | "
              f"{len(fr)} reachable, {self.ff.last_blocked} walled off, "
              f"{self.vis.observed_fraction*100:.1f}% observed")

        m = PointStamped()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.point.x, m.point.y, m.point.z = self.goal
        self.goal_pub.publish(m)
        self._ready_since = None

    def finish(self):
        self.done = True
        print("\n  ============================================")
        print("  3D EXPLORATION COMPLETE")
        print(f"     goals sent      : {self.sent}")
        print(f"     blacklisted     : {len(self.blacklist)}")
        print(f"     volume observed : {self.vis.observed_fraction*100:.1f}%")
        print(f"     octomap voxels  : {self.octo.num_voxels}")
        print("  ============================================\n")

    # ── rviz ────────────────────────────────────────────────────────────────
    def publish_markers(self, fr):
        arr = MarkerArray()
        d = Marker(); d.action = Marker.DELETEALL
        arr.markers.append(d)
        for i, f in enumerate(fr[:30]):
            mk = Marker()
            mk.header.frame_id = 'map'
            mk.header.stamp = self.get_clock().now().to_msg()
            mk.ns = 'frontiers'; mk.id = i
            mk.type = Marker.SPHERE; mk.action = Marker.ADD
            mk.pose.position.x = float(f['tx'])
            mk.pose.position.y = float(f['ty'])
            mk.pose.position.z = float(f['tz'])
            mk.pose.orientation.w = 1.0
            s = 0.25 + 0.02 * min(f['size'], 30)
            mk.scale.x = mk.scale.y = mk.scale.z = s
            frac = f['layer'] / max(len(Z_LAYERS) - 1, 1)
            mk.color.r = float(1.0 - frac)
            mk.color.g = 0.35
            mk.color.b = float(frac)
            mk.color.a = 0.95 if i == 0 else 0.5
            arr.markers.append(mk)
        self.mk_pub.publish(arr)


def main():
    rclpy.init()
    n = Explorer3D()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        n.finish()
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
