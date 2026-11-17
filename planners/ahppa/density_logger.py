#!/usr/bin/env python3
"""Log obstacle density along a flight, so AHPPA's thresholds come from
measurement rather than inherited constants.

Fly the drone around the house while this runs, then Ctrl+C for the histogram.

  python3 density_logger.py --ros-args -p use_sim_time:=true
  python3 density_logger.py --ros-args -p out:=/tmp/density_2floor.csv
"""
import csv, os, sys
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from density import DensityEstimator                      # noqa: E402

sys.path.insert(0, os.path.expanduser("~/Desktop/Drone_IP"))
from core.octomap_3d_query import OctoMap3DQuery           # noqa: E402


class DensityLogger(Node):
    def __init__(self):
        super().__init__('density_logger')
        self.declare_parameter('out', '/tmp/density_log.csv')
        self.declare_parameter('window_xy', 15.0)
        self.declare_parameter('band_z', 1.0)
        self.declare_parameter('world', 'house_2floor')

        self.path = self.get_parameter('out').value
        self.est = DensityEstimator(
            window_xy=float(self.get_parameter('window_xy').value),
            band_z=float(self.get_parameter('band_z').value))
        world = self.get_parameter('world').value

        self.octo = OctoMap3DQuery(resolution=0.10)
        self.pos = None
        self.rows = []

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(PointCloud2, '/octomap_point_cloud_centers',
                                 self.octo.update_from_pointcloud2, qos)
        self.create_subscription(TFMessage, f'/world/{world}/dynamic_pose/info',
                                 self.pose_cb, qos)
        self.create_timer(0.2, self.sample)                # 5 Hz

        print(f"\n  Density logger -> {self.path}")
        print(f"     window {self.est.window_xy} m, band +/-{self.est.band_z} m")
        print(f"     fly around, then Ctrl+C for the histogram\n")

    def pose_cb(self, msg):
        for tr in msg.transforms:
            if tr.child_frame_id.startswith('x500'):
                t = tr.transform.translation
                self.pos = (t.x, t.y, t.z)
                return

    def sample(self):
        if self.pos is None or not self.octo.is_ready:
            return
        x, y, z = self.pos
        d, det = self.est.compute(self.octo.voxel_centers, x, y, z)
        t = self.get_clock().now().nanoseconds / 1e9
        self.rows.append((round(t, 2), round(x, 2), round(y, 2), round(z, 2),
                          round(d, 4), det['occupied_cells'], det['total_cells'],
                          self.octo.num_voxels))
        if len(self.rows) % 25 == 0:
            print(f"  n={len(self.rows):5d}  pos=({x:6.2f},{y:6.2f},{z:5.2f})  "
                  f"density={d*100:5.1f}%  voxels={self.octo.num_voxels}")

    def report(self):
        if not self.rows:
            print("\n  no samples collected")
            return
        with open(self.path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', 'x', 'y', 'z', 'density',
                        'occupied_cells', 'total_cells', 'voxels'])
            w.writerows(self.rows)

        d = np.array([r[4] for r in self.rows]) * 100.0
        print(f"\n  {len(d)} samples -> {self.path}")
        print(f"  min {d.min():.1f}%  p25 {np.percentile(d,25):.1f}%  "
              f"median {np.median(d):.1f}%  p75 {np.percentile(d,75):.1f}%  "
              f"max {d.max():.1f}%")
        print("\n  histogram")
        hist, edges = np.histogram(d, bins=20)
        top = max(hist.max(), 1)
        for h, lo, hi in zip(hist, edges[:-1], edges[1:]):
            print(f"    {lo:5.1f}-{hi:5.1f}% | {'#' * int(40*h/top):<40} {h}")
        print("\n  Suggested thresholds (terciles of the observed distribution):")
        t1, t2 = np.percentile(d, [33, 66])
        print(f"    open_up   = {t1/100:.3f}   open_down = {(t1-2)/100:.3f}")
        print(f"    dense_up  = {t2/100:.3f}   dense_down= {(t2-2)/100:.3f}")


def main():
    rclpy.init()
    n = DensityLogger()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.report()
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
