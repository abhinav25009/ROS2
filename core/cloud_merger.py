#!/usr/bin/env python3
"""Merge N depth clouds into one, for OctoMap.

octomap_server accepts a single cloud_in, so fusing is mandatory. Keeping it
in one configurable node means swapping the 5-camera rig for a different
sensor later is a parameter change, not a rewrite.

  python3 cloud_merger.py --ros-args -p use_sim_time:=true
  python3 cloud_merger.py --ros-args -p input_topics:="['/lidar/points']"

Latest-message-wins: a slow or dead input never blocks the others.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
import tf2_ros

DEFAULT_INPUTS = ['/depth_front/points', '/depth_right/points',
                  '/depth_rear/points', '/depth_left/points',
                  '/depth_up/points']


def quat_to_mat(x, y, z, w):
    n = math_norm = (x * x + y * y + z * z + w * w) ** 0.5
    if n == 0:
        return np.eye(3, dtype=np.float32)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ], dtype=np.float32)


def xyz_from_msg(msg):
    try:
        a = point_cloud2.read_points_numpy(
            msg, field_names=('x', 'y', 'z'), skip_nans=True)
        return np.asarray(a, dtype=np.float32).reshape(-1, 3)
    except AttributeError:
        rec = point_cloud2.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True)
        rec = np.asarray(list(rec))
        if rec.size == 0:
            return np.empty((0, 3), np.float32)
        if rec.dtype.names:
            return np.stack([rec['x'], rec['y'], rec['z']], -1).astype(np.float32)
        return rec.astype(np.float32).reshape(-1, 3)


class CloudMerger(Node):
    def __init__(self):
        super().__init__('cloud_merger')
        self.declare_parameter('input_topics', DEFAULT_INPUTS)
        self.declare_parameter('output_topic', '/fused_points')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('voxel_size', 0.05)
        self.declare_parameter('publish_rate', 5.0)
        self.declare_parameter('max_age_sec', 2.0)

        self.inputs = list(self.get_parameter('input_topics').value)
        self.target = self.get_parameter('target_frame').value
        self.voxel = float(self.get_parameter('voxel_size').value)
        self.max_age = float(self.get_parameter('max_age_sec').value)
        out = self.get_parameter('output_topic').value
        rate = float(self.get_parameter('publish_rate').value)

        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self)

        self.latest = {}
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        for t in self.inputs:
            self.create_subscription(
                PointCloud2, t, lambda m, tp=t: self.latest.__setitem__(tp, m), qos)

        self.pub = self.create_publisher(PointCloud2, out, 5)
        self.create_timer(1.0 / rate, self.merge)
        self.n = 0

        print(f'\n  Cloud merger  ->  {out}  (frame: {self.target})')
        for t in self.inputs:
            print(f'     in: {t}')
        print(f'     voxel {self.voxel} m, {rate} Hz\n')

    def merge(self):
        now = self.get_clock().now()
        chunks, newest, used = [], None, 0

        for topic, msg in list(self.latest.items()):
            age = (now - rclpy.time.Time.from_msg(msg.header.stamp)).nanoseconds / 1e9
            if age > self.max_age:
                continue
            pts = xyz_from_msg(msg)
            pts = pts[np.isfinite(pts).all(1)]   # Inf survives skip_nans
            if pts.shape[0] == 0:
                continue
            src = msg.header.frame_id
            if src != self.target:
                try:
                    tf = self.buf.lookup_transform(
                        self.target, src, rclpy.time.Time()).transform
                except Exception:
                    continue
                R = quat_to_mat(tf.rotation.x, tf.rotation.y,
                                tf.rotation.z, tf.rotation.w)
                t = np.array([tf.translation.x, tf.translation.y,
                              tf.translation.z], np.float32)
                pts = pts @ R.T + t
            chunks.append(pts)
            used += 1
            if newest is None or \
               rclpy.time.Time.from_msg(msg.header.stamp) > newest:
                newest = rclpy.time.Time.from_msg(msg.header.stamp)

        if not chunks:
            return

        pts = np.concatenate(chunks, 0)
        pts = pts[np.isfinite(pts).all(1)]
        raw = pts.shape[0]

        if self.voxel > 0 and raw:
            keys = np.floor(pts / self.voxel).astype(np.int64)
            _, idx = np.unique(keys, axis=0, return_index=True)
            pts = pts[np.sort(idx)]

        self.publish(pts, newest)
        self.n += 1
        if self.n % 25 == 0:
            print(f'  {used}/{len(self.inputs)} clouds | '
                  f'{raw} -> {pts.shape[0]} pts after voxel')

    def publish(self, pts, stamp):
        m = PointCloud2()
        m.header.frame_id = self.target
        m.header.stamp = (stamp or self.get_clock().now()).to_msg()
        m.height, m.width = 1, pts.shape[0]
        m.fields = [PointField(name=n, offset=4 * i,
                               datatype=PointField.FLOAT32, count=1)
                    for i, n in enumerate(('x', 'y', 'z'))]
        m.is_bigendian = False
        m.point_step = 12
        m.row_step = 12 * pts.shape[0]
        m.is_dense = True
        m.data = np.ascontiguousarray(pts, np.float32).tobytes()
        self.pub.publish(m)


def main():
    rclpy.init()
    node = CloudMerger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
