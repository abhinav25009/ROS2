#!/usr/bin/env python3
"""TF broadcaster for the 5-camera 360 rig (x500_depth_360).

  map ─(dynamic, Gazebo ground truth)→ base_link
       ─(static)→ camera_360_link
            ├─→ cam_front_link   (identity)
            ├─→ cam_right_link   (yaw -90)
            ├─→ cam_rear_link    (yaw 180)
            ├─→ cam_left_link    (yaw +90)
            └─→ cam_up_link      (pitch -90)

Rotations mirror each sensor's <pose> in OakD-360/model.sdf. Gazebo publishes
depth clouds in the sensor's own frame, so these must match exactly.

  python3 tf_broadcaster_360.py --ros-args -p use_sim_time:=true
"""
import math
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

CAMERA_XYZ = (0.12, 0.03, 0.242)          # camera_360_link on base_link

# child frame -> (roll, pitch, yaw) matching the SDF sensor poses
CAMS = {
    'cam_front_link': (0.0,  0.0,        0.0),
    'cam_right_link': (0.0,  0.0,       -math.pi / 2),
    'cam_rear_link':  (0.0,  0.0,        math.pi),
    'cam_left_link':  (0.0,  0.0,        math.pi / 2),
    'cam_up_link':    (0.0, -math.pi / 2, 0.0),
}


def rpy_to_quat(r, p, y):
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


class Tf360(Node):
    def __init__(self):
        super().__init__('drone_tf_broadcaster_360')
        self.dyn = TransformBroadcaster(self)
        self.static = StaticTransformBroadcaster(self)
        self.pose = None

        self.declare_parameter('world', 'house_2floor')
        world = str(self.get_parameter('world').value)
        self.create_subscription(TFMessage,
                                 f'/world/{world}/dynamic_pose/info',
                                 self.on_pose, 10)
        self.create_timer(0.05, self.publish_dynamic)   # 20 Hz

        self.publish_static()
        print('\n  TF 360 broadcaster started')
        print('     map -> base_link -> camera_360_link -> 5 camera frames')
        for name in CAMS:
            print(f'       - {name}')

    def publish_static(self):
        out, now = [], self.get_clock().now().to_msg()

        t = TransformStamped()
        t.header.stamp, t.header.frame_id = now, 'base_link'
        t.child_frame_id = 'camera_360_link'
        t.transform.translation.x, t.transform.translation.y, \
            t.transform.translation.z = CAMERA_XYZ
        t.transform.rotation.w = 1.0
        out.append(t)

        for child, rpy in CAMS.items():
            qx, qy, qz, qw = rpy_to_quat(*rpy)
            c = TransformStamped()
            c.header.stamp, c.header.frame_id = now, 'camera_360_link'
            c.child_frame_id = child
            c.transform.rotation.x = qx
            c.transform.rotation.y = qy
            c.transform.rotation.z = qz
            c.transform.rotation.w = qw
            out.append(c)

        self.static.sendTransform(out)

    def on_pose(self, msg):
        # match the drone by name rather than assuming index 0
        for tr in msg.transforms:
            if tr.child_frame_id.startswith('x500'):
                self.pose = tr.transform
                return
        if msg.transforms:
            self.pose = msg.transforms[0].transform

    def publish_dynamic(self):
        if self.pose is None:
            return
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = 'map', 'base_link'
        t.transform = self.pose
        self.dyn.sendTransform(t)


def main():
    rclpy.init()
    node = Tf360()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
