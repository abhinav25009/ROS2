#!/usr/bin/env python3
"""Log the drone's Gazebo pose and tilt at 10 Hz (sim time) to a CSV.

  python3 benchmark/flight_pose_logger.py OUT.csv --ros-args -p use_sim_time:=true -p world:=glass_test
Columns: t, x, y, z, tilt_deg (angle between body z and world z).
"""
import math
import sys

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


class PoseLog(Node):
    def __init__(self, out):
        super().__init__('flight_pose_logger')
        self.declare_parameter('world', 'house_2floor')
        world = str(self.get_parameter('world').value)
        self.f = open(out, 'w', buffering=1)
        self.f.write('t,x,y,z,tilt_deg\n')
        self.last = -1.0
        self.create_subscription(TFMessage, f'/world/{world}/dynamic_pose/info', self.cb, 10)

    def cb(self, msg):
        t = self.get_clock().now().nanoseconds / 1e9
        if t - self.last < 0.1:
            return
        for tr in msg.transforms:
            if tr.child_frame_id.startswith('x500'):
                p, q = tr.transform.translation, tr.transform.rotation
                # body z axis in world: third column of R(q); tilt = acos(R22)
                r22 = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
                tilt = math.degrees(math.acos(max(-1.0, min(1.0, r22))))
                self.f.write(f'{t:.2f},{p.x:.3f},{p.y:.3f},{p.z:.3f},{tilt:.1f}\n')
                self.last = t
                return


def main():
    out = sys.argv[1]
    rclpy.init(args=sys.argv[2:])
    rclpy.spin(PoseLog(out))


if __name__ == '__main__':
    main()
