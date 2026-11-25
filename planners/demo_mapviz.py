#!/usr/bin/env python3
"""Display-only: republish the OctoMap markers without the ceiling and floor so
RViz can show the inside of a closed room. Nothing here reaches the planner.
  /occupied_cells_vis_array -> /demo/occupied_cells  (keeps zmin <= z <= zmax)"""
import sys
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray

ZMIN, ZMAX = 0.2, float(sys.argv[1]) if len(sys.argv) > 1 and not sys.argv[1].startswith('-') else 2.6


class F(Node):
    def __init__(self):
        super().__init__('demo_mapviz')
        self.pub = self.create_publisher(MarkerArray, '/demo/occupied_cells', 5)
        self.create_subscription(MarkerArray, '/occupied_cells_vis_array', self.cb, 5)

    def cb(self, msg):
        for m in msg.markers:
            keep = [i for i, p in enumerate(m.points) if ZMIN <= p.z <= ZMAX]
            m.points = [m.points[i] for i in keep]
            if len(m.colors) == len(keep) or m.colors:
                m.colors = [m.colors[i] for i in keep] if m.colors else []
        self.pub.publish(msg)


rclpy.init()
rclpy.spin(F())
