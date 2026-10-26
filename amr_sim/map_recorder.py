#!/usr/bin/env python3
"""Saves every /map update (with sim time) and ground truth / odom poses to an npz for analysis."""
import sys
import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy


class Rec(Node):
    def __init__(self, out):
        super().__init__("map_recorder")
        self.out = out
        self.maps, self.gt, self.od = [], [], []
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, "map", self.on_map, qos)
        self.create_subscription(Odometry, "ground_truth", lambda m: self.pose(m, self.gt), 20)
        self.create_subscription(Odometry, "odom", lambda m: self.pose(m, self.od), 20)

    def pose(self, m, buf):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        o = m.pose.pose.orientation
        buf.append((t, m.pose.pose.position.x, m.pose.pose.position.y, np.arctan2(2 * o.w * o.z, 1 - 2 * o.z * o.z)))

    def on_map(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        d = np.array(m.data, dtype=np.int8).reshape(m.info.height, m.info.width)
        self.maps.append((t, m.info.resolution, m.info.origin.position.x, m.info.origin.position.y, d))
        self.get_logger().info(f"map {len(self.maps)} {d.shape}")
        self.save()

    def save(self):
        np.savez_compressed(self.out, gt=np.array(self.gt), od=np.array(self.od),
                            mt=np.array([m[0] for m in self.maps]),
                            meta=np.array([m[1:4] for m in self.maps]),
                            **{f"map{i}": m[4] for i, m in enumerate(self.maps)})


def main():
    rclpy.init()
    n = Rec(sys.argv[1])
    try:
        rclpy.spin(n)
    except BaseException:
        pass
    n.save()


if __name__ == "__main__":
    main()
