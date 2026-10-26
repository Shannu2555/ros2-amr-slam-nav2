#!/usr/bin/env python3
"""Mapping drive: follows a fixed waypoint loop through the warehouse with a simple
pure-pursuit controller on ground truth (stand-in for a human teleoperating the robot)."""
import math
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

WP = [(1.0, 1.0), (8.6, 1.0), (9.35, 1.6), (9.35, 3.6), (11.2, 3.6), (11.3, 5.0),
      (9.6, 5.2), (5.0, 5.0), (1.2, 5.2), (1.2, 7.2), (5.0, 7.3), (8.5, 7.2), (9.6, 5.2),
      (9.4, 3.3), (5.0, 3.3), (1.5, 3.3), (1.5, 1.2), (1.0, 1.0)]


class Tour(Node):
    def __init__(self):
        super().__init__("tour")
        self.i = 0
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(Odometry, "ground_truth", self.cb, 10)

    def cb(self, m):
        if self.i >= len(WP):
            self.pub.publish(Twist())
            return
        x, y = m.pose.pose.position.x, m.pose.pose.position.y
        o = m.pose.pose.orientation
        yaw = math.atan2(2 * o.w * o.z, 1 - 2 * o.z * o.z)
        gx, gy = WP[self.i]
        d = math.hypot(gx - x, gy - y)
        if d < 0.15:
            self.i += 1
            self.get_logger().info(f"waypoint {self.i}/{len(WP)}")
            return
        e = math.atan2(gy - y, gx - x) - yaw
        e = math.atan2(math.sin(e), math.cos(e))
        t = Twist()
        t.angular.z = max(-1.0, min(1.0, 2.0 * e))
        t.linear.x = 0.35 * max(0.0, math.cos(e)) ** 3
        self.pub.publish(t)


def main():
    rclpy.init()
    n = Tour()
    while rclpy.ok() and n.i < len(WP):
        rclpy.spin_once(n, timeout_sec=0.1)
    n.get_logger().info("TOUR_DONE")


if __name__ == "__main__":
    main()
