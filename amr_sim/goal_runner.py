#!/usr/bin/env python3
"""Sends 20 pick-station goals one by one through nav2_simple_commander and logs
result, time, driven path length (ground truth) and final errors to a JSON file."""
import json
import math
import sys
import time
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry

# world-frame goals (x, y, yaw). Map frame = world shifted by the start pose (1, 1, 0).
GOALS = [(8.0, 1.0, 0), (9.35, 3.3, 1.57), (5.0, 3.3, 3.14), (11.2, 4.5, 1.57), (2.5, 1.0, 3.14),
         (9.35, 1.2, -1.57), (6.0, 7.2, 0), (1.3, 6.0, 1.57), (10.0, 7.2, 0), (5.5, 1.2, 0),
         (9.35, 3.0, 1.57), (3.0, 5.0, 0), (11.2, 3.6, -1.57), (7.5, 3.3, 3.14), (8.5, 1.2, 0),
         (1.5, 2.0, 1.57), (9.0, 5.0, 0), (4.5, 7.3, 3.14), (9.35, 1.5, 1.57), (1.0, 1.0, 0)]
OX, OY = 1.0, 1.0


def pose(nav, x, y, yaw):
    p = PoseStamped()
    p.header.frame_id = "map"
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x, p.pose.position.y = x - OX, y - OY
    p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return p


def main():
    out = sys.argv[1]
    rclpy.init()
    nav = BasicNavigator()
    st = {"gt": None, "amcl": None, "path": 0.0, "last": None}

    def on_gt(m):
        p = (m.pose.pose.position.x, m.pose.pose.position.y)
        if st["last"] is not None:
            st["path"] += math.dist(p, st["last"])
        st["last"] = p
        st["gt"] = p

    def on_amcl(m):
        st["amcl"] = (m.pose.pose.position.x + OX, m.pose.pose.position.y + OY)

    nav.create_subscription(Odometry, "ground_truth", on_gt, 20)
    nav.create_subscription(PoseWithCovarianceStamped, "amcl_pose", on_amcl, 10)
    init = PoseWithCovarianceStamped()
    nav.setInitialPose(pose(nav, 1.0, 1.0, 0.0))
    nav.waitUntilNav2Active()
    rows = []
    for i, (gx, gy, gyaw) in enumerate(GOALS):
        rclpy.spin_once(nav, timeout_sec=0.1)
        p0, t0 = st["path"], nav.get_clock().now().nanoseconds * 1e-9
        nav.goToPose(pose(nav, gx, gy, gyaw))
        wall0 = time.time()
        while not nav.isTaskComplete():
            rclpy.spin_once(nav, timeout_sec=0.05)
            if time.time() - wall0 > 150:
                nav.cancelTask()
        res = nav.getResult()
        for _ in range(10):
            rclpy.spin_once(nav, timeout_sec=0.05)
        t1 = nav.get_clock().now().nanoseconds * 1e-9
        g = st["gt"]
        a = st["amcl"]
        row = dict(goal=i + 1, x=gx, y=gy, result=str(res).split(".")[-1], time_s=round(t1 - t0, 1),
                   path_m=round(st["path"] - p0, 2), straight_m=None,
                   final_err_m=round(math.dist(g, (gx, gy)), 3),
                   amcl_err_m=round(math.dist(g, a), 3) if a else None)
        rows.append(row)
        nav.get_logger().info(f"GOAL {row}")
        json.dump(rows, open(out, "w"), indent=1)
    nav.get_logger().info("RUN_DONE")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
