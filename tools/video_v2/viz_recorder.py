#!/usr/bin/env python3
"""Records what RViz would show during a Nav2 run: /particle_cloud (AMCL), /plan (global path),
/amcl_pose, /scan, /ground_truth. Saves to npz for the demo video."""
import sys, pickle
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from nav2_msgs.msg import ParticleCloud
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan

def ts(h): return h.stamp.sec + h.stamp.nanosec * 1e-9
def yaw(o): return float(np.arctan2(2 * o.w * o.z, 1 - 2 * o.z * o.z))

class R(Node):
    def __init__(s, out):
        super().__init__("viz_recorder"); s.out = out
        s.pc, s.plan, s.amcl, s.scan, s.gt = [], [], [], [], []
        be = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        s.create_subscription(ParticleCloud, "particle_cloud", s.on_pc, be)
        s.create_subscription(Path, "plan", s.on_plan, 10)
        s.create_subscription(PoseWithCovarianceStamped, "amcl_pose", s.on_amcl, 10)
        s.create_subscription(LaserScan, "scan", s.on_scan, qos_profile_sensor_data)
        s.create_subscription(Odometry, "ground_truth", s.on_gt, 20)
        s.create_timer(5.0, s.save)
    def now(s): return s.get_clock().now().nanoseconds * 1e-9
    def on_pc(s, m):
        p = np.array([(q.pose.position.x, q.pose.position.y, yaw(q.pose.orientation), q.weight) for q in m.particles], np.float32)
        if len(p) > 400: p = p[np.random.default_rng(0).choice(len(p), 400, replace=False)]
        s.pc.append((ts(m.header), len(m.particles), p))
    def on_plan(s, m):
        if not m.poses: return
        s.plan.append((ts(m.header) or s.now(), np.array([(q.pose.position.x, q.pose.position.y) for q in m.poses], np.float32)))
    def on_amcl(s, m):
        s.amcl.append((ts(m.header), m.pose.pose.position.x, m.pose.pose.position.y, yaw(m.pose.pose.orientation)))
    def on_scan(s, m):
        s.scan.append((ts(m.header), np.array(m.ranges, np.float32)))
    def on_gt(s, m):
        s.gt.append((ts(m.header), m.pose.pose.position.x, m.pose.pose.position.y, yaw(m.pose.pose.orientation)))
    def save(s):
        pickle.dump(dict(pc=s.pc, plan=s.plan, amcl=s.amcl, scan=s.scan, gt=s.gt), open(s.out + ".tmp", "wb"))
        import os; os.replace(s.out + ".tmp", s.out)

def main():
    rclpy.init(); n = R(sys.argv[1])
    try: rclpy.spin(n)
    except BaseException: pass
    n.save()
if __name__ == "__main__": main()
