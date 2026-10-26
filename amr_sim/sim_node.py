#!/usr/bin/env python3
"""2D diff-drive AMR simulator: /cmd_vel in, /odom + TF odom->base_link, /scan
(2D lidar raycast against the warehouse grid), /ground_truth, /clock out.
Wheel odometry has small noise and calibration error so it drifts like a real base."""
import math
import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
import world


def q(yaw):
    return 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)


class Sim(Node):
    def __init__(self):
        super().__init__("amr_sim")
        p = self.declare_parameter
        self.x = np.array([p("x0", 1.0).value, p("y0", 1.0).value, p("yaw0", 0.0).value])
        self.rtf = p("real_time_factor", 1.0).value
        self.rng = np.random.default_rng(p("seed", 3).value)
        self.grid = world.build()
        self.xo = np.zeros(3)
        self.dt = 0.01
        self.k = 0
        self.cmd = (0.0, 0.0)
        self.cmd_t = 0.0
        self.n_beams, self.rmax = 360, 8.0
        self.angles = np.linspace(-math.pi, math.pi, self.n_beams, endpoint=False)
        self.collisions = 0
        self.pub_odom = self.create_publisher(Odometry, "odom", 20)
        self.pub_gt = self.create_publisher(Odometry, "ground_truth", 20)
        self.pub_scan = self.create_publisher(LaserScan, "scan", 10)
        self.pub_clock = self.create_publisher(Clock, "/clock", 10)
        self.tfb = TransformBroadcaster(self)
        self.stb = StaticTransformBroadcaster(self)
        self.create_subscription(Twist, "cmd_vel", self.on_cmd, 10)
        self.create_timer(self.dt / self.rtf, self.step)
        self.sent_static = False

    def on_cmd(self, m):
        self.cmd = (max(-0.5, min(0.5, m.linear.x)), max(-1.5, min(1.5, m.angular.z)))
        self.cmd_t = self.k * self.dt

    def stamp(self):
        ns = int(round(self.k * self.dt * 1e9)) + 10 ** 9
        return Time(sec=ns // 10 ** 9, nanosec=ns % 10 ** 9)

    def occupied(self, x, y):
        c, r = int(x / world.RES), int(y / world.RES)
        if r < 0 or c < 0 or r >= self.grid.shape[0] or c >= self.grid.shape[1]:
            return True
        return self.grid[r, c]

    def footprint_hits(self, x, y):
        for a in np.linspace(0, 2 * math.pi, 16, endpoint=False):
            if self.occupied(x + 0.18 * math.cos(a), y + 0.18 * math.sin(a)):
                return True
        return False

    def step(self):
        t = self.k * self.dt
        v, w = self.cmd if t - self.cmd_t < 0.5 else (0.0, 0.0)  # cmd timeout
        nx = self.x + self.dt * np.array([v * math.cos(self.x[2]), v * math.sin(self.x[2]), w])
        if self.footprint_hits(nx[0], nx[1]):
            if self.k - getattr(self, "last_hit", -999) > 50:
                self.get_logger().warn(f"CONTACT at x={self.x[0]:.2f} y={self.x[1]:.2f} t={self.k * self.dt:.1f}")
            self.last_hit = self.k
            self.collisions += 1
            v = 0.0
            nx = self.x + self.dt * np.array([0.0, 0.0, w])
        self.x = nx
        # wheel odom: 1.5 % scale error on rotation, noise
        vo = v * 1.005 + self.rng.normal(0, 0.005) * (abs(v) > 1e-4)
        wo = w * 1.015 + self.rng.normal(0, 0.01) * (abs(w) > 1e-4 or abs(v) > 1e-4)
        self.xo += self.dt * np.array([vo * math.cos(self.xo[2]), vo * math.sin(self.xo[2]), wo])
        self.k += 1
        st = self.stamp()
        self.pub_clock.publish(Clock(clock=st))
        if not self.sent_static:
            s = TransformStamped()
            s.header.stamp = st
            s.header.frame_id, s.child_frame_id = "base_link", "laser"
            s.transform.rotation.w = 1.0
            self.stb.sendTransform(s)
            self.sent_static = True
        if self.k % 2 == 0:
            self.pub_odom.publish(self.odom(st, self.xo, vo, wo, "odom"))
            self.pub_gt.publish(self.odom(st, self.x, v, w, "map"))
            tf = TransformStamped()
            tf.header.stamp = st
            tf.header.frame_id, tf.child_frame_id = "odom", "base_link"
            tf.transform.translation.x, tf.transform.translation.y = float(self.xo[0]), float(self.xo[1])
            (tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z,
             tf.transform.rotation.w) = q(float(self.xo[2]))
            self.tfb.sendTransform(tf)
        if self.k % 10 == 0:
            self.pub_scan.publish(self.scan(st))

    def scan(self, st):
        a = self.x[2] + self.angles
        r = np.arange(0.12, self.rmax, world.RES / 2)
        xs = self.x[0] + np.outer(np.cos(a), r)
        ys = self.x[1] + np.outer(np.sin(a), r)
        c, rr = (xs / world.RES).astype(int), (ys / world.RES).astype(int)
        inside = (c >= 0) & (rr >= 0) & (c < self.grid.shape[1]) & (rr < self.grid.shape[0])
        hit = np.ones_like(inside)
        hit[inside] = self.grid[rr[inside], c[inside]]
        first = np.where(hit.any(1), hit.argmax(1), -1)
        ranges = np.where(first >= 0, r[np.clip(first, 0, None)], float("inf"))
        ranges = ranges + self.rng.normal(0, 0.01, ranges.shape)
        m = LaserScan()
        m.header.stamp = st
        m.header.frame_id = "laser"
        m.angle_min, m.angle_max = float(self.angles[0]), float(self.angles[-1])
        m.angle_increment = float(self.angles[1] - self.angles[0])
        m.scan_time, m.time_increment = 0.1, 0.0
        m.range_min, m.range_max = 0.12, self.rmax
        m.ranges = [float(v) for v in ranges]
        return m

    def odom(self, st, x, v, w, frame):
        o = Odometry()
        o.header.stamp = st
        o.header.frame_id, o.child_frame_id = frame, "base_link"
        o.pose.pose.position.x, o.pose.pose.position.y = float(x[0]), float(x[1])
        (o.pose.pose.orientation.x, o.pose.pose.orientation.y, o.pose.pose.orientation.z,
         o.pose.pose.orientation.w) = q(float(x[2]))
        o.twist.twist.linear.x, o.twist.twist.angular.z = float(v), float(w)
        return o


def main():
    rclpy.init()
    n = Sim()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
