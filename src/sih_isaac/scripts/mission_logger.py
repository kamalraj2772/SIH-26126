#!/usr/bin/env python3
"""Log mission telemetry to CSV at 5 Hz: ground truth vs EKF vs commands.

    python3 mission_logger.py --out mission_log.csv     (SIGINT to stop)
"""
import argparse
import csv
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


class Logger(Node):
    def __init__(self, out):
        super().__init__("mission_logger", parameter_overrides=[
            rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.f = open(out, "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow(["t", "gt_x", "gt_y", "gt_yaw",
                         "ekf_x", "ekf_y", "cmd_v", "cmd_w"])
        self.ekf = (float("nan"), float("nan"))
        self.cmd = (0.0, 0.0)
        self.last_t = -1.0
        self.create_subscription(Odometry, "/ground_truth", self.on_gt, 20)
        self.create_subscription(Odometry, "/odometry/filtered",
                                 self.on_ekf, 20)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)

    def on_ekf(self, m):
        p = m.pose.pose.position
        self.ekf = (p.x, p.y)

    def on_cmd(self, m):
        self.cmd = (m.linear.x, m.angular.z)

    def on_gt(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if t - self.last_t < 0.2:
            return
        self.last_t = t
        p, q = m.pose.pose.position, m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        self.w.writerow([f"{t:.2f}", f"{p.x:.3f}", f"{p.y:.3f}",
                         f"{yaw:.4f}", f"{self.ekf[0]:.3f}",
                         f"{self.ekf[1]:.3f}", f"{self.cmd[0]:.3f}",
                         f"{self.cmd[1]:.3f}"])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    rclpy.init()
    node = Logger(args.out)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.f.flush()
        node.f.close()


if __name__ == "__main__":
    main()
