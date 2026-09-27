#!/usr/bin/env python3
"""Measure achieved yaw rate and forward speed vs command, in sim time.

Run with the sim already up:  python3 calibrate_yaw.py
Prints the ratio achieved/commanded so SKID_YAW_GAIN can be set.
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

CMD_W = 0.6
CMD_V = 0.6
PHASE_S = 12.0


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


class Cal(Node):
    def __init__(self):
        super().__init__("yaw_cal", parameter_overrides=[
            rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.sub = self.create_subscription(Odometry, "/ground_truth", self.cb, 10)
        self.t0 = None
        self.phase = "spin"
        self.yaw_acc = 0.0
        self.last_yaw = None
        self.p0 = None
        self.done = False

    def cb(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        yaw = yaw_of(m.pose.pose.orientation)
        p = m.pose.pose.position
        if self.t0 is None:
            self.t0 = t
            self.last_yaw = yaw
            self.p0 = (p.x, p.y)
        d = (yaw - self.last_yaw + math.pi) % (2 * math.pi) - math.pi
        self.yaw_acc += d
        self.last_yaw = yaw
        el = t - self.t0
        cmd = Twist()
        if self.phase == "spin":
            cmd.angular.z = CMD_W
            if el > PHASE_S:
                rate = self.yaw_acc / el
                print(f"[cal] spin: commanded {CMD_W} rad/s, achieved "
                      f"{rate:.4f} rad/s, ratio {rate / CMD_W:.4f}", flush=True)
                self.phase = "drive"
                self.t0, self.p0 = t, (p.x, p.y)
        elif self.phase == "drive":
            cmd.linear.x = CMD_V
            if el > PHASE_S:
                dist = math.hypot(p.x - self.p0[0], p.y - self.p0[1])
                print(f"[cal] drive: commanded {CMD_V} m/s, achieved "
                      f"{dist / el:.4f} m/s, ratio {dist / el / CMD_V:.4f}",
                      flush=True)
                self.phase = "stop"
                self.done = True
        self.pub.publish(cmd)


def main():
    rclpy.init()
    n = Cal()
    while rclpy.ok() and not n.done:
        rclpy.spin_once(n, timeout_sec=1.0)
    n.pub.publish(Twist())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
