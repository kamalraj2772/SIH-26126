#!/usr/bin/env python3
"""Send the rover to a UTM-44N goal (operator input), GPS-free.

    ros2 run sih_isaac utm_goal.py -e 403562.52 -n 1450024.12
    ros2 run sih_isaac utm_goal.py --map-x 56 --map-y 34      # map-frame direct

The easting/northing is converted to the local map frame against the surveyed
site anchor baked into generated/world.json at survey (world-generation) time.
Pure arithmetic -- no receiver, no NavSatFix, nothing measures lat/lon online.
Progress is reported back in both frames.
"""
import argparse
import json
import math
import pathlib
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

WORLD_JSON = pathlib.Path(__file__).resolve().parents[1] / "generated/world.json"


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("-e", "--easting", type=float)
    ap.add_argument("-n", "--northing", type=float)
    ap.add_argument("--map-x", type=float)
    ap.add_argument("--map-y", type=float)
    ap.add_argument("--yaw-deg", type=float, default=0.0)
    args = ap.parse_args()

    w = json.loads(WORLD_JSON.read_text())
    anchor_e = w["site_datum"]["anchor_e"]
    anchor_n = w["site_datum"]["anchor_n"]

    if args.easting is not None and args.northing is not None:
        x, y = args.easting - anchor_e, args.northing - anchor_n
        print(f"[utm_goal] UTM 44N goal E {args.easting:.2f} N {args.northing:.2f}"
              f" -> map ({x:.2f}, {y:.2f})   [anchor E {anchor_e:.2f} N {anchor_n:.2f}]")
    elif args.map_x is not None and args.map_y is not None:
        x, y = args.map_x, args.map_y
        print(f"[utm_goal] map goal ({x:.2f}, {y:.2f}) "
              f"= UTM E {x + anchor_e:.2f} N {y + anchor_n:.2f}")
    else:
        ap.error("give -e/-n (UTM) or --map-x/--map-y")

    half = w["world_size"] / 2.0
    if abs(x) > half or abs(y) > half:
        sys.exit(f"[utm_goal] goal outside the {w['world_size']:.0f} m site")

    rclpy.init()
    node = Node("utm_goal", parameter_overrides=[
        rclpy.parameter.Parameter("use_sim_time", value=True)])
    client = ActionClient(node, NavigateToPose, "navigate_to_pose")
    if not client.wait_for_server(timeout_sec=20.0):
        sys.exit("[utm_goal] navigate_to_pose action server not up")

    goal = NavigateToPose.Goal()
    goal.pose = PoseStamped()
    goal.pose.header.frame_id = "map"
    goal.pose.pose.position.x = float(x)
    goal.pose.pose.position.y = float(y)
    goal.pose.pose.orientation.z = math.sin(math.radians(args.yaw_deg) / 2)
    goal.pose.pose.orientation.w = math.cos(math.radians(args.yaw_deg) / 2)

    done = {}

    def fb(msg):
        f = msg.feedback
        p = f.current_pose.pose.position
        print(f"[utm_goal] pose map ({p.x:+7.2f},{p.y:+7.2f}) "
              f"UTM E {p.x + anchor_e:.2f} N {p.y + anchor_n:.2f}  "
              f"dist {f.distance_remaining:6.2f} m  "
              f"recoveries {f.number_of_recoveries}", flush=True)

    def on_resp(fut):
        gh = fut.result()
        if not gh.accepted:
            done["r"] = "REJECTED"
            return
        rf = gh.get_result_async()
        rf.add_done_callback(lambda f: done.update(r=f.result().status))

    client.send_goal_async(goal, feedback_callback=fb).add_done_callback(on_resp)
    while rclpy.ok() and "r" not in done:
        rclpy.spin_once(node, timeout_sec=0.5)
    # status 4 = SUCCEEDED (action_msgs/GoalStatus)
    print(f"[utm_goal] finished with status {done['r']}"
          + ("  -- GOAL REACHED" if done["r"] == 4 else ""))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
