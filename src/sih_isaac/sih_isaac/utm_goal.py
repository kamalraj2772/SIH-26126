#!/usr/bin/env python3
"""Send the rover to a goal given in operator coordinates, GPS-free.

    ros2 run sih_isaac utm_goal.py --lat 13.1147 --lon 80.1098   # WGS84 degrees
    ros2 run sih_isaac utm_goal.py -e 403562.52 -n 1450024.12    # UTM 44N metres
    ros2 run sih_isaac utm_goal.py --map-x 56 --map-y 34         # map frame direct

Lat/lon is converted to UTM 44N by the dependency-free transverse-Mercator
series in geodesy.py, then to the local map frame against the surveyed site
anchor baked into generated/world.json at survey (world-generation) time.
All of it is pure arithmetic on numbers typed at a keyboard -- no receiver, no
NavSatFix, nothing measures lat/lon online. Progress is reported in all frames.

If a recording run is active (see mission_run.py), the console output is also
written to <run>/goal.log and, when the mission ends, the run is finished
automatically: recorders stopped, highlight reel and PDF mission report built.
Pass --no-finish to leave the sim and nav stack running instead.
"""
import argparse
import json
import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

import mission_run
from geodesy import epsg_for, latlon_to_utm

WORLD_JSON = pathlib.Path(__file__).resolve().parents[1] / "generated/world.json"
FINISH_SH = mission_run.WS / "finish_mission.sh"


class Tee:
    """Mirror the mission console onto <run>/goal.log."""

    def __init__(self, stream, path):
        self.stream = stream
        self.log = open(path, "a", buffering=1)

    def write(self, s):
        self.stream.write(s)
        self.log.write(s)
        return len(s)

    def flush(self):
        self.stream.flush()
        self.log.flush()


def resolve_goal(args, world):
    """-> (map_x, map_y, description) from whichever coordinates were given."""
    anchor_e = world["site_datum"]["anchor_e"]
    anchor_n = world["site_datum"]["anchor_n"]

    if args.lat is not None and args.lon is not None:
        easting, northing, zone, north = latlon_to_utm(args.lat, args.lon)
        epsg = epsg_for(zone, north)
        if epsg != world["site_datum"]["epsg"]:
            sys.exit(f"[utm_goal] lat {args.lat} lon {args.lon} falls in EPSG:{epsg} "
                     f"(UTM zone {zone}{'N' if north else 'S'}), but the site is "
                     f"EPSG:{world['site_datum']['epsg']} -- that is a different "
                     f"part of the world, not a point on this {world['world_size']:.0f} m site")
        x, y = easting - anchor_e, northing - anchor_n
        return x, y, (f"WGS84 goal lat {args.lat:.6f} lon {args.lon:.6f} "
                      f"-> UTM {zone}N E {easting:.2f} N {northing:.2f} "
                      f"-> map ({x:.2f}, {y:.2f})")

    if args.easting is not None and args.northing is not None:
        x, y = args.easting - anchor_e, args.northing - anchor_n
        return x, y, (f"UTM 44N goal E {args.easting:.2f} N {args.northing:.2f}"
                      f" -> map ({x:.2f}, {y:.2f})   "
                      f"[anchor E {anchor_e:.2f} N {anchor_n:.2f}]")

    if args.map_x is not None and args.map_y is not None:
        x, y = args.map_x, args.map_y
        return x, y, (f"map goal ({x:.2f}, {y:.2f}) "
                      f"= UTM E {x + anchor_e:.2f} N {y + anchor_n:.2f}")

    return None, None, None


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--lat", type=float, help="WGS84 latitude, degrees")
    ap.add_argument("--lon", type=float, help="WGS84 longitude, degrees")
    ap.add_argument("-e", "--easting", type=float, help="UTM 44N easting, metres")
    ap.add_argument("-n", "--northing", type=float, help="UTM 44N northing, metres")
    ap.add_argument("--map-x", type=float)
    ap.add_argument("--map-y", type=float)
    ap.add_argument("--yaw-deg", type=float, default=0.0)
    ap.add_argument("--no-finish", action="store_true",
                    help="leave the sim and nav stack running when the mission ends "
                         "(no highlight reel, no report; finish_mission.sh does it later)")
    args = ap.parse_args()

    if (args.lat is None) != (args.lon is None):
        ap.error("--lat and --lon go together")

    w = json.loads(WORLD_JSON.read_text())
    anchor_e = w["site_datum"]["anchor_e"]
    anchor_n = w["site_datum"]["anchor_n"]

    x, y, what = resolve_goal(args, w)
    if what is None:
        ap.error("give --lat/--lon (WGS84), -e/-n (UTM 44N) or --map-x/--map-y")

    run = mission_run.active_run()
    if run is not None:
        sys.stdout = Tee(sys.stdout, run / "goal.log")
        print(f"[utm_goal] recording run: {run}")

    print(f"[utm_goal] {what}")

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
    try:
        while rclpy.ok() and "r" not in done:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        done.setdefault("r", "INTERRUPTED")
    # status 4 = SUCCEEDED (action_msgs/GoalStatus)
    reached = done.get("r") == 4
    print(f"[utm_goal] finished with status {done.get('r')}"
          + ("  -- GOAL REACHED" if reached else ""), flush=True)
    node.destroy_node()
    rclpy.shutdown()

    if run is None:
        return
    if args.no_finish:
        print(f"[utm_goal] --no-finish: sim and nav left running. Finish later with:"
              f"\n    {FINISH_SH} {run}")
        return
    print(f"[utm_goal] finishing run -> {run}", flush=True)
    sys.stdout.flush()
    # Hand this process over to the shell finisher (it owns shutdown ordering
    # and the videos/report). exec, not a child: on Ctrl-C, subprocess.call
    # SIGKILLs its child, which is how report_1401 lost its whole finish.
    os.execv(str(FINISH_SH), [str(FINISH_SH), str(run)])


if __name__ == "__main__":
    main()
