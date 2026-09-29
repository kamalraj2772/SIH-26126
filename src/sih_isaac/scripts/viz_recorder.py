#!/usr/bin/env python3
"""Record the mission's internal state as three 1080p videos (no words).

    python3 viz_recorder.py --out <run_dir>        (SIGINT to stop)

Writes, alongside the Isaac camera captures:

    costmap.mp4     what the planner is allowed to drive on: the global costmap
                    (GeoTIFF slope prior + lidar obstacles + inflation), the
                    Smac A* route, and the rolling local costmap with the MPPI
                    candidate trajectories being scored.
    ekf.mp4         where the rover thinks it is: EKF estimate against ground
                    truth and against wheel-only dead reckoning, with the
                    error history. No GNSS is read anywhere in this stack.
    slam_graph.mp4  the map being built from lidar, the pose graph, and the
                    live ROS 2 data-flow graph with per-topic message rates.
    timebase.csv    sim<->wall clock samples, so the wall-clock RViz screen
                    grab can be cut on mission time later.
    viz_start.txt   the sim time at which these three videos start.

Paced on SIM time (one frame per 1/8 s of /clock), exactly like run_sim.py's
camera captures, so every video in a run shares one clock: seconds into the
file + viz_start.txt == sim time, the same timestamps mission_log.csv uses.
Each video renders in its own process (47 ms per 1080p frame, measured; Agg
holds the GIL, so threads would share one core), and ROS callbacks here are
never blocked by drawing. A frame the renderer cannot take in time is
repeated rather than dropped, so the timeline never drifts.

Pure picture, by the operator's request: no titles, captions, labels, legends
or number panels -- only the maps, plots and graph, with numeric tick marks
for scale (video.strip_words enforces it). Nothing here subscribes to a fix of
any kind; it only draws what the GPS-free stack already publishes.
"""
import argparse
import math
import multiprocessing as mp
import pathlib
import queue
import sys
import time

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "sih_isaac"))
import video                                                   # noqa: E402

import rclpy                                                   # noqa: E402
from geometry_msgs.msg import Twist                            # noqa: E402
from map_msgs.msg import OccupancyGridUpdate                   # noqa: E402
from nav_msgs.msg import OccupancyGrid, Odometry, Path         # noqa: E402
from rclpy.executors import ExternalShutdownException         # noqa: E402
from rclpy.node import Node                                    # noqa: E402
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,  # noqa: E402
                       ReliabilityPolicy)
from sensor_msgs.msg import LaserScan                          # noqa: E402
from visualization_msgs.msg import MarkerArray                 # noqa: E402

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST)
BEST = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                  history=HistoryPolicy.KEEP_LAST)


# --------------------------------------------------------------- helpers ---
def grid_np(msg):
    """OccupancyGrid -> (int16 array, imshow extent in metres)."""
    a = np.asarray(msg.data, dtype=np.int16).reshape(msg.info.height,
                                                     msg.info.width)
    o, r = msg.info.origin.position, msg.info.resolution
    return a, (o.x, o.x + msg.info.width * r, o.y, o.y + msg.info.height * r)


def costmap_cmap():
    """The Nav2 cost convention as a colormap: unknown, free, ramp, inscribed,
    lethal. Index = cost + 1, so -1 (unknown) lands on 0."""
    from matplotlib.colors import ListedColormap
    cols = np.zeros((102, 4))
    cols[0] = (0.10, 0.14, 0.20, 1.0)                    # -1 unknown
    cols[1] = (0.078, 0.161, 0.247, 1.0)                 #  0 free
    for c in range(1, 99):                               #  1..98 ramp
        f = c / 98.0
        cols[c + 1] = (0.16 + 0.72 * f, 0.36 + 0.28 * (1 - f) + 0.30 * f,
                       0.72 - 0.58 * f, 1.0)
    cols[100] = (0.94, 0.62, 0.22, 1.0)                  # 99 inscribed
    cols[101] = (0.92, 0.26, 0.22, 1.0)                  # 100 lethal
    return ListedColormap(cols)


def copy_grid(grid):
    """A private copy of an (array, extent) pair: the live ones are patched
    in place by the costmap update callbacks while renderers read them."""
    if grid is None:
        return None
    a, ext = grid
    return a.copy(), ext


def occupancy_rgb(a):
    """Occupancy grid -> RGB, readable on the dark frame: unknown stays flat,
    free is a slate wash, occupied is bright."""
    out = np.empty(a.shape + (3,), dtype=np.uint8)
    out[...] = (20, 36, 54)                      # -1 unknown
    free = a == 0
    out[free] = (36, 62, 90)
    occ = a >= 50
    out[occ] = (226, 236, 246)
    mid = (a > 0) & (a < 50)
    out[mid] = (110, 150, 190)
    return out


def fit_view(ax, pts, min_span, half):
    """Frame the part of the site the mission uses, not all 150 m of it.

    A square window (equal aspect) around every point of interest seen so far,
    at least `min_span` metres across, padded 15 %, clamped to the site. The
    content bounds only ever grow, so the view never zooms back in and jitters.
    """
    pts = np.asarray([q for q in pts if q is not None and
                      np.all(np.isfinite(q))], dtype=float).reshape(-1, 2)
    if not len(pts):
        return
    lo, hi = pts.min(axis=0), pts.max(axis=0)
    seen = getattr(ax, "_seen", None)        # raw content bounds, unpadded
    if seen is not None:
        lo, hi = np.minimum(lo, seen[0]), np.maximum(hi, seen[1])
    ax._seen = (lo, hi)
    span = min(max(float(np.max(hi - lo)) * 1.15, min_span), 2 * half)
    cx, cy = (lo + hi) / 2
    x0 = float(np.clip(cx - span / 2, -half, half - span))
    y0 = float(np.clip(cy - span / 2, -half, half - span))
    ax.set_xlim(x0, x0 + span)
    ax.set_ylim(y0, y0 + span)


def path_xy(msg):
    if msg is None or not msg.poses:
        return np.zeros((0, 2))
    return np.array([[p.pose.position.x, p.pose.position.y] for p in msg.poses])


class Rate:
    """Message rate over a sliding wall-clock window, for the graph panel."""

    def __init__(self, window=4.0):
        self.window = window
        self.stamps = []
        self.count = 0

    def tick(self):
        now = time.monotonic()
        self.count += 1
        self.stamps.append(now)
        if len(self.stamps) > 400:
            del self.stamps[:200]

    def hz(self):
        now = time.monotonic()
        recent = [s for s in self.stamps if now - s <= self.window]
        self.stamps = recent[-400:]
        if len(recent) < 2:
            return 0.0
        return (len(recent) - 1) / max(recent[-1] - recent[0], 1e-6)


# ------------------------------------------------------------ the renderers
class Renderer:
    """One video: a figure built once, its artists updated per snapshot.

    Each renderer runs in its own process (see render_worker): matplotlib's
    Agg drawing holds the GIL, so three renderers as threads would share one
    core -- 3 videos x 8 fps x 47 ms is more than a second of CPU per second.
    """

    name_ = "video"

    def __init__(self):
        self.fig = None

    def frame(self, snap):
        if self.fig is None:
            self.build()
            video.strip_words(self.fig)     # no words in any video
        self.update(snap)
        return video.fig_rgb(self.fig)

    # subclasses implement these
    def build(self):
        raise NotImplementedError

    def update(self, s):
        raise NotImplementedError



def render_worker(view, out, ffmpeg, fps, q):
    """Process body: snapshots in, mp4 out, one frame per sim-time tick.

    Every snapshot carries its tick index. If ticks were skipped (the parent
    could not hand them over in time), the last frame is repeated for them, so
    seconds into the file stay equal to seconds of sim time no matter what.
    """
    import signal
    signal.signal(signal.SIGINT, signal.SIG_IGN)   # the parent decides when
    r = VIEWS[view]()
    pipe = video.FramePipe(out, fps=fps, ffmpeg=ffmpeg)
    last_k, last = -1, None
    while True:
        item = q.get()
        if item is None:
            break
        k, snap = item
        if last is not None and k > last_k + 1:
            for _ in range(k - last_k - 1):
                pipe.write(last)
        try:
            last = r.frame(snap)
        except Exception as exc:                    # never kill the recording
            print(f"[viz] {view} frame {k} failed: {exc}", flush=True)
            if last is None:
                continue
        pipe.write(last)
        last_k = k
    pipe.close()


class CostmapView(Renderer):
    name_ = "costmap"

    def build(self):
        self.fig = video.new_figure()
        cm = costmap_cmap()
        self.ax = self.fig.add_axes([0.030, 0.045, 0.520, 0.920])
        self.gim = self.ax.imshow(np.zeros((2, 2)), origin="lower",
                                  extent=(-75, 75, -75, 75), cmap=cm,
                                  vmin=0, vmax=101, interpolation="nearest")
        self.gtrail, = self.ax.plot([], [], color=video.MUTED, lw=1.2,
                                    alpha=0.8, label="driven")
        self.gplan, = self.ax.plot([], [], color=video.GREEN, lw=2.6,
                                   label="global plan (Smac A*)")
        self.gscan = self.ax.scatter([], [], s=3.0, color=video.RED,
                                     label="lidar returns")
        self.grobot, = self.ax.plot([], [], marker="o", ms=11,
                                    color=video.GOLD, mec=video.NAVY, mew=1.6,
                                    ls="none", label="rover (EKF)")
        self.ggoal, = self.ax.plot([], [], marker="*", ms=22,
                                   color=video.VIOLET, mec=video.NAVY,
                                   mew=1.2, ls="none", label="commanded goal")
        self.ax.set_aspect("equal")

        self.lax = self.fig.add_axes([0.580, 0.045, 0.400, 0.920])
        self.lim = self.lax.imshow(np.zeros((2, 2)), origin="lower",
                                   extent=(-7, 7, -7, 7), cmap=cm, vmin=0,
                                   vmax=101, interpolation="nearest")
        self.lcand = self.lax.scatter([], [], s=1.2, color=video.SKY,
                                      alpha=0.55)
        self.lplan, = self.lax.plot([], [], color=video.GOLD, lw=2.4)
        self.lscan = self.lax.scatter([], [], s=4.0, color=video.RED)
        self.lrobot, = self.lax.plot([], [], marker="o", ms=9,
                                     color=video.GOLD, ls="none")
        self.lax.set_aspect("equal")

    def update(self, s):
        if s["gmap"] is not None:
            a, ext = s["gmap"]
            self.gim.set_data(a + 1)
            self.gim.set_extent(ext)
        fit_view(self.ax, [s["start"], s["goal"], (s["x"], s["y"])]
                 + list(s["plan"][::10]), 60.0, s["half"])
        self.gtrail.set_data(s["trail"][:, 0], s["trail"][:, 1])
        self.gplan.set_data(s["plan"][:, 0], s["plan"][:, 1])
        self.gscan.set_offsets(s["scan"] if len(s["scan"]) else np.zeros((0, 2)))
        self.grobot.set_data([s["x"]], [s["y"]])
        if s["goal"] is not None:
            self.ggoal.set_data([s["goal"][0]], [s["goal"][1]])

        if s["lmap"] is not None:
            a, ext = s["lmap"]
            self.lim.set_data(a + 1)
            self.lim.set_extent(ext)
            self.lax.set_xlim(ext[0], ext[1])
            self.lax.set_ylim(ext[2], ext[3])
        elif math.isfinite(s["x"]) and math.isfinite(s["y"]):
            self.lax.set_xlim(s["x"] - 7, s["x"] + 7)
            self.lax.set_ylim(s["y"] - 7, s["y"] + 7)
        self.lcand.set_offsets(s["cand"] if len(s["cand"]) else np.zeros((0, 2)))
        self.lplan.set_data(s["lplan"][:, 0], s["lplan"][:, 1])
        self.lscan.set_offsets(s["scan"] if len(s["scan"]) else np.zeros((0, 2)))
        self.lrobot.set_data([s["x"]], [s["y"]])



class EkfView(Renderer):
    name_ = "ekf"

    def build(self):
        self.fig = video.new_figure()
        self.ax = self.fig.add_axes([0.030, 0.045, 0.500, 0.920])
        self.gt, = self.ax.plot([], [], color=video.MUTED, lw=3.0, alpha=0.85,
                                label="ground truth (scoring only)")
        self.ek, = self.ax.plot([], [], color=video.SKY, lw=2.0,
                                label="EKF estimate  /odometry/filtered")
        self.wh, = self.ax.plot([], [], color=video.ORANGE, lw=1.6, ls="--",
                                label="wheel-only dead reckoning")
        self.now, = self.ax.plot([], [], marker="o", ms=11, color=video.GOLD,
                                 mec=video.NAVY, mew=1.6, ls="none",
                                 label="rover now")
        self.goal, = self.ax.plot([], [], marker="*", ms=22,
                                  color=video.VIOLET, mec=video.NAVY, mew=1.2,
                                  ls="none", label="commanded goal")
        self.ax.set_aspect("equal")

        self.eax = self.fig.add_axes([0.570, 0.540, 0.410, 0.420])
        self.eekf, = self.eax.plot([], [], color=video.SKY, lw=2.0,
                                   label="EKF")
        self.ewhl, = self.eax.plot([], [], color=video.ORANGE, lw=1.6,
                                   ls="--", label="wheel only")

        self.hax = self.fig.add_axes([0.570, 0.060, 0.410, 0.420])
        self.hgt, = self.hax.plot([], [], color=video.MUTED, lw=2.2)
        self.hek, = self.hax.plot([], [], color=video.SKY, lw=1.6)

    def update(self, s):
        self.gt.set_data(s["gt"][:, 0], s["gt"][:, 1])
        self.ek.set_data(s["ekf"][:, 0], s["ekf"][:, 1])
        self.wh.set_data(s["whl"][:, 0], s["whl"][:, 1])
        self.now.set_data([s["x"]], [s["y"]])
        if s["goal"] is not None:
            self.goal.set_data([s["goal"][0]], [s["goal"][1]])
        fit_view(self.ax, [s["start"], s["goal"], (s["x"], s["y"])]
                 + list(s["gt"][::25]) + list(s["whl"][::25]), 40.0,
                 s["half"])

        th, ee, ew = s["hist_t"], s["hist_eekf"], s["hist_ewhl"]
        self.eekf.set_data(th, ee)
        self.ewhl.set_data(th, ew)
        if len(th) > 1:
            self.eax.set_xlim(th[0], max(th[-1], th[0] + 1))
            top = max(1.0, float(np.nanmax(ew)) * 1.15 if len(ew) else 1.0)
            self.eax.set_ylim(0, top)
            self.hax.set_xlim(th[0], max(th[-1], th[0] + 1))
        ygt = np.degrees(np.unwrap(np.radians(s["hist_ygt"])))
        yek = np.degrees(np.unwrap(np.radians(s["hist_yekf"])))
        if len(ygt) and len(yek):
            # the estimate may differ from truth by a whole turn after
            # unwrapping; shift it onto the same branch
            yek += 360.0 * np.round((ygt[0] - yek[0]) / 360.0)
            lo = float(min(ygt.min(), yek.min()))
            hi = float(max(ygt.max(), yek.max()))
            pad = max((hi - lo) * 0.12, 10.0)
            self.hax.set_ylim(lo - pad, hi + pad)
        self.hgt.set_data(th, ygt)
        self.hek.set_data(th, yek)



class SlamGraphView(Renderer):
    name_ = "slam_graph"

    # A serpentine layout: data runs left to right along the top, down the
    # right edge into the costmaps, right to left along the bottom, and the
    # command closes the loop up the left edge. Laid out by hand because an
    # auto-routed graph of seven edges is unreadable at video size.
    BW, BH = 0.27, 0.17
    FLOW = [
        ("Isaac Sim 6\nrover, lidar, camera, IMU", 0.015, 0.56),
        ("robot_localization\nEKF  ->  odom / base_link", 0.365, 0.56),
        ("slam_toolbox / map_server\nmap -> odom,  /prior_map", 0.715, 0.56),
        ("controller_server\nMPPI", 0.015, 0.14),
        ("planner_server\nSmac A*", 0.365, 0.14),
        ("costmap_2d\nglobal + local", 0.715, 0.14),
    ]
    LINKS = [
        (0, 1, "/odom_wheel  /imu/data", "right"),
        (1, 2, "TF odom -> base_link", "right"),
        (0, 2, "/scan", "arc"),
        (2, 5, "/map  /prior_map  /scan", "down"),
        (5, 4, "costmap", "left"),
        (4, 3, "/plan", "left"),
        (3, 0, "/cmd_vel", "up"),
    ]

    def build(self):
        self.fig = video.new_figure()
        self.ax = self.fig.add_axes([0.030, 0.045, 0.500, 0.920])
        self.mim = self.ax.imshow(np.zeros((2, 2, 3), dtype=np.uint8),
                                  origin="lower", extent=(-75, 75, -75, 75),
                                  interpolation="nearest")
        self.hits = self.ax.scatter([], [], s=1.4, color=video.SKY, alpha=0.5,
                                    label="accumulated lidar returns")
        self.gnodes = self.ax.scatter([], [], s=26, color=video.GOLD,
                                      marker="o", label="pose-graph nodes")
        self.traj, = self.ax.plot([], [], color=video.GREEN, lw=1.8,
                                  label="estimated trajectory")
        self.robot, = self.ax.plot([], [], marker="o", ms=11, color=video.GOLD,
                                   mec=video.NAVY, mew=1.6, ls="none")
        self.ax.set_aspect("equal")
        w = video.world()["world_size"] / 2
        self.ax.set_xlim(-w, w)
        self.ax.set_ylim(-w, w)

        self.gax = self.fig.add_axes([0.560, 0.150, 0.420, 0.700])
        self.gax.set_xlim(0, 1)
        self.gax.set_ylim(0, 1)
        self.gax.axis("off")
        self._draw_graph()

    def _anchor(self, i, j, kind):
        """Where an edge leaves one box and meets the next, plus its label
        spot -- boxes are connected edge to edge, never centre to centre."""
        w, h = self.BW, self.BH
        (_, xs, ys), (_, xd, yd) = self.FLOW[i], self.FLOW[j]
        if kind == "right":
            a, b = (xs + w, ys + h / 2), (xd, yd + h / 2)
            lab, rad = ((a[0] + b[0]) / 2, ys + h + 0.05), 0.0
        elif kind == "left":
            a, b = (xs, ys + h / 2), (xd + w, yd + h / 2)
            lab, rad = ((a[0] + b[0]) / 2, ys + h + 0.05), 0.0
        elif kind == "down":
            a, b = (xs + w / 2, ys), (xd + w / 2, yd + h)
            lab, rad = (a[0], (a[1] + b[1]) / 2), 0.0
        elif kind == "up":
            a, b = (xs + w / 2, ys + h), (xd + w / 2, yd)
            lab, rad = (a[0], (a[1] + b[1]) / 2), 0.0
        else:                                              # arc over the row
            a, b = (xs + w / 2, ys + h), (xd + w / 2, yd + h)
            lab, rad = ((a[0] + b[0]) / 2, a[1] + 0.20), -0.32
        return a, b, lab, rad

    def _draw_graph(self):
        from matplotlib.patches import FancyArrowPatch, FancyBboxPatch
        for label, x, y in self.FLOW:
            self.gax.add_patch(FancyBboxPatch(
                (x, y), self.BW, self.BH, boxstyle="round,pad=0.006",
                facecolor=video.PANEL, edgecolor=video.EDGE, lw=1.4,
                zorder=3))
        self.arrows = []
        for i, j, topic, kind in self.LINKS:
            a, b, lab, rad = self._anchor(i, j, kind)
            arr = FancyArrowPatch(a, b, arrowstyle="-|>", mutation_scale=15,
                                  lw=1.6, color="#3C5F80", zorder=2,
                                  connectionstyle=f"arc3,rad={rad}",
                                  shrinkA=2, shrinkB=4)
            self.gax.add_patch(arr)
            self.arrows.append(arr)

    def update(self, s):
        if s["slam"] is not None:
            a, ext = s["slam"]
            self.mim.set_data(occupancy_rgb(a))
            self.mim.set_extent(ext)
        self.hits.set_offsets(s["hits"] if len(s["hits"]) else np.zeros((0, 2)))
        self.gnodes.set_offsets(s["graph"] if len(s["graph"])
                                else np.zeros((0, 2)))
        self.traj.set_data(s["ekf"][:, 0], s["ekf"][:, 1])
        self.robot.set_data([s["x"]], [s["y"]])

        for arr, (_, _, topic, _kind) in zip(self.arrows, self.LINKS):
            hz = s["rates"].get(topic.split(",")[0].split(" ")[0], 0.0)
            live = hz > 0.2
            arr.set_color(video.GREEN if live else "#3C5F80")
            arr.set_linewidth(2.6 if live else 1.4)



VIEWS = {"costmap": CostmapView, "ekf": EkfView, "slam_graph": SlamGraphView}


# ------------------------------------------------------------------- node --
class VizRecorder(Node):
    TOPICS = ["/scan", "/imu/data", "/odom_wheel", "/odometry/filtered",
              "/plan", "/local_plan", "/cmd_vel", "/camera/image",
              "/global_costmap/costmap", "/local_costmap/costmap"]

    def __init__(self, run, fps, workers):
        super().__init__("viz_recorder", parameter_overrides=[
            rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.run = pathlib.Path(run)
        self.fps = fps
        self.dt = 1.0 / fps
        self.world = video.world()
        self.anchor = (self.world["site_datum"]["anchor_e"],
                       self.world["site_datum"]["anchor_n"])
        self.berm = self.world["berm"]

        self.gmap = self.lmap = self.slam = self.prior = None
        self.gmsg = self.lmsg = None
        self.plan = self.lplan = np.zeros((0, 2))
        self.cand = np.zeros((0, 2))
        self.graph = np.zeros((0, 2))
        self.scan = np.zeros((0, 2))
        self.hits = np.zeros((0, 2))
        self.goal = None
        self.cmd = (0.0, 0.0)
        self.pose = (self.world["start"]["x"], self.world["start"]["y"],
                     math.radians(self.world["start"]["yaw_deg"]))
        self.gt_pose = self.pose
        self.whl_pose = self.pose
        self.moved = False
        self.tracks = {k: [] for k in ("gt", "ekf", "whl")}
        self.hist = {k: [] for k in ("t", "eekf", "ewhl", "ygt", "yekf")}
        self.rate = {t: Rate() for t in self.TOPICS}

        self.workers = workers
        self.dropped = {name: 0 for name in workers}

        self.tb = open(self.run / "timebase.csv", "w", buffering=1)
        self.tb.write("sim_t,wall_t\n")
        self.last_tb = -1e9

        sub = self.create_subscription
        sub(OccupancyGrid, "/global_costmap/costmap", self.on_gmap, LATCHED)
        sub(OccupancyGridUpdate, "/global_costmap/costmap_updates",
            lambda m: self.on_update(m, "g"), 5)
        sub(OccupancyGrid, "/local_costmap/costmap", self.on_lmap, LATCHED)
        sub(OccupancyGridUpdate, "/local_costmap/costmap_updates",
            lambda m: self.on_update(m, "l"), 5)
        sub(OccupancyGrid, "/map", self.on_slam, LATCHED)
        sub(OccupancyGrid, "/prior_map", self.on_prior, LATCHED)
        sub(Path, "/plan", self.on_plan, 5)
        sub(Path, "/local_plan", self.on_lplan, 5)
        sub(MarkerArray, "/trajectories", self.on_cand, 2)
        sub(MarkerArray, "/slam_toolbox/graph_visualization", self.on_graph, 2)
        sub(LaserScan, "/scan", self.on_scan, BEST)
        sub(Odometry, "/odometry/filtered", self.on_ekf, 10)
        sub(Odometry, "/odom_wheel", self.on_whl, 10)
        sub(Odometry, "/ground_truth", self.on_gt, 10)
        sub(Twist, "/cmd_vel", self.on_cmd, 10)
        try:                       # raw: counted for the graph panel, not decoded
            from sensor_msgs.msg import Image
            sub(Image, "/camera/image", lambda _m: self.rate["/camera/image"].tick(),
                BEST, raw=True)
        except TypeError:
            pass

        self.start_sim = None
        self.next_frame = None
        self.frames = 0
        self.create_timer(0.02, self.tick)
        print(f"[viz] recording costmap / ekf / slam_graph -> {self.run} "
              f"at {fps} fps of sim time", flush=True)

    # ------------------------------------------------------------ callbacks
    def sim_now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_gmap(self, m):
        self.rate["/global_costmap/costmap"].tick()
        self.gmsg = m
        self.gmap = grid_np(m)

    def on_lmap(self, m):
        self.rate["/local_costmap/costmap"].tick()
        self.lmsg = m
        self.lmap = grid_np(m)

    def on_update(self, m, which):
        """Nav2 sends the full grid once, then rectangular patches."""
        base = self.gmap if which == "g" else self.lmap
        msg = self.gmsg if which == "g" else self.lmsg
        if base is None or msg is None:
            return
        a, ext = base
        try:
            patch = np.asarray(m.data, dtype=np.int16).reshape(m.height, m.width)
            a[m.y:m.y + m.height, m.x:m.x + m.width] = patch
        except ValueError:
            return
        if which == "g":
            self.gmap = (a, ext)
            self.rate["/global_costmap/costmap"].tick()
        else:
            self.lmap = (a, ext)
            self.rate["/local_costmap/costmap"].tick()

    def on_slam(self, m):
        self.slam = grid_np(m)

    def on_prior(self, m):
        self.prior = grid_np(m)

    def on_plan(self, m):
        self.rate["/plan"].tick()
        self.plan = path_xy(m)
        if len(self.plan):
            goal = tuple(float(v) for v in self.plan[-1])
            if self.goal is None or math.dist(goal, self.goal) > 0.05:
                # the goal as the planner saw it, for runs whose goal came
                # from RViz rather than utm_goal.py (which logs its own)
                (self.run / "goal_seen.txt").write_text(
                    f"{goal[0]:.3f} {goal[1]:.3f}\n")
            self.goal = goal

    def on_lplan(self, m):
        self.rate["/local_plan"].tick()
        self.lplan = path_xy(m)

    def on_cand(self, m):
        pts = []
        for mk in m.markers:
            if mk.points:
                pts.extend((p.x, p.y) for p in mk.points)
            elif mk.type in (2, 3):                       # SPHERE / CYLINDER
                pts.append((mk.pose.position.x, mk.pose.position.y))
        self.cand = np.array(pts) if pts else np.zeros((0, 2))

    def on_graph(self, m):
        pts = []
        for mk in m.markers:
            if mk.points:
                pts.extend((p.x, p.y) for p in mk.points)
            else:
                pts.append((mk.pose.position.x, mk.pose.position.y))
        self.graph = np.array(pts) if pts else np.zeros((0, 2))

    def on_scan(self, m):
        self.rate["/scan"].tick()
        r = np.asarray(m.ranges, dtype=np.float64)
        ang = m.angle_min + np.arange(len(r)) * m.angle_increment
        ok = np.isfinite(r) & (r > m.range_min) & (r < min(m.range_max, 40.0))
        if not ok.any():
            self.scan = np.zeros((0, 2))
            return
        x, y, yaw = self.pose                 # lidar sits on the base_link axis
        c, s = math.cos(yaw), math.sin(yaw)
        lx, ly = r[ok] * np.cos(ang[ok]), r[ok] * np.sin(ang[ok])
        pts = np.column_stack([x + c * lx - s * ly, y + s * lx + c * ly])
        self.scan = pts
        if len(pts):                          # thin the accumulated cloud
            keep = pts[::7]
            self.hits = (np.vstack([self.hits, keep])[-24000:]
                         if len(self.hits) else keep)

    def on_ekf(self, m):
        self.rate["/odometry/filtered"].tick()
        p, q = m.pose.pose.position, m.pose.pose.orientation
        if not all(map(math.isfinite, (p.x, p.y, q.z, q.w))):
            return                       # a diverged filter must not kill a frame
        self.pose = (p.x, p.y, video.yaw_of(q.z, q.w))
        self.tracks["ekf"].append((p.x, p.y))

    def on_whl(self, m):
        self.rate["/odom_wheel"].tick()
        p = m.pose.pose.position
        if not (math.isfinite(p.x) and math.isfinite(p.y)):
            return
        self.whl_pose = (p.x, p.y, 0.0)
        self.tracks["whl"].append((p.x, p.y))

    def on_gt(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        if not all(map(math.isfinite, (p.x, p.y, q.z, q.w))):
            return
        self.gt_pose = (p.x, p.y, video.yaw_of(q.z, q.w))
        self.tracks["gt"].append((p.x, p.y))

    def on_cmd(self, m):
        self.rate["/cmd_vel"].tick()
        self.cmd = (m.linear.x, m.angular.z)
        if abs(m.linear.x) > 0.05:
            self.moved = True

    # ---------------------------------------------------------------- pacing
    def tick(self):
        now = self.sim_now()
        if now <= 0.0:
            return                                   # /clock not flowing yet
        if self.start_sim is None:
            self.start_sim = now
            self.next_frame = now
            # video t = 0 is this sim time; make_highlight.py needs it to cut
            # these files on the same mission clock as the Isaac captures
            (self.run / "viz_start.txt").write_text(f"{now:.3f}\n")
        n = 0
        while self.next_frame <= now and n < 4:      # bounded catch-up
            self.emit(self.next_frame)
            self.next_frame += self.dt
            n += 1
        if now - self.last_tb >= 1.0:
            self.last_tb = now
            self.tb.write(f"{now:.3f},{time.time():.3f}\n")

    def emit(self, sim_t):
        t = sim_t
        x, y, yaw = self.pose
        gx, gy, gyaw = self.gt_pose
        wx, wy, _ = self.whl_pose
        err_ekf = math.hypot(gx - x, gy - y)
        err_whl = math.hypot(gx - wx, gy - wy)

        self.hist["t"].append(t)
        self.hist["eekf"].append(err_ekf)
        self.hist["ewhl"].append(err_whl)
        self.hist["ygt"].append(math.degrees(gyaw))
        self.hist["yekf"].append(math.degrees(yaw))
        for k in self.hist:
            if len(self.hist[k]) > 20000:
                del self.hist[k][::2]

        def arr(name):
            v = self.tracks[name]
            return np.array(v[::2]) if len(v) > 1 else np.zeros((0, 2))

        gmap = copy_grid(self.gmap)
        gres = self.gmsg.info.resolution if self.gmsg else 0.0
        gcells = (f"{self.gmsg.info.width}x{self.gmsg.info.height}"
                  if self.gmsg else "-")
        glethal = (100.0 * float(np.mean(gmap[0] >= 99)) if gmap else 0.0)
        slam = copy_grid(self.slam or self.prior)
        known = (100.0 * float(np.mean(slam[0] >= 0)) if slam else 0.0)
        plan_len = (float(np.sum(np.hypot(*np.diff(self.plan, axis=0).T)))
                    if len(self.plan) > 1 else 0.0)
        dist_goal = (math.hypot(x - self.goal[0], y - self.goal[1])
                     if self.goal else float("nan"))

        rates = {t_: self.rate[t_].hz() for t_ in self.TOPICS}
        rates["TF"] = rates["/odometry/filtered"]
        rates["costmap"] = rates["/global_costmap/costmap"]
        rates["/map"] = rates["/global_costmap/costmap"]
        table = [(t_, (rates[t_], self.rate[t_].count)) for t_ in self.TOPICS]

        snap = dict(
            t=t, x=x, y=y, yaw=yaw,
            start=(self.world["start"]["x"], self.world["start"]["y"]),
            half=self.world["world_size"] / 2.0,
            utm_e=x + self.anchor[0], utm_n=y + self.anchor[1],
            goal=self.goal, v=self.cmd[0], w=self.cmd[1],
            gmap=gmap, lmap=copy_grid(self.lmap), slam=slam, gres=gres, gcells=gcells,
            glethal=glethal, lres=(self.lmsg.info.resolution
                                   if self.lmsg else 0.0),
            plan=self.plan, lplan=self.lplan, cand=self.cand,
            ncand=len(self.cand), plan_len=plan_len, dist_goal=dist_goal,
            scan=self.scan, hits=self.hits, graph=self.graph, known=known,
            trail=arr("gt"), gt=arr("gt"), ekf=arr("ekf"), whl=arr("whl"),
            err_ekf=err_ekf, err_whl=err_whl,
            hist_t=np.array(self.hist["t"]),
            hist_eekf=np.array(self.hist["eekf"]),
            hist_ewhl=np.array(self.hist["ewhl"]),
            hist_ygt=np.array(self.hist["ygt"]),
            hist_yekf=np.array(self.hist["yekf"]),
            rates=rates, rate_table=table)
        for name, (_proc, q) in self.workers.items():
            try:
                q.put_nowait((self.frames, snap))
            except queue.Full:            # the worker repeats a frame for it
                self.dropped[name] += 1
        self.frames += 1

    def shutdown(self):
        print(f"[viz] finalizing {self.frames} frames per video", flush=True)
        self.tb.close()
        for name, n in self.dropped.items():
            if n:
                print(f"[viz] {name}: {n} frames repeated (renderer fell "
                      "behind; timeline preserved)", flush=True)


def stop_workers(workers):
    for _proc, q in workers.values():
        q.put(None)
    for name, (proc, _q) in workers.items():
        proc.join(timeout=240)
        if proc.is_alive():
            print(f"[viz] {name} renderer did not finish; terminating",
                  flush=True)
            proc.terminate()
    print("[viz] visualisation recordings finalized", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True, help="run directory")
    ap.add_argument("--ffmpeg", default=None)
    ap.add_argument("--fps", type=int, default=video.LIVE_FPS)
    args = ap.parse_args()

    # Renderer processes first, before rclpy exists in this process: they
    # need no ROS at all, and spawning (not forking) keeps them clear of it.
    ctx = mp.get_context("spawn")
    ffmpeg = args.ffmpeg or video.ffmpeg_bin()
    run = pathlib.Path(args.out)
    workers = {}
    for name in VIEWS:
        q = ctx.Queue(maxsize=64)
        proc = ctx.Process(target=render_worker, name=f"viz-{name}",
                           args=(name, str(run / f"{name}.mp4"), ffmpeg,
                                 args.fps, q), daemon=True)
        proc.start()
        workers[name] = (proc, q)

    rclpy.init()
    node = VizRecorder(run, args.fps, workers)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
        stop_workers(workers)


if __name__ == "__main__":
    main()
