"""SIH26126 navigation stack (runs beside the Isaac Sim process).

    ros2 launch sih_isaac nav.launch.py            # stack + RViz
    ros2 launch sih_isaac nav.launch.py rviz:=false
    ros2 launch sih_isaac nav.launch.py record:=none   # no screen capture

record:=auto (the default) attaches a full-screen capture, the telemetry
logger and the visualisation recorder to the recording run that run_isaac.sh
started, so this terminal, the RViz window and the other two terminals all land
in <run>/rviz.mp4, and <run>/costmap.mp4, ekf.mp4 and slam_graph.mp4 are drawn
live from this stack's own topics. With no such run active it does nothing.
record:=<dir> targets a directory explicitly.

TF ownership (exactly one publisher per edge):
    map -> odom            slam_toolbox (lidar SLAM, GPS-free)
    odom -> base_link      robot_localization EKF (wheel odom + IMU)
    base_link -> sensors   robot_state_publisher (URDF) + one static TF for
                           lidar_link (the mast-top RTX lidar)
Isaac Sim publishes no TF. No GNSS anywhere in this file or below it.
"""
import os
import pathlib
import shlex
import subprocess
import sys
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG_SRC = pathlib.Path("/home/qbotix-rover/sih_ws/src/sih_isaac")
sys.path.insert(0, str(PKG_SRC / "sih_isaac"))
import mission_run                                             # noqa: E402


def _screen_size(display):
    """Root-window size on `display`, as (w, h) forced even, or None."""
    try:
        out = subprocess.run(["xwininfo", "-root"], check=True, timeout=10,
                             capture_output=True, text=True,
                             env={**os.environ, "DISPLAY": display}).stdout
    except (OSError, subprocess.SubprocessError):
        return None
    for line in out.splitlines():
        if line.strip().startswith("-geometry"):
            geom = line.split()[1].split("+")[0].split("-")[0]
            try:
                w, h = (int(v) for v in geom.lower().split("x"))
            except ValueError:
                return None
            return w // 2 * 2, h // 2 * 2
    return None


def _pidded_process(pidfile, cmd):
    """Run cmd, leaving its own pid in pidfile so finish_mission.sh can stop it.

    `exec` keeps the pid the shell already wrote, and each recorder is stopped
    individually (SIGINT) before this launch shuts down, so ffmpeg and the
    logger always get to finalize their files unhurried.
    """
    quoted = " ".join(shlex.quote(str(a)) for a in cmd)
    return ExecuteProcess(
        cmd=["bash", "-c", f"echo $$ > {shlex.quote(str(pidfile))}; exec {quoted}"],
        output="log")


def _recording_actions(context, *_args, **_kwargs):
    """Screen capture + telemetry logger, attached to the active run."""
    mode = LaunchConfiguration("record").perform(context)
    if mode in ("none", "false", "0", ""):
        return []
    if mode in ("auto", "true", "1"):
        run = mission_run.active_run()
        if run is None:
            print("[nav] record:=auto but no recording sim is running "
                  "(start terminal 1 with ./run_isaac.sh) -- not capturing")
            return []
    else:
        run = pathlib.Path(mode)
        run.mkdir(parents=True, exist_ok=True)

    # so finish_mission.sh can stop this terminal's whole group
    (run / "nav.pid").write_text(f"{os.getpid()}\n")

    actions = []
    display = os.environ.get("DISPLAY", ":1")
    size = _screen_size(display)
    if size is None:
        print(f"[nav] WARNING: no X display on {display}; skipping screen capture")
    else:
        w, h = size
        # x11grab refuses a rectangle past the screen edge, so grab the whole
        # root window: RViz and all three terminals are in it as arranged.
        scale = [] if (w, h) == (1920, 1080) else \
            ["-vf", "scale=1920:1080:flags=lanczos,format=yuv420p"]
        # Measured on this ffmpeg (imageio-ffmpeg 7.0.2 static): while grabbing
        # x11, it never acts on SIGINT or SIGTERM -- 90 s and still capturing --
        # and a plain mp4 killed mid-write has no moov atom, i.e. is unplayable.
        # A fragmented mp4 carries its index in each fragment, so the file stays
        # playable however the process dies. Keyframe every 2 s bounds the loss.
        # The screen grab runs on wall-clock time while everything else runs
        # on sim time; stamping the moment it starts lets viz_recorder.py's
        # sim<->wall table convert mission times into rviz.mp4 times later.
        (run / "rviz_start.txt").write_text(f"{time.time():.3f}\n")
        actions.append(_pidded_process(
            run / "rviz_ffmpeg.pid",
            [mission_run.ffmpeg_bin(), "-y", "-nostdin", "-loglevel", "error",
             "-f", "x11grab", "-framerate", "15", "-video_size", f"{w}x{h}",
             "-i", f"{display}+0,0", *scale,
             "-c:v", "libx264", "-preset", "veryfast", "-crf", "21",
             "-pix_fmt", "yuv420p", "-g", "30",
             "-movflags", "+frag_keyframe+empty_moov",
             str(run / "rviz.mp4")]))
        print(f"[nav] screen capture {w}x{h} -> {run / 'rviz.mp4'}")

    actions.append(_pidded_process(
        run / "logger.pid",
        ["python3", str(PKG_SRC / "scripts/mission_logger.py"),
         "--out", str(run / "mission_log.csv")]))
    print(f"[nav] telemetry -> {run / 'mission_log.csv'}")

    # The costmap / EKF / SLAM-and-graph videos: drawn from this stack's own
    # topics, paced on sim time so they share the Isaac captures' timeline.
    actions.append(_pidded_process(
        run / "viz.pid",
        ["python3", str(PKG_SRC / "scripts/viz_recorder.py"),
         "--out", str(run)]))
    print(f"[nav] costmap / ekf / slam_graph videos -> {run}")
    return actions


def generate_launch_description():
    share = get_package_share_directory("sih_isaac")
    urdf = (pathlib.Path(share) / "sih_rover.urdf").read_text()
    params = str(pathlib.Path(share) / "config/nav2_params.yaml")
    map_yaml = str(PKG_SRC / "generated/map.yaml")

    use_rviz = LaunchConfiguration("rviz")
    use_slam = LaunchConfiguration("slam")

    nodes = [
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("slam", default_value="false"),
        DeclareLaunchArgument("record", default_value="auto"),

        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": urdf, "use_sim_time": True}]),

        # mast-top lidar frame (the only URDF-external sensor frame)
        Node(package="tf2_ros", executable="static_transform_publisher",
             arguments=["--x", "0", "--y", "0", "--z", "0.45",
                        "--frame-id", "base_link", "--child-frame-id", "lidar_link"],
             parameters=[{"use_sim_time": True}]),

        Node(package="robot_localization", executable="ekf_node",
             name="ekf_filter_node",
             parameters=[str(pathlib.Path(share) / "config/ekf.yaml")],
             remappings=[("odometry/filtered", "/odometry/filtered")]),

        # map->odom, exactly one owner. Default: slam_toolbox (SYNC node --
        # the async variant has a nondeterministic activation race under sim
        # time where its scan pipeline never starts). With slam:=false, a
        # single static identity holds the edge instead, which is valid
        # because the EKF's odom frame is seeded at the surveyed deployment
        # pose (dead reckoning only -- drifts on long missions).
        Node(package="tf2_ros", executable="static_transform_publisher",
             arguments=["--frame-id", "map", "--child-frame-id", "odom"],
             parameters=[{"use_sim_time": True}],
             condition=UnlessCondition(use_slam)),
        Node(package="slam_toolbox", executable="sync_slam_toolbox_node",
             name="slam_toolbox",
             parameters=[str(pathlib.Path(share) / "config/slam_toolbox.yaml")],
             condition=IfCondition(use_slam)),
        # slam_toolbox is a lifecycle node in Jazzy: bring it up once it exists
        ExecuteProcess(cmd=["bash", "-c",
                            "sleep 8 && ros2 lifecycle set /slam_toolbox configure"
                            " && sleep 2 && ros2 lifecycle set /slam_toolbox activate"],
                       output="screen",
                       condition=IfCondition(use_slam)),

        Node(package="nav2_map_server", executable="map_server", name="map_server",
             parameters=[params, {"yaml_filename": map_yaml}]),
        Node(package="nav2_planner", executable="planner_server",
             parameters=[params]),
        Node(package="nav2_controller", executable="controller_server",
             parameters=[params]),
        Node(package="nav2_behaviors", executable="behavior_server",
             parameters=[params]),
        Node(package="nav2_bt_navigator", executable="bt_navigator",
             parameters=[params]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_navigation",
             parameters=[{"use_sim_time": True, "autostart": True,
                          "node_names": ["map_server", "planner_server",
                                         "controller_server", "behavior_server",
                                         "bt_navigator"]}]),

        Node(package="rviz2", executable="rviz2",
             arguments=["-d", str(pathlib.Path(share) / "config/sih.rviz")],
             parameters=[{"use_sim_time": True}],
             condition=IfCondition(use_rviz)),
        # mission recording: attaches to the run run_isaac.sh created
        OpaqueFunction(function=_recording_actions),
    ]
    return LaunchDescription(nodes)
