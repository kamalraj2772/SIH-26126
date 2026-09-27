"""SIH26126 navigation stack (runs beside the Isaac Sim process).

    ros2 launch sih_isaac nav.launch.py            # stack + RViz
    ros2 launch sih_isaac nav.launch.py rviz:=false

TF ownership (exactly one publisher per edge):
    map -> odom            slam_toolbox (lidar SLAM, GPS-free)
    odom -> base_link      robot_localization EKF (wheel odom + IMU)
    base_link -> sensors   robot_state_publisher (URDF) + one static TF for
                           lidar_link (the mast-top RTX lidar)
Isaac Sim publishes no TF. No GNSS anywhere in this file or below it.
"""
import pathlib

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG_SRC = pathlib.Path("/home/qbotix-rover/sih_ws/src/sih_isaac")


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
    ]
    return LaunchDescription(nodes)
