"""Phase 1 bring-up: procedurally generated world + robot spawn + sensor bridge.

Brings up, in order:
  1. The seeded Gazebo Harmonic world (heightmap + trees/rocks), generated on
     the fly if not already on disk for this seed.
  2. robot_state_publisher, fed robot_description from the single xacro
     source in sih_description (owns base_link -> sensors, static).
  3. gz sim server (headless with gui:=false, the default).
  4. The robot, spawned from the /robot_description topic (not a second SDF).
  5. ros_gz_bridge, driven by config/bridge.yaml.

use_sim_time is true everywhere, without exception.
"""
import pathlib

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _generate_world(context, *args, **kwargs):
    from sih_sim import world_generator

    seed = int(LaunchConfiguration("seed").perform(context))
    sim_share = pathlib.Path(get_package_share_directory("sih_sim"))
    heightmap_dir = sim_share / "heightmaps" / "generated"
    world_path = sim_share / "worlds" / "generated" / f"qslam_world_{seed}.sdf"

    features_path = heightmap_dir / f"features_{seed}.json"
    if not features_path.exists() or not world_path.exists():
        import json
        from sih_sim import heightmap_generator
        features = heightmap_generator.generate(seed, heightmap_dir)
        sdf = world_generator.build_world(features, heightmap_dir)
        world_path.parent.mkdir(parents=True, exist_ok=True)
        world_path.write_text(sdf)

    with open(features_path) as f:
        features = __import__("json").load(f)
    spawn = features["spawn"]

    context.launch_configurations["_world_path"] = str(world_path)
    context.launch_configurations["_spawn_x"] = str(spawn["x"])
    context.launch_configurations["_spawn_y"] = str(spawn["y"])
    # base_link rests 0.255 m above ground (wheel radius 0.15 + half chassis
    # height 0.125 - 0.02); add margin so the robot settles onto the terrain
    # instead of spawning embedded in it.
    context.launch_configurations["_spawn_z"] = str(spawn["z"] + 0.35)
    return []


def _launch_setup(context, *args, **kwargs):
    gui = LaunchConfiguration("gui").perform(context).lower() == "true"
    world_path = context.launch_configurations["_world_path"]
    spawn_x = context.launch_configurations["_spawn_x"]
    spawn_y = context.launch_configurations["_spawn_y"]
    spawn_z = context.launch_configurations["_spawn_z"]

    xacro_path = PathJoinSubstitution(
        [get_package_share_directory("sih_description"), "urdf", "sih_rover.xacro"]
    )
    cam_w = LaunchConfiguration("camera_width").perform(context)
    cam_h = LaunchConfiguration("camera_height").perform(context)
    cam_fps = LaunchConfiguration("camera_fps").perform(context)
    robot_description = ParameterValue(
        Command([
            "xacro ", xacro_path,
            " camera_width:=", cam_w,
            " camera_height:=", cam_h,
            " camera_fps:=", cam_fps,
        ]),
        value_type=str,
    )

    gz_args = f"-r {world_path}" if gui else f"-s -r {world_path}"

    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={"gz_args": gz_args}.items(),
    )

    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        # publish_frequency defaults to 20 Hz, which throttles /tf below the
        # 50 Hz joint state rate; match the joint states instead.
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True,
            "publish_frequency": 50.0,
        }],
        output="screen",
    )

    spawn_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-world", "qslam_world",
            "-topic", "robot_description",
            "-name", "sih_rover",
            "-x", spawn_x, "-y", spawn_y, "-z", spawn_z,
        ],
        output="screen",
    )

    bridge_config = PathJoinSubstitution(
        [get_package_share_directory("sih_sim"), "config", "bridge.yaml"]
    )
    bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"config_file": bridge_config, "use_sim_time": True}],
        output="screen",
    )

    return [gz_sim_launch, rsp_node, spawn_node, bridge_node]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("seed", default_value="42"),
        DeclareLaunchArgument("gui", default_value="false"),
        DeclareLaunchArgument("camera_width", default_value="1280"),
        DeclareLaunchArgument("camera_height", default_value="720"),
        DeclareLaunchArgument("camera_fps", default_value="30"),
        OpaqueFunction(function=_generate_world),
        OpaqueFunction(function=_launch_setup),
    ])
