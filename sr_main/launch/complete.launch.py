import os
from os import pathsep
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable 
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PythonExpression

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    sr_description = get_package_share_directory("sr_description")
    sr_gazebo = get_package_share_directory("sr_gazebo")
    ros2_gz_bridge_config = os.path.join(
        get_package_share_directory('sr_main'),
        'config',
        'complete_bridge.yaml'
    )

    model_arg1 = DeclareLaunchArgument(
        name="model1",
        default_value=os.path.join(sr_description, "urdf", "lifter_bot.urdf.xacro"),
        description="Path to lifter robot urdf"
    )

    model_arg2 = DeclareLaunchArgument(
        name="model2",
        default_value=os.path.join(sr_description, "urdf", "runner_bot.urdf.xacro"),
        description="Path to runner robot urdf"
    )

    world_arg = DeclareLaunchArgument(
        name="world",
        default_value=os.path.join(sr_gazebo, "worlds", "map2.world"),
        description="Full path to world file"
    )

    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=(
            str(Path(sr_description).parent.resolve())
            + pathsep +
            os.path.join(sr_gazebo, "model")
        )
    )

    # LIFTER RSPs
    lifter1_rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="lifter1",
        name="robot_state_publisher",
        parameters=[{
            "robot_description": ParameterValue(Command(["xacro ", LaunchConfiguration("model1"), " robot_ns:=lifter1"]), value_type=str),
            "use_sim_time": True
        }]
    )

    lifter2_rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="lifter2",
        name="robot_state_publisher",
        parameters=[{
            "robot_description": ParameterValue(Command(["xacro ", LaunchConfiguration("model1"), " robot_ns:=lifter2"]), value_type=str),
            "use_sim_time": True
        }]
    )

    lifter3_rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="lifter3",
        name="robot_state_publisher",
        parameters=[{
            "robot_description": ParameterValue(Command(["xacro ", LaunchConfiguration("model1"), " robot_ns:=lifter3"]), value_type=str),
            "use_sim_time": True
        }]
    )

    lifter4_rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="lifter4",
        name="robot_state_publisher",
        parameters=[{
            "robot_description": ParameterValue(Command(["xacro ", LaunchConfiguration("model1"), " robot_ns:=lifter4"]), value_type=str),
            "use_sim_time": True
        }]
    )

    # RUNNER RSPs
    runner1_rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="runner1",
        name="robot_state_publisher",
        parameters=[{
            "robot_description": ParameterValue(Command(["xacro ", LaunchConfiguration("model2"), " robot_ns:=runner1"]), value_type=str),
            "use_sim_time": True
        }]
    )

    runner2_rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="runner2",
        name="robot_state_publisher",
        parameters=[{
            "robot_description": ParameterValue(Command(["xacro ", LaunchConfiguration("model2"), " robot_ns:=runner2"]), value_type=str),
            "use_sim_time": True
        }]
    )

    runner3_rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="runner3",
        name="robot_state_publisher",
        parameters=[{
            "robot_description": ParameterValue(Command(["xacro ", LaunchConfiguration("model2"), " robot_ns:=runner3"]), value_type=str),
            "use_sim_time": True
        }]
    )

    runner4_rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="runner4",
        name="robot_state_publisher",
        parameters=[{
            "robot_description": ParameterValue(Command(["xacro ", LaunchConfiguration("model2"), " robot_ns:=runner4"]), value_type=str),
            "use_sim_time": True
        }]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": PythonExpression(["'", LaunchConfiguration("world"), " -v 4 -r --physics-engine ", "'"])
        }.items()
    )

    # SPAWN LIFTERS
    spawn_lifter1 = Node(
        package="ros_gz_sim", executable="create", output="screen",
        arguments=["-topic", "/lifter1/robot_description", "-name", "lifter1", "-x", "-4.5", "-y", "4.0", "-z", "0.05"],
    )

    spawn_lifter2 = Node(
        package="ros_gz_sim", executable="create", output="screen",
        arguments=["-topic", "/lifter2/robot_description", "-name", "lifter2", "-x", "-3.5", "-y", "4.0", "-z", "0.05"],
    )

    spawn_lifter3 = Node(
        package="ros_gz_sim", executable="create", output="screen",
        arguments=["-topic", "/lifter3/robot_description", "-name", "lifter3", "-x", "-4.5", "-y", "3.0", "-z", "0.05"],
    )

    spawn_lifter4 = Node(
        package="ros_gz_sim", executable="create", output="screen",
        arguments=["-topic", "/lifter4/robot_description", "-name", "lifter4", "-x", "-3.5", "-y", "3.0", "-z", "0.05"],
    )

    # SPAWN RUNNERS
    spawn_runner1 = Node(
        package="ros_gz_sim", executable="create", output="screen",
        arguments=["-topic", "/runner1/robot_description", "-name", "runner1", "-x", "-4.5", "-y", "-4.0", "-z", "0.05"],
    )

    spawn_runner2 = Node(
        package="ros_gz_sim", executable="create", output="screen",
        arguments=["-topic", "/runner2/robot_description", "-name", "runner2", "-x", "-3.5", "-y", "-4.0", "-z", "0.05"],
    )

    spawn_runner3 = Node(
        package="ros_gz_sim", executable="create", output="screen",
        arguments=["-topic", "/runner3/robot_description", "-name", "runner3", "-x", "-4.5", "-y", "-3.0", "-z", "0.05"],
    )

    spawn_runner4 = Node(
        package="ros_gz_sim", executable="create", output="screen",
        arguments=["-topic", "/runner4/robot_description", "-name", "runner4", "-x", "-3.5", "-y", "-3.0", "-z", "0.05"],
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"use_sim_time": True}],
        arguments=['--ros-args', '-p', f'config_file:={ros2_gz_bridge_config}'],
        output="screen",
    )

    return LaunchDescription([
        model_arg1, model_arg2, world_arg, gazebo_resource_path,
        lifter1_rsp, lifter2_rsp, lifter3_rsp, lifter4_rsp,
        runner1_rsp, runner2_rsp, runner3_rsp, runner4_rsp,
        gazebo,
        spawn_lifter1, spawn_lifter2, spawn_lifter3, spawn_lifter4,
        spawn_runner1, spawn_runner2, spawn_runner3, spawn_runner4,
        gz_ros2_bridge,
    ])