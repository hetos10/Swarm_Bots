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
        get_package_share_directory('sr_gazebo'), 
        'config',
        'bridge.yaml'
    )

    # -------------------------------
    # Launch Arguments
    # -------------------------------
    model_arg1 = DeclareLaunchArgument(
        name="model1",
        default_value=os.path.join(
            sr_description, "urdf", "lifter_bot.urdf.xacro"
        ),
        description="Path to lifter robot urdf"
    )

    model_arg2 = DeclareLaunchArgument(
        name="model2",
        default_value=os.path.join(
            sr_description, "urdf", "runner_bot.urdf.xacro"
        ),
        description="Path to runner robot urdf"
    )

    world_arg = DeclareLaunchArgument(
        name="world",
        default_value=os.path.join(
            sr_gazebo, "worlds", "map1.world"
        ),
        description="Full path to world file"
    )

    # -------------------------------
    # Environment Path Fix
    # -------------------------------
    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=(
            str(Path(sr_description).parent.resolve())
            + pathsep +
            os.path.join(sr_gazebo, "model")
        )
    )

    # -------------------------------
    # Robot Descriptions
    # -------------------------------
    lifter_description = ParameterValue(
        Command([
            "xacro ",
            LaunchConfiguration("model1"),
            " is_sim:=True"
        ]),
        value_type=str
    )

    runner_description = ParameterValue(
        Command([
            "xacro ",
            LaunchConfiguration("model2"),
            " is_sim:=True"
        ]),
        value_type=str
    )

    # -------------------------------
    # Robot State Publishers
    # -------------------------------
    lifter_rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="lifter1",
        name="robot_state_publisher",
        parameters=[{
            "robot_description": lifter_description,
            "use_sim_time": True
        }]
    )

    runner_rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="runner1",
        name="robot_state_publisher",
        parameters=[{
            "robot_description": runner_description,
            "use_sim_time": True
        }]
    )

    # -------------------------------
    # Gazebo Launch
    # -------------------------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py"
            )
        ),
        launch_arguments={
            "gz_args": PythonExpression([
                "'",
                LaunchConfiguration("world"),
                " -v 4 -r --physics-engine ",
                "'"
            ])
        }.items()
    )
    # -------------------------------
    # Spawn Robots
    # -------------------------------
    spawn_lifter = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "/lifter1/robot_description",
            "-name", "lifter1",
            "-x", "-4.5",
            "-y", "4.0",
            "-z", "0.05"
        
        ],
    )

    spawn_runner = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "/runner1/robot_description",
            "-name", "runner1",
            "-x", "-4.5",
            "-y", "-4.0",
            "-z", "0.05"
           
        ],
    )

    # -------------------------------
    # Gz-ROS Bridge
    # -------------------------------
    gz_ros2_bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            parameters=[
                {"use_sim_time": True}
            ],
            arguments=[
                '--ros-args', 
                '-p', f'config_file:={ros2_gz_bridge_config}'
            ],
            output="screen",
        )

    return LaunchDescription([
        model_arg1,
        # model_arg2,
        world_arg,
        gazebo_resource_path,
        lifter_rsp,
        # runner_rsp,
        gazebo,
        spawn_lifter,
        # spawn_runner,
        gz_ros2_bridge,
    ])
