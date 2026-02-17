import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.substitutions import Command, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    sr_gazebo_pkg = get_package_share_directory('sr_gazebo')
    sr_description_pkg = get_package_share_directory('sr_description')
    sr_control_pkg = get_package_share_directory('sr_control')

    world_path = os.path.join(sr_gazebo_pkg, 'worlds', 'map.sdf')
    lifter_xacro = os.path.join(sr_description_pkg, 'urdf', 'lifter_bot.urdf.xacro')
    runner_xacro = os.path.join(sr_description_pkg, 'urdf', 'runner_bot.urdf.xacro')
    
    # 8 Robot Configuration
    robot_configs = [
        {'name': 'lifter1', 'ns': 'lifter1', 'xacro': lifter_xacro, 'x': '-3.5', 'y': '-2.0'},
        {'name': 'lifter2', 'ns': 'lifter2', 'xacro': lifter_xacro, 'x': '-1.5', 'y': '-2.0'},
        {'name': 'lifter3', 'ns': 'lifter3', 'xacro': lifter_xacro, 'x': '-3.5', 'y': '2.0'},
        {'name': 'lifter4', 'ns': 'lifter4', 'xacro': lifter_xacro, 'x': '-1.5', 'y': '2.0'},
        {'name': 'runner1', 'ns': 'runner1', 'xacro': runner_xacro, 'x': '1.5', 'y': '-2.0'},
        {'name': 'runner2', 'ns': 'runner2', 'xacro': runner_xacro, 'x': '3.5', 'y': '-2.0'},
        {'name': 'runner3', 'ns': 'runner3', 'xacro': runner_xacro, 'x': '1.5', 'y': '2.0'},
        {'name': 'runner4', 'ns': 'runner4', 'xacro': runner_xacro, 'x': '3.5', 'y': '2.0'},
    ]

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
        launch_arguments={"gz_args": PythonExpression(["'", world_path, " -v 4 -r'"])}.items()
    )

    nodes_list = []
    
    for i, robot in enumerate(robot_configs):
        ns = robot['ns']
        # Staggered start: Every robot starts 5 seconds after the previous one
        delay = float(i * 5.0) 

        # 1. Robot State Publisher
        rsp = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=ns,
            parameters=[{
                "robot_description": ParameterValue(
                    Command(['xacro ', robot['xacro'], f' namespace:={ns}']),
                    value_type=str
                ),
                "use_sim_time": True
            }]
        )

        # 2. Spawn Robot in Gazebo
        spawn = Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-topic", f"/{ns}/robot_description",
                "-name", robot['name'],
                "-x", robot['x'], "-y", robot['y'], "-z", "0.2"
            ]
        )

        # 3. Controller Spawners (Targeted at the namespaced manager)
        jsb = TimerAction(
            period=delay + 3.0,
            actions=[Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster', '-c', f'/{ns}/controller_manager']
            )]
        )

        mecanum = TimerAction(
            period=delay + 5.0,
            actions=[Node(
                package='controller_manager',
                executable='spawner',
                arguments=['mecanum_controller', '-c', f'/{ns}/controller_manager']
            )]
        )

        nodes_list.extend([rsp, spawn, jsb, mecanum])

    return LaunchDescription([gazebo] + nodes_list)