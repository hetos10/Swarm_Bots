import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    sr_description = get_package_share_directory("sr_description")
    sr_gazebo = get_package_share_directory("sr_gazebo")
    sr_main= get_package_share_directory("sr_main")
    # Path to your URDF/Xacro files
    lifter_xacro = os.path.join(sr_description, "urdf", "lifter_bot.urdf.xacro")
    runner_xacro = os.path.join(sr_description, "urdf", "runner_bot.urdf.xacro")
    
    # Your provided robot configurations
    robot_configs = [
        {'name': 'lifter1', 'x': '-4.5', 'y': '4.0', 'file': lifter_xacro},
        {'name': 'lifter2', 'x': '-3.5', 'y': '4.0', 'file': lifter_xacro},
        {'name': 'lifter3', 'x': '-4.5', 'y': '3.0', 'file': lifter_xacro},
        {'name': 'lifter4', 'x': '-3.5', 'y': '3.0', 'file': lifter_xacro},
        {'name': 'runner1', 'x': '-4.5', 'y': '-4.0', 'file': runner_xacro},
        {'name': 'runner2', 'x': '-3.5', 'y': '-4.0', 'file': runner_xacro},
        {'name': 'runner3', 'x': '-4.5', 'y': '-3.0', 'file': runner_xacro},
        {'name': 'runner4', 'x': '-3.5', 'y': '-3.0', 'file': runner_xacro},
    ]

    ld = LaunchDescription()

    # 1. Setup Environment and Gazebo World (Same as your file)
    world_path = os.path.join(sr_gazebo, "worlds", "map2.world") # Adjust name if needed
    
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[os.path.join(sr_description, 'urdf')]
    )
    ld.add_action(gazebo_resource_path)

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
        )]),
        launch_arguments={'gz_args': f'-v 4 -r {world_path}'}.items()
    )
    ld.add_action(gazebo)

    # 2. Loop to generate nodes for all 8 robots
    for bot in robot_configs:
        # Robot State Publisher (per namespace)
        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=bot['name'],
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'robot_description': Command(['xacro ', bot['file'], ' robot_ns:=', bot['name']])
            }]
        )
        
        # Spawn Entity in Gazebo
        spawn_robot = Node(
            package='ros_gz_sim',
            executable='create',
            namespace=bot['name'],
            output='screen',
            arguments=[
                '-topic', f'/{bot["name"]}/robot_description',
                '-name', bot['name'],
                '-x', bot['x'],
                '-y', bot['y'],
                '-z', '0.08' # Spawning slightly higher prevents "floor-snapping" bugs
            ],
        )

        ld.add_action(robot_state_publisher)
        ld.add_action(spawn_robot)

    # 3. Gz-ROS Bridge (Using your config path)
    ros2_gz_bridge_config = os.path.join(sr_main, 'config', 'complete_bridge.yaml')
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"use_sim_time": True}],
        arguments=['--ros-args', '-p', f'config_file:={ros2_gz_bridge_config}'],
        output="screen",
    )
    ld.add_action(gz_ros2_bridge)

    return ld