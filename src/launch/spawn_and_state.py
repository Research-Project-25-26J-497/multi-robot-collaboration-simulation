import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Define the absolute path to your URDF file.
URDF_PATH = os.path.expanduser('~/multi_robot_ws/src/simple_robot_with_control.urdf')

def generate_launch_description():
    # 1. Define namespace and position arguments
    robot_namespace = LaunchConfiguration('robot_namespace')
    x_pos = LaunchConfiguration('x', default='0.0')
    y_pos = LaunchConfiguration('y', default='0.0')
    z_pos = LaunchConfiguration('z', default='0.2')
    
    robot_namespace_arg = DeclareLaunchArgument(
        'robot_namespace',
        default_value='robot1',
        description='Namespace for the robot'
    )
    
    x_arg = DeclareLaunchArgument('x', default_value='0.0', description='X position')
    y_arg = DeclareLaunchArgument('y', default_value='0.0', description='Y position')
    z_arg = DeclareLaunchArgument('z', default_value='0.2', description='Z position')
    
    # 2. Read URDF file content
    with open(URDF_PATH, 'r') as urdf_file:
        robot_description_content = urdf_file.read()

    # 3. Robot State Publisher Node (CRUCIAL for TF frames, which enable the sensor)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=robot_namespace,
        parameters=[
            {'robot_description': robot_description_content},
            {'use_sim_time': True},
        ]
    )

    # 4. Spawn Entity Node (Injects the robot model into Gazebo)
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', robot_namespace,
            '-robot_namespace', robot_namespace,
            '-x', x_pos,
            '-y', y_pos,
            '-z', z_pos
        ],
        output='screen'
    )

    return LaunchDescription([
        robot_namespace_arg,
        x_arg,
        y_arg,
        z_arg,
        robot_state_publisher_node,
        spawn_entity_node,
    ])
