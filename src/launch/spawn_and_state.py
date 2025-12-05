import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

# Define the absolute path to your URDF file.
URDF_PATH = os.path.expanduser('~/multi_robot_ws/src/simple_robot_with_control.urdf')

def generate_launch_description():
    # 1. Define namespace argument
    robot_namespace = LaunchConfiguration('robot_namespace')
    robot_namespace_arg = DeclareLaunchArgument(
        'robot_namespace',
        default_value='robot1',
        description='Namespace for the robot'
    )
    
    # 2. Use the Command substitution to run 'cat' and safely read the XML.
    # This is the standard, reliable method in ROS 2 launch files.
    robot_description_content = Command(['cat', URDF_PATH])

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
            '-topic', 'robot_description', # Read robot description from parameter
            '-entity', robot_namespace,
            '-robot_namespace', robot_namespace,
            '-x', '0.0', '-y', '0.0', '-z', '0.2'
        ],
        output='screen'
    )

    return LaunchDescription([
        robot_namespace_arg,
        robot_state_publisher_node,
        spawn_entity_node,
    ])
