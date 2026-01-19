import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('robot_description')

    # Check if we are using the new clean file or the old one
    # Defaulting to the clean one we created: 'snake_robot.urdf.xacro'
    default_model_path = os.path.join(pkg_share, 'urdf', 'snake_robot.urdf.xacro')

    model_arg = DeclareLaunchArgument(name='model', default_value=default_model_path,
                                      description='Absolute path to robot urdf file')

    # Run xacro to convert .xacro to .urdf
    robot_description = Command(['xacro ', LaunchConfiguration('model')])

    return LaunchDescription([
        model_arg,

        # Publish the robot state (tf transforms)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),

        # GUI to move joints manually
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),

        # Launch RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])