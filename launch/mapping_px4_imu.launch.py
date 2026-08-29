import os.path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    package_path = get_package_share_directory('fast_lio')
    config_path = LaunchConfiguration('config_path')
    mapping_config = LaunchConfiguration('mapping_config')
    bridge_config = LaunchConfiguration('bridge_config')
    rviz_use = LaunchConfiguration('rviz')

    return LaunchDescription([
        DeclareLaunchArgument('config_path', default_value=os.path.join(package_path, 'config')),
        DeclareLaunchArgument('mapping_config', default_value='mid360_px4_imu.yaml'),
        DeclareLaunchArgument('bridge_config', default_value='px4_imu_bridge.yaml'),
        DeclareLaunchArgument('rviz', default_value='true'),
        Node(
            package='fast_lio',
            executable='px4_imu_bridge',
            parameters=[PathJoinSubstitution([config_path, bridge_config])],
            output='screen'),
        Node(
            package='fast_lio',
            executable='fastlio_mapping',
            parameters=[PathJoinSubstitution([config_path, mapping_config])],
            output='screen'),
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     arguments=['-d', os.path.join(package_path, 'rviz', 'fastlio.rviz')],
        #     condition=IfCondition(rviz_use)),
    ])