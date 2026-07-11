import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bridge_node = Node(
        package='bridge',
        executable='bridge_node',
        name='bridge_node',
        output='screen',
        parameters=[
            {'port': '/dev/ttyACM0'},
            {'baudrate': 115200}
        ]
    )

    laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_laser_tf',
        output='screen',
        arguments=[
            '--x', '0.065',
            '--y', '0.0',
            '--z', '0.09',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'laser'
        ]
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('sllidar_ros2'),
                'launch',
                'sllidar_c1_launch.py'
            )
        )
    )

    return LaunchDescription([
        bridge_node,
        laser_tf,
        lidar_launch
    ])
