from launch import LaunchDescription
from launch_ros.actions import Node


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

    return LaunchDescription([
        bridge_node,
        laser_tf
    ])
