from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("slam_toolbox"),
                "launch",
                "localization_launch.py"
            ])
        ]),
        launch_arguments={
            "use_sim_time": "false",
            "slam_params_file": "/ros2_ws/localization_params.yaml"
        }.items()
    )

    return LaunchDescription([
        slam_toolbox_launch
    ])
