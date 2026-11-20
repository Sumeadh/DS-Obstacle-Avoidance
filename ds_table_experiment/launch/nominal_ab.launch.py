import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    experiment_share = get_package_share_directory("ds_table_experiment")
    velocity_launch = os.path.join(
        experiment_share, "launch", "nominal_ab_clean.launch.py"
    )
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(velocity_launch),
            launch_arguments={
                "gui": LaunchConfiguration("gui"),
                "rviz": LaunchConfiguration("rviz"),
            }.items(),
        ),
    ])
