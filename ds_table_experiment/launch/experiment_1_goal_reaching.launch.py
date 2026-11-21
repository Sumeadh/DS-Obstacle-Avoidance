from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    main_launch = PathJoinSubstitution([
        FindPackageShare("ds_table_experiment"),
        "launch",
        "nominal_ab_clean.launch.py",
    ])
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(main_launch),
            launch_arguments={
                "experiment": "experiment_1_goal_reaching",
                "auto_change_target": "false",
            }.items(),
        )
    ])
