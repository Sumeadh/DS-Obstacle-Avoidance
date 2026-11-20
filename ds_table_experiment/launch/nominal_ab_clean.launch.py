import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    OpaqueFunction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ur_moveit_config.launch_common import load_yaml
import yaml


def generate_launch_description():
    gazebo_ros = get_package_share_directory("gazebo_ros")
    experiment_share = get_package_share_directory("ds_table_experiment")
    world = os.path.join(experiment_share, "worlds", "table.world")
    simulation_share = get_package_share_directory("ur_simulation_gazebo")

    controllers_file = os.path.join(
        simulation_share, "config", "ur_controllers.yaml"
    )
    controller_config = load_yaml(
        "ds_table_experiment", "config/controller_params.yaml"
    )
    controller_parameters = controller_config["controller"]["ros__parameters"]
    initial_positions = controller_config["initial_positions"]
    initial_positions_fd, initial_positions_file = tempfile.mkstemp(
        prefix="ds_table_initial_positions_", suffix=".yaml"
    )
    with os.fdopen(initial_positions_fd, "w", encoding="utf-8") as positions_file:
        yaml.safe_dump(initial_positions, positions_file, sort_keys=False)

    def remove_generated_positions_file(context):
        if os.path.exists(initial_positions_file):
            os.unlink(initial_positions_file)
        return []

    cleanup_positions_file = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                OpaqueFunction(function=remove_generated_positions_file)
            ]
        )
    )
    robot_description = ParameterValue(
        Command([
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution([
                FindPackageShare("ur_description"), "urdf", "ur.urdf.xacro"
            ]),
            " name:=ur ur_type:=ur5e safety_limits:=true",
            " safety_pos_margin:=0.15 safety_k_position:=20",
            " sim_gazebo:=true simulation_controllers:=",
            controllers_file,
            " initial_positions_file:=",
            initial_positions_file,
        ]),
        value_type=str,
    )
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "gui": LaunchConfiguration("gui"),
            "world": world,
        }.items(),
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
        output="screen",
    )
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "ur", "-topic", "robot_description"],
        output="screen",
    )
    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "120",
        ],
        output="screen",
    )
    velocity_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "forward_velocity_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "120",
        ],
        output="screen",
    )
    trajectory_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "120",
        ],
        output="screen",
    )

    start_joint_state = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_spawner],
        )
    )
    start_trajectory_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_spawner,
            on_exit=[trajectory_spawner],
        ),
        condition=IfCondition(LaunchConfiguration("setup_mode")),
    )
    start_velocity_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_spawner,
            on_exit=[velocity_spawner],
        ),
        condition=UnlessCondition(LaunchConfiguration("setup_mode")),
    )
    moveit_share = get_package_share_directory("ur_moveit_config")
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_share, "launch", "ur_moveit.launch.py")
        ),
        launch_arguments={
            "ur_type": "ur5e",
            "use_sim_time": "true",
            "launch_rviz": LaunchConfiguration("rviz"),
            "launch_servo": "false",
        }.items(),
    )
    servo_robot_description = {
        "robot_description": ParameterValue(
            Command([
                FindExecutable(name="xacro"),
                " ",
                PathJoinSubstitution([
                    FindPackageShare("ur_description"),
                    "urdf",
                    "ur.urdf.xacro",
                ]),
                " robot_ip:=xxx.yyy.zzz.www",
                " joint_limit_params:=",
                PathJoinSubstitution([
                    FindPackageShare("ur_description"),
                    "config",
                    "ur5e",
                    "joint_limits.yaml",
                ]),
                " kinematics_params:=",
                PathJoinSubstitution([
                    FindPackageShare("ur_description"),
                    "config",
                    "ur5e",
                    "default_kinematics.yaml",
                ]),
                " physical_params:=",
                PathJoinSubstitution([
                    FindPackageShare("ur_description"),
                    "config",
                    "ur5e",
                    "physical_parameters.yaml",
                ]),
                " visual_params:=",
                PathJoinSubstitution([
                    FindPackageShare("ur_description"),
                    "config",
                    "ur5e",
                    "visual_parameters.yaml",
                ]),
                " safety_limits:=true safety_pos_margin:=0.15",
                " safety_k_position:=20 name:=ur ur_type:=ur5e",
                " script_filename:=ros_control.urscript",
                " input_recipe_filename:=rtde_input_recipe.txt",
                " output_recipe_filename:=rtde_output_recipe.txt prefix:=\"\"",
            ]),
            value_type=str,
        )
    }
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            Command([
                FindExecutable(name="xacro"),
                " ",
                PathJoinSubstitution([
                    FindPackageShare("ur_moveit_config"),
                    "srdf",
                    "ur.srdf.xacro",
                ]),
                " name:=ur prefix:=\"\"",
            ]),
            value_type=str,
        )
    }
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        parameters=[
            {"moveit_servo": load_yaml(
                "ds_table_experiment", "config/servo_velocity.yaml"
            )},
            servo_robot_description,
            robot_description_semantic,
            PathJoinSubstitution([
                FindPackageShare("ur_moveit_config"),
                "config",
                "kinematics.yaml",
            ]),
            {"use_sim_time": True},
        ],
        output="screen",
    )
    experiment_node = Node(
        package="ds_table_experiment",
        executable="Controller.py",
        name="nominal_ab_controller_clean",
        output="screen",
        parameters=[controller_parameters, {"use_sim_time": True}],
    )
    start_experiment = RegisterEventHandler(
        OnProcessExit(
            target_action=velocity_spawner,
            on_exit=[experiment_node],
        ),
        condition=UnlessCondition(LaunchConfiguration("setup_mode")),
    )
    add_table_node = Node(
        package="ds_table_experiment",
        executable="AddTable.py",
        name="add_experiment_obstacle",
        output="screen",
        parameters=[{
            name: controller_parameters[name]
            for name in ("planning_frame", "obstacle_size", "obstacle_center")
        }],
        condition=IfCondition(LaunchConfiguration("setup_mode")),
    )
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("setup_mode", default_value="false"),
        cleanup_positions_file,
        gazebo,
        robot_state_publisher,
        start_joint_state,
        start_trajectory_controller,
        start_velocity_controller,
        start_experiment,
        add_table_node,
        spawn_robot,
        moveit_launch,
        servo_node,
    ])
