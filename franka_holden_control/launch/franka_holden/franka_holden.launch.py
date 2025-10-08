# SPDX-License-Identifier: Apache-2.0
import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, Shutdown
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.parameter_descriptions import ParameterValue

def launch_setup(context):

    # --- Launch arguments ---
    robot_ip = LaunchConfiguration("robot_ip").perform(context)
    use_fake_hardware = LaunchConfiguration("use_fake_hardware").perform(context)
    namespace = LaunchConfiguration("namespace").perform(context)

    # --- Percorsi principali ---
    pkg_name = "franka_holden_control"
    pkg_franka = FindPackageShare(pkg_name).perform(context)
    joint_limits_file = os.path.join(pkg_franka, "config", "joint_limits.yaml")
    kinematics_file = os.path.join(pkg_franka, "config", "kinematics.yaml")
    controllers_file = os.path.join(pkg_franka, "config", "fr3_controllers.yaml")
    ros2_controllers_file = os.path.join(pkg_franka, "config", "fr3_ros_controllers.yaml")
    rviz_file = os.path.join(pkg_franka, "rviz", "franka_holden.rviz")

    # --- Robot description ---
    srdf_path = PathJoinSubstitution([FindPackageShare(pkg_name), "urdf", "franka_holden.srdf"]).perform(context)
    urdf_path = PathJoinSubstitution([FindPackageShare(pkg_name), "urdf", "franka_holden.urdf.xacro"]).perform(context)
    urdf_args = {"robot_ip": robot_ip, "fake_use_fake_hardware": use_fake_hardware}

    robot_description_content = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name='xacro')]),
                " ",
                urdf_path,
                " ",
                " ".join([f"{k}:={v}" for k, v in urdf_args.items()])
            ]
        ),
        value_type=str
    )

    # --- MoveIt Configuration ---
    moveit_config = (
        MoveItConfigsBuilder("fr3_on_table", package_name=pkg_name)
        .robot_description(file_path=urdf_path, mappings=urdf_args)
        .robot_description_semantic(file_path=srdf_path)
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
            publish_planning_scene=True,
            publish_geometry_updates=True,
            publish_state_updates=True,
            publish_transforms_updates=True)
        .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl"])
        .joint_limits(file_path=joint_limits_file)
        .trajectory_execution(file_path=controllers_file)
        .to_moveit_configs()
    )

    moveit_params = moveit_config.to_dict()
    with open(kinematics_file, 'r') as f:
        kinematics_yaml = yaml.safe_load(f)

    moveit_params["robot_description_kinematics"] = kinematics_yaml

    # --- Nodes ---
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        namespace=namespace,
        output="screen",
        parameters=[moveit_params],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_file],
        parameters=[
            {"robot_description": robot_description_content},
            {"robot_description_kinematics": kinematics_yaml}
        ],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=namespace,
        parameters=[
            {"robot_description": robot_description_content},  # il contenuto del URDF
            ros2_controllers_file  # il file YAML dei controller
        ],
        output={"stdout": "screen", "stderr": "screen"},
        on_exit=Shutdown(),
    )

    fake_gripper_controller = Node(
            package="controller_manager",
            executable="spawner",
            namespace=namespace,
            arguments=["fr3_gripper", 
                    "--controller-manager", "/controller_manager"],
            output='screen',
            condition=IfCondition(use_fake_hardware)
        )

    fr3_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=["fr3_arm_controller", 
                "--controller-manager", "/controller_manager"],
        output='screen',
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=["joint_state_broadcaster",
                "--controller-manager","/controller_manager"],
        output='screen',
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=namespace,
        output="screen",
        parameters=[{"robot_description": robot_description_content}],
    )


    franka_robot_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        namespace=namespace,
        arguments=['franka_robot_state_broadcaster'],
        output='screen',
        condition=UnlessCondition(use_fake_hardware),
    )

    # Gripper launch
    pkg_franka_gripper = FindPackageShare("franka_gripper").perform(context)
    gripper_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_franka_gripper, 'launch', 'gripper.launch.py')
        ),
        launch_arguments={
            'robot_ip': robot_ip,
            'use_fake_hardware': use_fake_hardware,
            'namespace': namespace
        }.items()
    )

    return [
        move_group_node,
        rviz_node,
        ros2_control_node,
        franka_robot_state_broadcaster,
        gripper_launch_file,
        robot_state_publisher_node, 
        fr3_arm_controller,
        fake_gripper_controller,
        joint_state_broadcaster_spawner
    ]


def generate_launch_description():

    declare_robot_ip_arg = DeclareLaunchArgument(
        "robot_ip", default_value="", description="IP address of the robot (empty for fake hardware)."
    )
    declare_use_fake_hardware_arg = DeclareLaunchArgument(
        "use_fake_hardware", default_value="true", description="Whether to use fake hardware."
    )
    declare_namespace_arg = DeclareLaunchArgument(
        "namespace", default_value="", description="ROS namespace for the robot."
    )

    return LaunchDescription([
        declare_robot_ip_arg,
        declare_use_fake_hardware_arg,
        declare_namespace_arg,
        OpaqueFunction(function=launch_setup),
    ])
