# SPDX-License-Identifier: Apache-2.0
import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, Shutdown
from launch.conditions import UnlessCondition, IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def generate_launch_description():
    # --- Launch Arguments ---
    robot_ip = LaunchConfiguration('robot_ip')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    namespace = LaunchConfiguration('namespace')

    declare_robot_ip_arg = DeclareLaunchArgument(
        'robot_ip', default_value='', description='IP address of the robot (empty for fake hardware).'
    )
    declare_use_fake_hardware_arg = DeclareLaunchArgument(
        'use_fake_hardware', default_value='true', description='Whether to use fake hardware.'
    )
    declare_namespace_arg = DeclareLaunchArgument(
        'namespace', default_value='', description='ROS namespace for the robot.'
    )
    declare_fake_sensor_commands_arg = DeclareLaunchArgument(
        'fake_sensor_commands', default_value='false',
        description='If true, fakes sensor commands (only valid with fake hardware).'
    )
    declare_db_flag_arg = DeclareLaunchArgument(
        'db', default_value='false', description='Database flag (optional).'
    )

    # --- Robot Description ---
    franka_xacro_file = os.path.join(
        get_package_share_directory('franka_holden_control'),
        'urdf', 'franka_holden.urdf.xacro'
    )

    robot_description_config = Command([
        FindExecutable(name='xacro'),
        ' ',
        franka_xacro_file,
        ' robot_ip:=', robot_ip,
        ' use_fake_hardware:=', use_fake_hardware
    ])
    robot_description = {'robot_description': ParameterValue(robot_description_config, value_type=str)}

    # --- SRDF come testo ---
    franka_semantic_file = os.path.join(
        get_package_share_directory('franka_holden_control'),
        'urdf', 'franka_holden.srdf'
    )
    with open(franka_semantic_file, 'r') as f:
        srdf_text = f.read()
    robot_description_semantic = {'robot_description_semantic': ParameterValue(srdf_text, value_type=str)}

    # --- Yaml Configs ---
    kinematics_yaml = load_yaml('franka_holden_control', 'config/kinematics.yaml')
    ompl_planning_yaml = load_yaml('franka_holden_control', 'config/ompl_planning.yaml')
    moveit_controllers_yaml = load_yaml('franka_holden_control', 'config/fr3_controllers.yaml')
    ros2_controllers_path = os.path.join(get_package_share_directory('franka_holden_control'), 'config', 'fr3_ros_controllers.yaml')

    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_pipelines': ['ompl'],
            'default_planning_pipeline': 'ompl',
            'ompl': {
                'planning_plugin': 'ompl_interface/OMPLPlanner',
                'request_adapters': (
                    'default_planner_request_adapters/AddTimeOptimalParameterization '
                    'default_planner_request_adapters/FixWorkspaceBounds '
                    'default_planner_request_adapters/FixStartStateBounds '
                    'default_planner_request_adapters/FixStartStateCollision '
                    'default_planner_request_adapters/FixStartStatePathConstraints'
                ),
                'start_state_max_bounds_error': 0.1,
                'planner_configs': ompl_planning_yaml.get('planner_configs', {})
            }
        }
    }

    moveit_controllers = {
        'moveit_simple_controller_manager': moveit_controllers_yaml,
        'moveit_controller_manager': 'moveit_simple_controller_manager'
                                     '/MoveItSimpleControllerManager',
    }

    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.execution_duration_monitoring': False,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
        'publish_robot_description': True,
        'publish_robot_description_semantic': True,
    }

    # --- Nodes ---
    run_move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        namespace=namespace,
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
        ]
    )

    rviz_config_file = os.path.join(
        get_package_share_directory('franka_holden_control'), 'rviz', 'franka_holden.rviz'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[robot_description, robot_description_semantic, ompl_planning_pipeline_config, kinematics_yaml],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        output='both',
        parameters=[robot_description],
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace=namespace,
        parameters=[robot_description, ros2_controllers_path],
        remappings=[('joint_states', 'franka/joint_states')],
        output={'stdout': 'screen', 'stderr': 'screen'},
        on_exit=Shutdown(),
    )

    # Load controllers
    # load_controllers = []
    # for controller in ['fr3_arm_controller', 'fr3_arm_scaled_controller', 'joint_state_broadcaster']:
    #     load_controllers.append(
    #         ExecuteProcess(
    #             cmd=[
    #                 'ros2', 'run', 'controller_manager', 'spawner', controller,
    #                 '--controller-manager', PathJoinSubstitution([namespace, 'controller_manager']),
    #                 '--controller-manager-timeout', '60'
    #             ],
    #             output='screen'
    #         )
    #     )


    fake_gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["fr3_gripper", 
                "--controller-manager", "/controller_manager"],
        output='screen',
        condition=IfCondition(use_fake_hardware)
    )

    fr3_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["fr3_arm_controller", 
                "--controller-manager", "/controller_manager", "--inactive"],
        output='screen',
    )

    fr3_arm_scaled_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["fr3_arm_scaled_controller", 
                "--controller-manager", "/controller_manager"],
        output='screen',
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                "--controller-manager","/controller_manager"],
        output='screen',
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace=namespace,
        parameters=[{'source_list': ['franka/joint_states', 'fr3_gripper/joint_states'], 'rate': 30}],
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
    gripper_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('franka_gripper'), 'launch', 'gripper.launch.py')
        ),
        launch_arguments={
            'robot_ip': robot_ip,
            'use_fake_hardware': use_fake_hardware,
            'namespace': namespace
        }.items()
    )

    return LaunchDescription([
        declare_robot_ip_arg,
        declare_use_fake_hardware_arg,
        declare_namespace_arg,
        declare_fake_sensor_commands_arg,
        declare_db_flag_arg,
        rviz_node,
        robot_state_publisher,
        run_move_group_node,
        ros2_control_node,
        joint_state_publisher,
        franka_robot_state_broadcaster,
        gripper_launch_file,
        fr3_arm_controller,
        fr3_arm_scaled_controller,
        fake_gripper_controller,
        joint_state_broadcaster_spawner
    ])
