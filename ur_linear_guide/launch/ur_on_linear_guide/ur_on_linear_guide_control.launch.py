from launch.launch_description import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

from launch.event_handlers import OnExecutionComplete

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

from launch.conditions import IfCondition, UnlessCondition

description_package = "ur_linear_guide"
description_file = "linear_ur.urdf.xacro"

def generate_launch_description():

  launch_args = [
    DeclareLaunchArgument(name="fake", default_value="true", description="use fake hardware for the robot"),
  ]

  launch_args = [
    DeclareLaunchArgument(name="robot_ip", default_value="192.168.1.102", description="robot IP address"),
  ]

  controllers_config = PathJoinSubstitution([FindPackageShare("ur_linear_guide"),
   "config", "ur_on_linear_guide_controllers.yaml"])

  controller_manager_node = Node(
    package="controller_manager",
    executable="ros2_control_node",
    parameters=[controllers_config],
    output="screen",
    remappings=[("/controller_manager/robot_description","/robot_description")],
  )

  ur_on_linear_guide_controller_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["ur_on_linear_guide_controller", 
               "--controller-manager", "/controller_manager"],
    output='screen',
  )

  linear_guide_controller_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["linear_guide_controller", 
               "--controller-manager", "/controller_manager", "--inactive"],
    output='screen',
  )

  ur_controller_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["ur_controller", 
               "--controller-manager", "/controller_manager", "--inactive"],
    output='screen',
  )

  joint_state_broadcaster_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["joint_state_broadcaster",
               "--controller-manager","/controller_manager"],
    output='screen',
  )

  ur_control_node = Node(
    package="ur_robot_driver",
    executable="ur_ros2_control_node",
    output="screen",
    condition=UnlessCondition(LaunchConfiguration("fake")),
  )

  dashboard_client_node = Node(
    package="ur_robot_driver",
    condition=UnlessCondition(LaunchConfiguration("fake")),
    executable="dashboard_client",
    name="dashboard_client",
    output="screen",
    emulate_tty=True,
    parameters=[{"robot_ip": LaunchConfiguration("robot_ip")}],
  )

#   tool_communication_node = Node(
#     package="ur_robot_driver",
#     condition=IfCondition(use_tool_communication),
#     executable="tool_communication.py",
#     name="ur_tool_comm",
#     output="screen",
#     parameters=[
#         {
#             "robot_ip": robot_ip,
#             "tcp_port": tool_tcp_port,
#             "device_name": tool_device_name,
#         }
#     ],
#   )

  # controller_stopper_node = Node(
  #     package="ur_robot_driver",
  #     executable="controller_stopper_node",
  #     name="controller_stopper",
  #     output="screen",
  #     emulate_tty=True,
  #     condition=UnlessCondition(use_fake_hardware),
  #     parameters=[
  #         {"headless_mode": headless_mode},
  #         {"joint_controller_active": activate_joint_controller},
  #         {
  #             "consistent_controllers": [
  #                 "io_and_status_controller",
  #                 "force_torque_sensor_broadcaster",
  #                 "joint_state_broadcaster",
  #                 "speed_scaling_state_broadcaster",
  #             ]
  #         },
  #     ],
  # )

  urscript_interface = Node(
    package="ur_robot_driver",
    executable="urscript_interface",
    parameters=[{"robot_ip": LaunchConfiguration("robot_ip")}],
    output="screen",
    condition=UnlessCondition(LaunchConfiguration("fake")),
  )   

  ld = LaunchDescription(launch_args)

  ld.add_action(controller_manager_node)
  ld.add_action(joint_state_broadcaster_spawner)
  ld.add_action(ur_controller_spawner)
  ld.add_action(linear_guide_controller_spawner)
  ld.add_action(ur_on_linear_guide_controller_spawner)

  return ld