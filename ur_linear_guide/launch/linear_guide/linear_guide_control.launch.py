from launch.launch_description import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction
from launch.substitutions import PathJoinSubstitution

from launch.event_handlers import OnExecutionComplete

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():

  controllers_config = PathJoinSubstitution([FindPackageShare("ur_linear_guide"),
   "config", "linear_guide_controllers.yaml"])

  controller_manager_node = Node(
    package="controller_manager",
    executable="ros2_control_node",
    parameters=[controllers_config],
    output="screen",
    remappings=[("/controller_manager/robot_description","/robot_description")],
  )

  linear_guide_controller_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["linear_guide_controller", 
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

  ld = LaunchDescription()

  ld.add_action(controller_manager_node)
  ld.add_action(joint_state_broadcaster_spawner)
  ld.add_action(linear_guide_controller_spawner)
    
  return ld