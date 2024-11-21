from launch.launch_description import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable, LaunchConfiguration, TextSubstitution

from launch.event_handlers import OnExecutionComplete

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
  robot_description_content = Command(
      [
          PathJoinSubstitution([FindExecutable(name='xacro')]),
          " ",
          PathJoinSubstitution([FindPackageShare("ur_linear_guide"), "urdf", 'linear_guide.urdf.xacro']),
          " fake:='", LaunchConfiguration("fake").perform(context),"'"
      ]
  )
  robot_description = {'robot_description': robot_description_content}

  controllers_config = PathJoinSubstitution([FindPackageShare("ur_linear_guide"),
   "config", "linear_guide_controllers.yaml"])

  controller_manager_node = Node(
    package="controller_manager",
    executable="ros2_control_node",
    parameters=[controllers_config,robot_description],
    output="screen",
#    remappings=[("/controller_manager/robot_description","/robot_description")],
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

  robot_state_publisher_node = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    output="screen",
    parameters=[robot_description]
  )

  what_to_launch = [
    controller_manager_node,
    joint_state_broadcaster_spawner,
    linear_guide_controller_spawner,
    robot_state_publisher_node,
    ]

  return what_to_launch

def generate_launch_description():
  launch_args = []
  launch_args.append(DeclareLaunchArgument(name="fake", default_value="true", description="use fake hardware"))

  ld = LaunchDescription(launch_args+[OpaqueFunction(function=launch_setup)])
    
  return ld

  