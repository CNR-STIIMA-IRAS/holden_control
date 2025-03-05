from launch.launch_description import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, Command, FindExecutable

from launch.event_handlers import OnExecutionComplete

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

from launch.conditions import IfCondition, UnlessCondition

def launch_setup(context, *args, **kwargs):

  robot_description_content = Command(
      [
          PathJoinSubstitution([FindExecutable(name='xacro')]),
          " ",
          PathJoinSubstitution([FindPackageShare("ur_linear_guide"), "urdf", 'linear_ur.urdf.xacro']),
          " fake_guide:='", LaunchConfiguration("fake_guide").perform(context),"'",
          " fake_ur:='", LaunchConfiguration("fake_ur").perform(context),"'",
          " fake_robotiq:='", LaunchConfiguration("fake_robotiq").perform(context),"'",
      ])

  robot_description = {'robot_description': robot_description_content}

  controllers_config = PathJoinSubstitution([FindPackageShare("ur_linear_guide"),
   "config", "ur_on_linear_guide_controllers.yaml"])

  controller_manager_node = Node(
    package="controller_manager",
    executable="ros2_control_node",
    parameters=[controllers_config,robot_description],
    output="screen",
    # prefix=['gdb -ex start --args'],
    arguments=["--ros-args", "--log-level", "info"],
    # remappings=[("/controller_manager/robot_description","/robot_description")],
  )

  robot_state_publisher_node = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    output="screen",
    parameters=[robot_description]
  )

  ur_on_linear_guide_controller_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["ur_on_linear_guide_controller", 
               "--controller-manager", "/controller_manager", "--inactive"],
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

  ur_on_linear_guide_scaled_controller_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["ur_on_linear_guide_scaled_controller", 
               "--controller-manager", "/controller_manager"],
    output='screen',
  )
#  "--ros-args", "--log-level", "debug"],  
  linear_guide_scaled_controller_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["linear_guide_scaled_controller", 
               "--controller-manager", "/controller_manager", "--inactive"],
    output='screen',
  )

  ur_scaled_controller_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["ur_scaled_controller", 
               "--controller-manager", "/controller_manager", "--inactive"],
    output='screen',
  )

  robotiq_controller_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["robotiq_action_controller", 
               "--controller-manager", "/controller_manager", "--inactive"],
    output='screen',
  )

  robotiq_activation_controller_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["robotiq_activation_controller", 
               "--controller-manager", "/controller_manager", "--inactive"],
    output='screen',
  )

  controller_stopper_node = Node(
      package="ur_robot_driver",
      executable="controller_stopper_node",
      name="controller_stopper",
      output="screen",
      emulate_tty=True,
      condition=UnlessCondition(LaunchConfiguration("fake_ur").perform(context)),
      parameters=[
          {"headless_mode": False},
          {"joint_controller_active": True},
          {
              "consistent_controllers": [
                  # "io_and_status_controller",
                  # "force_torque_sensor_broadcaster",
                  "joint_state_broadcaster",
                  # "speed_scaling_state_broadcaster",
              ]
          },
      ],
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
    condition=UnlessCondition(LaunchConfiguration("fake_ur")),
  )

  dashboard_client_node = Node(
    package="ur_robot_driver",
    condition=UnlessCondition(LaunchConfiguration("fake_ur")),
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

  urscript_interface = Node(
    package="ur_robot_driver",
    executable="urscript_interface",
    parameters=[{"robot_ip": LaunchConfiguration("robot_ip")}],
    output="screen",
    condition=UnlessCondition(LaunchConfiguration("fake_ur")),
  )   

  what_to_launch = [
    controller_manager_node,
    joint_state_broadcaster_spawner,
    robot_state_publisher_node,
    ur_controller_spawner,
    linear_guide_controller_spawner,
    ur_on_linear_guide_controller_spawner,
    ur_on_linear_guide_scaled_controller_spawner,
    linear_guide_scaled_controller_spawner,
    ur_scaled_controller_spawner,
    robotiq_controller_spawner,
    robotiq_activation_controller_spawner,
    # robotiq_forward_controller_spawner,
    #controller_stopper_node
    ur_control_node,
    dashboard_client_node,
    urscript_interface
    ]
  
  return what_to_launch

def generate_launch_description():
  launch_args = []
  launch_args.append(DeclareLaunchArgument(name="fake_robotiq", default_value="true", description="use fake hardware for the robotiq gripper"))
  launch_args.append(DeclareLaunchArgument(name="fake_guide", default_value="true", description="use fake hardware for the linear guide"))
  launch_args.append(DeclareLaunchArgument(name="fake_ur", default_value="true", description="use fake hardware for the ur"))
  launch_args.append(DeclareLaunchArgument(name="robot_ip", default_value="192.168.1.102", description="robot IP address"))

  ld = LaunchDescription(launch_args+[OpaqueFunction(function=launch_setup)])

  return ld