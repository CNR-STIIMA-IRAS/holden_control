from launch.launch_description import LaunchDescription
from launch.actions import OpaqueFunction, IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
  launch_args = [
    DeclareLaunchArgument(name="fake_ur", default_value="true", description="use fake ur"),
    DeclareLaunchArgument(name="fake_guide", default_value="true", description="use fake guide"),
    DeclareLaunchArgument(name="fake_robotiq", default_value="true", description="use fake robotiq"),
    DeclareLaunchArgument(name="robot_ip", default_value="192.168.1.102", description="robot ip address"),
  ]
  return LaunchDescription(launch_args + [OpaqueFunction(function=launch_setup)])

def launch_setup(context):
  launch_moveit_path = PathJoinSubstitution([FindPackageShare('ur_linear_guide'), 'launch', 'ur_on_linear_guide', 'ur_on_linear_guide_moveit.launch.py'])
  launch_moveit_and_robot_description = IncludeLaunchDescription(
    launch_description_source=PythonLaunchDescriptionSource(launch_moveit_path),
    launch_arguments=[('fake_ur', LaunchConfiguration("fake_ur")),
    ('fake_guide', LaunchConfiguration("fake_guide")),
    ('fake_robotiq', LaunchConfiguration("fake_robotiq"))]
  )

  launch_controllers_path = PathJoinSubstitution([FindPackageShare('ur_linear_guide'), 'launch', 'ur_on_linear_guide', 'ur_on_linear_guide_control.launch.py'])
  launch_controllers = IncludeLaunchDescription(
    launch_description_source=PythonLaunchDescriptionSource(launch_controllers_path),
    launch_arguments=[('fake_ur', LaunchConfiguration("fake_ur")),
    ('fake_guide', LaunchConfiguration("fake_guide")),
    ('robot_ip', LaunchConfiguration("robot_ip")),
    ('fake_robotiq', LaunchConfiguration("fake_robotiq"))]
  )

  return [
    launch_moveit_and_robot_description,
    launch_controllers
  ]