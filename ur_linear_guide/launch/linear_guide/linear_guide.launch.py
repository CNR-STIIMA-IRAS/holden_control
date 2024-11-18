from launch.launch_description import LaunchDescription
from launch.actions import OpaqueFunction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
  launch_args = [
    DeclareLaunchArgument(name="fake", default_value="true", description="use fake hardware"),
  ]
  return LaunchDescription(launch_args + [OpaqueFunction(function=launch_setup)])

def launch_setup(context):
  launch_moveit_path = PathJoinSubstitution([FindPackageShare('ur_linear_guide'), 'launch', 'linear_guide', 'linear_guide_moveit.launch.py'])
  launch_moveit_and_robot_description = IncludeLaunchDescription(
    launch_description_source=PythonLaunchDescriptionSource(launch_moveit_path),
    launch_arguments=[('fake', LaunchConfiguration("fake"))]
  )

  launch_controllers_path = PathJoinSubstitution([FindPackageShare('ur_linear_guide'), 'launch', 'linear_guide', 'linear_guide_control.launch.py'])
  launch_controllers = IncludeLaunchDescription(
    launch_description_source=PythonLaunchDescriptionSource(launch_controllers_path),
  )

  return [
    launch_moveit_and_robot_description,
    launch_controllers
  ]