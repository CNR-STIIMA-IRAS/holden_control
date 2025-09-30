from launch.launch_description import LaunchDescription
from launch.actions import OpaqueFunction, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
  launch_args = [
    DeclareLaunchArgument(name="fake", default_value="true", description="use fake hardware"),
  ]
  return LaunchDescription(launch_args + [OpaqueFunction(function=launch_setup)])

def launch_setup(context):

  rviz_file = PathJoinSubstitution([FindPackageShare("ur_linear_guide"), "rviz", "linear_guide.rviz"]).perform(context)

  rviz_node = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    arguments=['--display-config', rviz_file]
    )

  robot_description_path = PathJoinSubstitution([FindPackageShare("ur_linear_guide"), "urdf", "linear_guide.urdf.xacro"]).perform(context)
  robot_description_args = {
    "fake" : LaunchConfiguration("fake")
  }

  srdf_path = PathJoinSubstitution([FindPackageShare("linear_guide_moveit_config"), "config", "platform.srdf"]).perform(context)
  joint_limits_path = PathJoinSubstitution([FindPackageShare("linear_guide_moveit_config"), "config", "joint_limits.yaml"]).perform(context)
  moveit_controllers_path = PathJoinSubstitution([FindPackageShare("linear_guide_moveit_config"), "config", "moveit_controllers.yaml"]).perform(context)

  moveit_config = (
    MoveItConfigsBuilder("linear_guide", package_name="linear_guide_moveit_config")
    .robot_description(file_path=robot_description_path, mappings=robot_description_args)
    .robot_description_semantic(file_path=srdf_path)
    .planning_scene_monitor(publish_robot_description=True,
                            publish_robot_description_semantic=True,
                            publish_planning_scene=True)
    .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl"])
    .joint_limits(file_path=joint_limits_path)
    .trajectory_execution(file_path=moveit_controllers_path)
    .to_moveit_configs()
  )

  move_group_node = Node(
    package="moveit_ros_move_group",
    executable="move_group",
    output="screen",
    parameters=[moveit_config.to_dict()],
  )

  return [
    rviz_node,
    move_group_node
  ]