from launch.launch_description import LaunchDescription
from launch.actions import OpaqueFunction, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder

def launch_setup(context):

  rviz_file = PathJoinSubstitution([FindPackageShare("ur_linear_guide"), "rviz", "ur_on_linear_guide.rviz"]).perform(context)

  rviz_node = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    arguments=['--display-config', rviz_file]
    )

  robot_description_path = PathJoinSubstitution([FindPackageShare("ur_linear_guide"), "urdf", "linear_ur.urdf.xacro"]).perform(context)
  robot_description_args = {
    "fake_guide" : LaunchConfiguration("fake_guide"),
    "fake_ur" : LaunchConfiguration("fake_ur"),
  }

  srdf_path = PathJoinSubstitution([FindPackageShare("ur_on_linear_guide_moveit_config"), "config", "ur_on_linear_guide.srdf"]).perform(context)
  joint_limits_path = PathJoinSubstitution([FindPackageShare("ur_on_linear_guide_moveit_config"), "config", "joint_limits.yaml"]).perform(context)
  moveit_controllers_path = PathJoinSubstitution([FindPackageShare("ur_on_linear_guide_moveit_config"), "config", "moveit_controllers.yaml"]).perform(context)

  moveit_config = (
    MoveItConfigsBuilder("linear_guide", package_name="ur_on_linear_guide_moveit_config")
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
    move_group_node,
  ]

def generate_launch_description():
  launch_args = [
    DeclareLaunchArgument(name="fake_guide", default_value="true", description="use fake hardware for the linear guide"),
    DeclareLaunchArgument(name="fake_ur", default_value="true", description="use fake hardware for the robot"),
  ]
  return LaunchDescription(launch_args + [OpaqueFunction(function=launch_setup)])