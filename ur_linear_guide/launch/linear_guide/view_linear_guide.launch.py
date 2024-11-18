from launch import LaunchDescription, LaunchContext, LaunchService
from launch.actions import OpaqueFunction, DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition, UnlessCondition
import yaml
import os

def total_launcher(context: LaunchContext, *args, **kwargs):
    ns_sub = context.perform_substitution(LaunchConfiguration('ns'))
    ns_str = "{}".format(ns_sub)

    rviz_file = os.path.join(get_package_share_directory('ur_linear_guide'), 'rviz',
                             "linear_guide.rviz")

    rviz_node = Node(package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['--display-config', rviz_file],
        # namespace=ns_str,
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    node = namespace_nodes(ns_str)

    return node + [rviz_node]


def namespace_nodes(ns_str):
    joint_state_publisher_gui = LaunchConfiguration('js_publisher_gui', default=True)

    joint_state_publisher_node = Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            # namespace=ns_str,
            condition=IfCondition(joint_state_publisher_gui)
        )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution([FindPackageShare('ur_linear_guide'), "urdf", 'linear_guide.urdf.xacro']),
            
        ]
    )

    state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}],
        # namespace=ns_str,
    )
    return [state_publisher_node, joint_state_publisher_node]

def generate_launch_description():

    ns_launch_arg = DeclareLaunchArgument(
        'ns',
        default_value='linear'
    )

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz',
        default_value='True'
    )

    nodes_to_start = [
        ns_launch_arg,
        rviz_launch_arg,
        OpaqueFunction(function=total_launcher, args=[]),
        ]
    
    return LaunchDescription(nodes_to_start)