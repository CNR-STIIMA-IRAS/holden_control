from launch import LaunchDescription, LaunchContext
from launch.actions import OpaqueFunction, DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def total_launcher(context: LaunchContext, *args, **kwargs):
    ns_sub = context.perform_substitution(LaunchConfiguration('ns'))
    ns_str = "{}".format(ns_sub)

    rviz_file = os.path.join(
        get_package_share_directory('franka_holden_control'),
        'rviz',
        "franka_holden.rviz"
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['--display-config', rviz_file],
    )

    node = namespace_nodes(ns_str)

    return node + [rviz_node]


def namespace_nodes(ns_str):
    joint_state_publisher_gui = LaunchConfiguration('js_publisher_gui', default=True)

    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(joint_state_publisher_gui)
    )

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('franka_holden_control'),
            "urdf",
            'franka_holden.urdf.xacro'
        ]),
    ])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(robot_description_content, value_type=str)
        }],
    )

    return [robot_state_publisher_node, joint_state_publisher_node]


def generate_launch_description():
    ns_launch_arg = DeclareLaunchArgument(
        'ns',
        default_value='franka_holden'
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
