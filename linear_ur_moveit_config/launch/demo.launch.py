from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os
import yaml
from launch_ros.descriptions import ParameterValue


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def generate_launch_description():
    # Path al pacchetto con la descrizione del robot
    robot_description_pkg = FindPackageShare(package='linear_guide').find('linear_guide')
    moveit_config_pkg = FindPackageShare(package='linear_ur_moveit_config').find('linear_ur_moveit_config')

    # Carica il modello URDF del robot
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([robot_description_pkg, 'urdf', 'linear_ur.urdf.xacro'])
    ])

    # Carica il modello URDF del robot
    robot_semantic_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([moveit_config_pkg, 'srdf', 'linear_ur.srdf.xacro']),' name:=platform'])


    srdf_xacro_file = os.path.join(get_package_share_directory('linear_ur_moveit_config'), 'srdf',
                                     'linear_ur.srdf.xacro')

    robot_description_semantic_config = Command(
        [FindExecutable(name='xacro'), ' ', srdf_xacro_file, ' name:=platform'])



    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(robot_description_semantic_config)
    }


    # Parametri per MoveIt
    robot_description = {'robot_description': robot_description_content}
    kinematics_yaml = PathJoinSubstitution([moveit_config_pkg, 'config', 'kinematics.yaml'])
    ompl_planning_yaml = PathJoinSubstitution([moveit_config_pkg, 'config', 'ompl_planning.yaml'])

    # # Lancia Gazebo con il robot
    # gazebo_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         FindPackageShare('gazebo_ros'), '/launch', '/gazebo.launch.py'
    #     ])
    # )

    # Nodo per la descrizione del robot
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    planning_yaml = load_yaml(
        "linear_ur_moveit_config", "config/ompl_planning.yaml"
    )
    ompl_planning = {"ompl": planning_yaml}

    moveit_cpp_yaml = load_yaml(
        "linear_ur_moveit_config", "config/moveit_cpp.yaml"
    )


    # Nodo per il planning MoveIt
    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[robot_description, 
                    robot_description_semantic,
                    {'robot_description_kinematics': kinematics_yaml},
                    ompl_planning,
                    moveit_cpp_yaml
                    ]
    )

    # RViz per visualizzare MoveIt
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([moveit_config_pkg, 'config', 'moveit.rviz'])]
    )

    return LaunchDescription([
        # gazebo_launch,
        robot_state_publisher,
        move_group,
        rviz_node,
    ])
