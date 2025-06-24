"""
Robot State Publisher Launch File

This launch file launches the the necessary nodes to connect to the Gazebo
simulation environment.

Author: Hunter Ellis
Date: 3-29-25
"""
import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition

ARGUMENTS = [
    DeclareLaunchArgument('add_world',
                          default_value='true',
                          choices=['true', 'false'],
                          description='Whether to add world link'),
    DeclareLaunchArgument('base_link',
                          default_value='base_link',
                          description='Name of the base link'),
    DeclareLaunchArgument('base_type',
                          default_value='g_shape',
                          description='Type of the base'),
    DeclareLaunchArgument('flange_link',
                          default_value='base_link',
                          description='Name of the flange link'),
    DeclareLaunchArgument('use_camera',
                          default_value='false',
                          choices=['true', 'false'],
                          description='Whether to use the RGBD Gazebo plugin for point cloud'),
    DeclareLaunchArgument('use_gazebo',
                          default_value='false',
                          choices=['true', 'false'],
                          description='Whether to use Gazebo simulation'),
    DeclareLaunchArgument('use_gripper',
                          default_value='false',
                          choices=['true', 'false'],
                          description='Whether to attach a gripper')
]


def generate_launch_description():
    desc_pkg_path = get_package_share_directory('arm_description')

    urdf_path = os.path.join(
        desc_pkg_path, 'model', 'arm.urdf.xacro'
    )

    sim_time = LaunchConfiguration('use_sim_time')
    declare_sim_time = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        choices=['true', 'false'],
        description='Use simulation (Gazebo) clock if true'
    )

    gui_flag = LaunchConfiguration('jsp_gui')
    jsp_gui = DeclareLaunchArgument(
        name='jsp_gui',
        default_value='true',
        choices=['true', 'false'],
        description='joint state publisher GUI flag')

    urdf_model = LaunchConfiguration('urdf_model')
    declare_urdf_model_path_cmd = DeclareLaunchArgument(
        name='urdf_model',
        default_value=urdf_path,
        description='Absolute path to robot urdf file')

    use_rviz = LaunchConfiguration('use_rviz')
    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='true',
        description='Whether to start RVIZ')

    default_rviz = os.path.join(
        desc_pkg_path,
        'rviz',
        'arm_description.rviz')

    rviz_config_file = LaunchConfiguration('rviz_config_file')
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=default_rviz,
        description='Full path to the RVIZ config file to use')

    robot_description = ParameterValue(Command([
        'xacro', ' ', urdf_model, ' ',
        'robot_name:=', 'arm', ' ',
        'add_world:=', LaunchConfiguration('add_world'), ' ',
        'base_link:=', LaunchConfiguration('base_link'), ' ',
        'base_type:=', LaunchConfiguration('base_type'), ' ',
        'flange_link:=', LaunchConfiguration('flange_link'), ' ',
        'use_camera:=', LaunchConfiguration('use_camera'), ' ',
        'use_gazebo:=', LaunchConfiguration('use_gazebo'), ' ',
        'use_gripper:=', LaunchConfiguration('use_gripper')
    ]), value_type=str)

    # Nodes
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': sim_time,
            'robot_description': robot_description
        }]
    )

    jsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc_pkg_path,
                         'launch',
                         'jsp.launch.py'
                         )
        ),
        launch_arguments={'jsp_gui': gui_flag}.items()
    )

    rviz = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[{'use_sim_time': sim_time}],
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription(ARGUMENTS+[
        jsp_gui,
        declare_rviz_config_file_cmd,
        declare_urdf_model_path_cmd,
        declare_use_rviz_cmd,
        declare_sim_time,
        # Actions
        jsp,
        jsp_gui,
        rsp,
        rviz
    ])
