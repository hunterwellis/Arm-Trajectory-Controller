import os
import xacro
from pathlib import Path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    # package paths
    desc_pkg_path = get_package_share_directory('arm_description')
    sim_pkg_path = get_package_share_directory('arm_simulation')
    ros_gz_pkg = get_package_share_directory('ros_gz_sim')

    urdf_path = os.path.join(desc_pkg_path, 'model', 'camera', 'camera.urdf.xacro')

    world = LaunchConfiguration('world')
    declare_world = DeclareLaunchArgument(
        name='world',
        default_value='arm_world.sdf',
        description='world file (default arm_world.sdf)'
    )

    world_path = PathJoinSubstitution([
        sim_pkg_path,
        'worlds',
        'dataset_worlds',
        world
    ])

    # Set gazebo sim resource path
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(sim_pkg_path, 'worlds'), ':' +
            str(Path(desc_pkg_path).parent.resolve())
            ]
        )

    # Launch Gazebo
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_pkg,
                         'launch',
                         'gz_sim.launch.py'
                         )
        ),
        launch_arguments=[('gz_args', [world_path, ' -v 4', ' -r']),
                          ('use_sim_time', 'true')]
    )

    doc = xacro.process_file(urdf_path, mappings={'use_sim' : 'true'})

    robot_desc = doc.toprettyxml(indent='  ')

    params = {'robot_description': robot_desc}

    # Gazebo spawn
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
                   '-string', robot_desc,
                   # '-topic', '/robot_description'
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '1.04',
                   '-R', '0.0',
                   '-P', '0.0',
                   '-Y', '0.0',
                   '-name', 'arm',
                   '-allow_renaming', 'false'],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/image_raw@sensor_msgs/msg/Image@gz.msgs.Image'],
        output='screen'
    )

    # state pub
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[params,
                    {'use_sim_time': True}]
    )

    return LaunchDescription([
        gazebo_resource_path,
        declare_world,
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        bridge,
    ])
