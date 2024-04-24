import launch
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
import os
import xacro
import launch_ros.descriptions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share_dir = get_package_share_directory('hexapod_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_ugv_sim = get_package_share_directory('hexapod_description')
    world_path = pkg_ugv_sim + "/worlds/" + "world_test.sdf"
    hexapod_description = share_dir 
    parameters=[{'robot_description': launch_ros.descriptions.ParameterValue( launch.substitutions.Command(['xacro ',os.path.join(hexapod_description,'hexapod.xacro')]), value_type=str)  }]
    xacro_file = os.path.join(share_dir, 'urdf', 'hexapod.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    robot_urdf = "/home/aryan/hexapod/src/Hexapod/hexapod_description/urdf/hexapod.xacro"
    gazebo_file = robot_urdf
    robot_xacro_config = xacro.process_file(robot_urdf)
    robot_urdf = robot_xacro_config.toxml()
    rviz_file = robot_urdf
    spawn_robot = Node(package = "ros_gz_sim",
    executable = "create",
    arguments = ["-topic", "/robot_description",
                "-name", "hexapod",
                "-allow_renaming", "true",
                "-z", "0",
                "-x", "0",
                "-y", "0",
                "-Y", "0",
                ],
    output='screen'
    )

    # robot_state_publisher_node = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     name='robot_state_publisher',
    #     parameters=[
    #         {'robot_description': robot_urdf}
    #     ]
    # )

    # joint_state_publisher_node = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher'
    # )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
             'gz_args' : world_path + " -r"
        }.items()          
    )

    with_bridge = LaunchConfiguration('with_bridge')



    # gazebo_client = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare('gazebo_ros'),
    #             'launch',
    #             'gzclient.launch.py'
    #         ])
    #     ])
    # )

    # urdf_spawn_node = Node(
    #     package='gazebo_ros',
    #     executable='spawn_entity.py',
    #     arguments=[
    #         '-entity', 'hexapod',
    #         '-topic', 'robot_description'
    #     ],
    #     output='screen'
    # )

    return LaunchDescription([
        # robot_state_publisher_node,
        # joint_state_publisher_node,
        gz_sim,
        # urdf_spawn_node,
    ])