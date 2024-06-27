import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_mr_robot_desc = get_package_share_directory('hexapod_description')

    # launch GZ Sim with world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': os.path.join(pkg_mr_robot_desc, 'worlds', 'world.sdf') + ' -v 4'
        }.items()
    )

    # spawn robot with rviz
    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_mr_robot_desc, 'launch', 'robot.launch.py')
        )
    )

    return LaunchDescription([
        gz_sim,
        robot
    ])
