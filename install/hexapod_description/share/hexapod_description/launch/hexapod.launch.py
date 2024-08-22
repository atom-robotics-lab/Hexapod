import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    #pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_hexapod_desc = get_package_share_directory('hexapod_description')
    pkg_hexapod_control = get_package_share_directory('hexapod_control')

    # launch GZ Sim with empty world
    #gz_sim = IncludeLaunchDescription(
    #            PythonLaunchDescriptionSource(
    #                os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
    #            ),
    #            launch_arguments={'gz_args': '-r empty.sdf'}.items()     
    #        )
    
    # spawn robot with rviz
    robot = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_hexapod_desc, 'launch', 'robot.launch.py')
                )
            )

    return LaunchDescription([
        #gz_sim,
        robot,
        ExecuteProcess(
            cmd=[os.path.join(pkg_hexapod_control, 'scripts', 'bot_controller.py')],
            output='screen'
        ),
    ])