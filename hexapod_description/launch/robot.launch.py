import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription,ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit



def generate_launch_description():	

	# get the required paths of packages & files
	pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
	pkg_mr_robot_desc = get_package_share_directory('hexapod_description')
	xacro_path = pkg_mr_robot_desc + '/urdf/hexapod.xacro'
	
	# TODO: take rviz config file as launch arg 
	# use this one as default
	#rviz_config = pkg_mr_robot_desc + '/config/urdf.rviz'
	#bridge_config = pkg_mr_robot_desc + '/config/bridge.yaml'

	# launch configs to use launch args
	use_sim_time = LaunchConfiguration('use_sim_time')

	rviz_config_file = os.path.join(pkg_mr_robot_desc, 'config', 'display.rviz')

	# create urdf from xacro 
	robot_xacro_config = xacro.process_file(xacro_path)
	robot_urdf = robot_xacro_config.toxml()

	# joint state publisher
	state_publisher = Node(package = 'robot_state_publisher',
								executable = 'robot_state_publisher',
								parameters = [{'robot_description': ParameterValue(Command( \
											['xacro ', os.path.join(pkg_mr_robot_desc, 'urdf/hexapod.xacro'),
											]), value_type=str)}]
								)

	# spawn robot in gz sim using urdf
	spawn_robot = Node(package = "ros_gz_sim",
                           executable = "create",
                           arguments = ["-topic", "/robot_description",
                                        "-name", "hexapod",
                                        "-allow_renaming", "true",
                                        "-z", "1.0",
                                        "-x", "2.0",
                                        "-y", "0.0",
                                        "-Y", "-1.57",
                                        ],
							output='screen'
                           )
	
	load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )
	load_hexapod_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'hexapod_controller'],
        output='screen'
    )

	rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

	arg_use_sim_time = DeclareLaunchArgument('use_sim_time',
											default_value='true',
											description="Enable sim time from /clock")
	
	# argument to specify if rviz needs to be launched

	
	
	return LaunchDescription([
		arg_use_sim_time,
		RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_robot,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
		RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_hexapod_controller],
            )
        ),

		spawn_robot,

		state_publisher,

#		rviz_node,

	])