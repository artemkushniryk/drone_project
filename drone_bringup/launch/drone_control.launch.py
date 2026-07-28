import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, IcludeLaunchDescriptionSource, SetEnvironmentVariable, TimerAction, Shutdown, Include
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from launch.conditions import IfCondition
from launch_ros.actions import Node

import xacro
import yaml

def generate_launch_description():
	
	print("Launching drone_control.launch.py *******************************")

	# ==========================
	# Launch Arguments / Configurations
	# ==========================
	geometry_robot_config_file_path = LaunchConfiguration('geometry_robot_config_file')
	robot_description_file_path = LaunchConfiguration('robot_model_path')

	use_sim_time = LaunchConfiguration('use_sim_time')


	# Robot description
	robot_urdf = xacro.process_file(robot_description_file_path, mappings={
									'name': 'Drone_X3',
									'geometry_robot_config_file': geometry_robot_config_file_path,
									# 'geometry_sensing_config_file': geometry_sensing_config_file_path,
									# 'simulation_config_file': simulation_config_file_path,
									# 'controller_config_file': controller_config_file_path,
									'use_sim_time': str(use_sim_time)}).toxml()



	joint_state_publisher_node = Node(
	    package='controller_manager',
	    executable='spawner',
	    arguments=['joint_state_broadcaster',"--controller-manager", "controller_manager"],
	    parameters=[{'use_sim_time':use_sim_time}]
	)

	robot_description_node = Node(
	    package='robot_state_publisher',
	    executable='robot_state_publisher',
	    name='robot_state_publisher',
	    output='log',
	    parameters=[
	        {'use_sim_time': True},
	        {'robot_description': robot_urdf},
	    ],
	)

	return LaunchDescription([
		DeclareLaunchArgument(
		    'use_sim_time',
		    default_value = 'true',
		    description = 'Use of the simulation time',
		),
		DeclareLaunchArgument(
			'robot_description_file_path',
			default_value = '',
			description = 'Description of the robot'
		),
		DeclareLaunchArgument(
			'robot_name',
			default_value = "UAV1",
			description = "Drone name"
		),
		DeclareLaunchArgument(
			'robot_namespace',
			default_value = 'drone_robot00',
			description = 'Robot namespace'
		),

        robot_description_node,
        
		joint_state_publisher_node,
	])