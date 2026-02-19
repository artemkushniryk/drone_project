import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from launch.conditions import IfCondition
from launch.actions import Shutdown, IncludeLaunchDescription
from launch_ros.actions import Node



from launch.launch_description_sources import PythonLaunchDescriptionSource



def generate_launch_description():
    print("Launching gazebo_sim.launch.py **********************************")

    # Get the path to the package
    drone_pkg_path = get_package_share_directory("robot_gazebo")
    sim_path = get_package_share_directory("simulation_gz")
    
    # Open help to use the robot in the robot_description
    models_path = os.path.join(drone_pkg_path, "models")
    drone_model_path = os.path.join(models_path, "X3", "model.sdf")
    robot_desc = open(drone_model_path).read()


    # Run simulation
    sim_launch_path = os.path.join(sim_path, 'launch', 'simulation_gz.launch.py')
    sim_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(sim_launch_path),
                       launch_arguments={'robot_model_path': str(drone_model_path)})

    robot_desccription_node=Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
        ],
        description = "Robot Description Launch"
    )

    launch_rviz = LaunchConfiguration('launch_rviz', default='true')

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        condition=IfCondition(launch_rviz),
        on_exit=Shutdown(),
    )



    return LaunchDescription([
        #robot_desccription_node,
        #sim_launch,
        rviz_node

    ])

