import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # ==========================
    # Launch Arguments / Configurations
    # ==========================
    robot_model_path = LaunchConfiguration('robot_model_path')
    map_simulation_path = LaunchConfiguration('map_simulation_path')
    use_sim_time = LaunchConfiguration('use_sim_time')
    gz_ros_bride_params_path = LaunchConfiguration('gz_ros_bride_params')

    # ==========================
    # Package Directories
    # ==========================
    sim_pkg = get_package_share_directory("simulation_gz")

    # ==========================
    # Gazebo Simulation Launch
    # ==========================
    gz_sim_node = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': [map_simulation_path, ' -r -v 4']
        }.items()
    )

    # ==========================
    # Gazebo Resource Paths
    # ==========================
    # Path to the simulation models directory
    models_path = os.path.join(sim_pkg, "models")

    # Add the models directory to Gazebo's resource search path
    gz_sim_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=models_path + ":" +
              os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    )

    # ==========================
    # Robot Spawn Configuration
    # ==========================
    spawn_drone_ = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', 'maze',
            '-file', robot_model_path,
            '-name', 'X3',
            '-x', '1', '-y', '1', '-z', '1'
        ],
        output='screen'
    )

    # ==========================
    # ROS - Gazebo Bridge
    # ==========================
    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={gz_ros_bride_params_path}',
        ],
        output='screen',
    )

    # ==========================
    # Launch Description
    # ==========================
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_model_path',
            default_value = os.path.join(models_path, "X3", "model.sdf"),
            description = 'Path to the model of the robot'),
        
        DeclareLaunchArgument(
            'map_simulation_path',
            default_value = os.path.join(sim_pkg, "worlds/maze", "model.sdf"),
            description = 'Path to the simulation world'),
        
        DeclareLaunchArgument(
            'use_sim_time',
            default_value = 'true',
            description = 'Use of the simulation time',
        ),
        
        DeclareLaunchArgument(
            'gz_ros_bride_params_path',
            default_value = os.path.join(sim_pkg, 'config', 'bridge_params.yaml'),
            description = 'Path to the bridge params between ros and gz'
        ),

        gz_sim_resource_path,
        
        gz_sim_node,
        
        #spawn_drone_cmd,
        
        #start_gazebo_ros_bridge_cmd
    ])
