import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from launch import LaunchDescription
from launch.substitution import LaunchConfiguration

def generate_launch_description():

    robot_model_path = LaunchConfiguration('robot_model_path')

    robot_pkg = get_package_share_directory("robot_gazebo")
    sim_pkg = get_package_share_directory("simulation_gz")
    models_path = os.path.join(sim_pkg, "models")
    robot_desc = open(robot_model_path).read()

    gz_sim_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=models_path + ":" + os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    )

    simulation_world_file_path = Path(sim_pkg, "worlds/maze/model.sdf").as_posix()

    gz_sim_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch',
            'gz_sim.launch.py')),
        launch_arguments=[('gz_args', simulation_world_file_path + ' -r -v 4')]
    )

    spawn_drone_cmd = Node(
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

    # Start the ROS 2 Gazebo bridge (uncomment and configure if needed)
    bridge_params = os.path.join(
        get_package_share_directory('robot_gazebo'),
        'config',
        'gz_ros2_bridge.yaml'
    )

    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_model_path',
            default_value = os.path.join(models_path, "X3", "model.sdf"),
            description = 'Path to the model of the robot'),
        gz_sim_resource_path,
        gz_sim_node,
        spawn_drone_cmd,
        start_gazebo_ros_bridge_cmd
    ])