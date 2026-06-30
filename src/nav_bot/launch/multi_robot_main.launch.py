import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Setup folders and paths
    pkg_gazebo = get_package_share_directory('nav_bot')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    rviz_config_path = os.path.join(pkg_gazebo, 'config', 'multi_bot_config.rviz')

    
    # Path to your custom world (Replace 'empty.sdf' with your actual world file if needed)
    world_file = os.path.join(pkg_gazebo, 'worlds', 'empty_world.sdf') 
    
    # Path to your robots configuration
    robots_config_path = os.path.join(pkg_gazebo, 'config', 'robots.yaml')
    
    # 2. Load the robots.yaml file
    with open(robots_config_path, 'r') as f:
        robots = yaml.safe_load(f)['robots']

    # 3. Create the Gazebo Initiation Action
    # This starts the Gazebo Harmonic Simulator
    gazebo_start = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r {world_file}' # -r runs the simulation immediately
        }.items(),
    )

    # 4. Create the Launch Description
    ld = LaunchDescription()

    # Add Gazebo to the launch
    ld.add_action(gazebo_start)


    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
    )
    ld.add_action(rviz_node)

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    )
    
    ld.add_action(clock_bridge)


    # 5. Loop through the robots.yaml and spawn the enabled robots
    for robot in robots:
        if robot.get('enabled', True):
            # We use a TimerAction to wait 5 seconds before spawning robots.
            # This ensures Gazebo is fully loaded and ready to accept the "create" request.
            spawn_robot_event = TimerAction(
                period=5.0, # Wait 5 seconds for Gazebo to stabilize
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(pkg_gazebo, 'launch', 'spawn_robot.launch.py')
                        ),
                        launch_arguments={
                            'robot_name': robot['name'],
                            'x_pose': str(robot['x_pose']),
                            'y_pose': str(robot['y_pose']),
                            'z_pose': str(robot['z_pose']),
                        }.items()
                    )
                ]
            )
            ld.add_action(spawn_robot_event)

    return ld

