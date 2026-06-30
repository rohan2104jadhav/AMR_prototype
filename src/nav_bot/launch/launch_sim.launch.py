import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # 1. Find the package directories
    pkg_name = 'nav_bot'
    pkg_share = get_package_share_directory(pkg_name)
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    default_rviz_config_path = os.path.join(pkg_share, 'config', 'config.rviz')
    default_world_path = os.path.join(pkg_share, 'worlds', 'small_house_world.sdf')
    use_sim_time = LaunchConfiguration('use_sim_time')
    world_file = LaunchConfiguration('world')  
    slam_params_file = os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml')  



    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(pkg_name),
                'launch',
                'rsp.launch.py'
            )
        ), 
        launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'joystick.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    twist_mux_params = os.path.join(pkg_share, 'config', 'twist_mux.yaml')
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        parameters=[twist_mux_params, {'use_sim_time': use_sim_time}],
        remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')]
    )

    # 4. Include the Gazebo Classic simulation launch file
    # 'gazebo.launch.py' launches both the server (gzserver) and client (gzclient)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -v 4 ', world_file], 'on_exit_shutdown': 'true'}.items()
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        parameters=[{'use_sim_time': use_sim_time}]

    )    

    # 6. Node to spawn the entity in Gazebo Classic
    # usage: ros2 run ros_gz_sim spawn_entity.py -topic robot_description -entity my_bot2
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'nav_bot',
            '-z', '0.1'
        ],
        output='screen'
    )

    bridge_params = os.path.join(get_package_share_directory(pkg_name), 'config', 'gz_bridge.yaml')
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}'
        ]
    )

    ros_gz_image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=["/camera/image_raw"]
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    slam_toolbox_node = Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                parameters=[
                    slam_params_file,
                    {
                        'use_sim_time': True,
                        'map_frame': 'map',

                    }
                ],
            )    


    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_world_cmd = DeclareLaunchArgument(
        name='world',
        default_value=default_world_path,
        description='Absolute path to world file to load in Gazebo'
    )

    # 7. BRIDGE IS NOT NEEDED IN CLASSIC
    # Gazebo Classic handles ROS 2 communication natively via plugins.

    return LaunchDescription([
    DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path, description='Absolute path to rviz config file'),
    declare_use_sim_time_cmd,
    declare_world_cmd,
    gazebo,
    spawn_entity,
    rviz_node,    
    rsp,
    joint_broad_spawner,
    diff_drive_spawner,
    joystick,
    twist_mux_node,
    ros_gz_bridge,
    ros_gz_image_bridge,
    # slam_toolbox_node
    ])