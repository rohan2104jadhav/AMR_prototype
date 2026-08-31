import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 1. Find the package directories
    pkg_name = 'nav_bot'
    pkg_share = get_package_share_directory(pkg_name)
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    default_rviz_config_path = os.path.join(pkg_share, 'config', 'config.rviz')
    default_world_path = os.path.join(pkg_share, 'worlds', 'small_house_world.sdf')
    default_map_path = os.path.join(pkg_share, 'map', 'new_map_save.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    world_file = LaunchConfiguration('world')
    map_yaml_file = LaunchConfiguration('map')

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'rsp.launch.py')
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

    # 4. Gazebo (this is the new Gazebo / "gz sim", NOT Gazebo Classic —
    # ros_gz_sim, ros_gz_bridge and ros_gz_image below all confirm this)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r -v 4 ', world_file],
            'gz_version': '6',
            'on_exit_shutdown': 'true'
        }.items()
    )

    # 6. Spawn the entity in gz sim
    # (gz sim's spawner executable is 'create', from ros_gz_sim — this was already correct)
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

    bridge_params = os.path.join(pkg_share, 'config', 'gz_bridge.yaml')
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

    # 7. AMCL (localization against a pre-built map). These are launch files,
    # not node executables, so they must be included, not run via Node().
    # Do NOT also add standalone map_server / lifecycle_manager_localization
    # Node()s elsewhere in this file — localization_launch.py already starts
    # both internally under these exact names, and a second lifecycle manager
    # racing to configure/activate the same node names is what produces
    # "Unable to start transition ...: Transition is not registered." errors.
    amcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
        }.items()
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
            'use_sim_time': use_sim_time,
        }.items()
        # Note: map_subscribe_transient_local is a QoS setting for map_server/
        # costmap nodes, not a topic remap — set it in nav2_params.yaml instead
        # of passing it here.
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        parameters=[{'use_sim_time': use_sim_time}]
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

    declare_map_cmd = DeclareLaunchArgument(
        name='map',
        default_value=default_map_path,
        description='Full path to map yaml file to load for AMCL localization'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='rvizconfig',
            default_value=default_rviz_config_path,
            description='Absolute path to rviz config file'
        ),
        declare_use_sim_time_cmd,
        declare_world_cmd,
        declare_map_cmd,
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
        amcl_launch,
        nav2_launch,
    ])