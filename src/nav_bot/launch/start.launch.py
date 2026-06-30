import os
import yaml
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

# Utility imports from your scripts/utils.py
from scripts.utils import create_namespaced_bridge_yaml, generate_rviz_config, patch_robot_description

def generate_launch_description():
    pkg_nav_bot = get_package_share_directory('nav_bot')
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')
    
    # Paths
    xacro_path = os.path.join(pkg_nav_bot, 'description', 'bot.urdf.xacro')
    world_path = os.path.join(pkg_nav_bot, 'worlds', 'empty_world.sdf')
    robot_config_path = os.path.join(pkg_nav_bot, 'config', 'robots.yaml')
    bridge_template = os.path.join(pkg_nav_bot, 'config', 'gz_bridge.yaml')
    # base_rviz_path = os.path.join(pkg_nav_bot, 'rviz', 'multi_robot_base.rviz')
    
    # Nav2 / SLAM Params
    nav2_params_path = os.path.join(pkg_nav_bot, 'config', 'nav2_params.yaml')
    slam_params_path = os.path.join(pkg_nav_bot, 'config', 'slam_toolbox.yaml')

    # Start Gazebo
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {world_path}'}.items()
    )

    ld = LaunchDescription()
    ld.add_action(gz_sim)

    with open(robot_config_path, 'r') as f:
        robots = yaml.safe_load(f)['robots']

    for robot in robots:
        if not robot.get('enabled', True): continue
        
        namespace = robot['name']
        
        # 1. Process and Patch URDF
        robot_description_raw = xacro.process_file(xacro_path, mappings={'robot_name': namespace}).toxml()
        patched_robot_description = patch_robot_description(robot_description_raw, namespace)

        # 2. Setup Namespaced Params (Arshadlab Secret)
        configured_params = RewrittenYaml(
            source_file=nav2_params_path,
            root_key=namespace,
            param_rewrites={},
            convert_types=True)

        # 3. Create Bridge and RViz Configs
        tmp_bridge = create_namespaced_bridge_yaml(bridge_template, namespace)
        # tmp_rviz = generate_rviz_config(namespace, base_rviz_path)

        # 4. Robot State Publisher
        ld.add_action(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=namespace,
            parameters=[{'robot_description': patched_robot_description, 'use_sim_time': True, 'frame_prefix': namespace+'/'}]
        ))

        # 5. Spawner
        ld.add_action(Node(
            package='ros_gz_sim',
            executable='create',
            namespace=namespace,
            arguments=['-name', namespace, '-topic', 'robot_description', '-x', str(robot['x_pose']), '-y', str(robot['y_pose']), '-z', '0.1']
        ))

        # 6. Bridge
        ld.add_action(Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            namespace=namespace,
            arguments=['--ros-args', '-p', f'config_file:={tmp_bridge}']
        ))

        # 7. SLAM Toolbox
        ld.add_action(Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            namespace=namespace,
            parameters=[slam_params_path, {'use_sim_time': True}],
            remappings=[('/map', 'map'), ('/tf', '/tf'), ('/tf_static', '/tf_static')]
        ))

        # 8. Navigation Stack (Individual Nodes)
        nav_nodes = {
            'controller_server': 'nav2_controller',
            'planner_server': 'nav2_planner',
            'behavior_server': 'nav2_behaviors', # Note the 's' at the end
            'bt_navigator': 'nav2_bt_navigator'
        }

        for node_name, package_name in nav_nodes.items():
            ld.add_action(Node(
                package=package_name,
                executable=node_name,
                name=node_name,
                namespace=namespace,
                output='screen',
                parameters=[configured_params],
                remappings=[('/tf', '/tf'), ('/tf_static', '/tf_static')]
            ))

        # 9. Lifecycle Manager (Update the list here too)
        ld.add_action(Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            namespace=namespace,
            output='screen',
            parameters=[{
                'use_sim_time': True, 
                'autostart': True, 
                'node_names': list(nav_nodes.keys()) # Uses ['controller_server', ...]
            }]
        ))

        # 10. RViz
        ld.add_action(Node(
            package='rviz2',
            executable='rviz2',
            namespace=namespace,
            # arguments=['-d', tmp_rviz],
            parameters=[{'use_sim_time': True}]
        ))

    # Global Clock Bridge
    ld.add_action(Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']
    ))

    return ld