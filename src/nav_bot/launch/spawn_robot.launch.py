import os
from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.parameter_descriptions import ParameterValue 
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_description = get_package_share_directory('nav_bot')
    pkg_nav = get_package_share_directory('nav_bot')
    
    robot_name = LaunchConfiguration('robot_name')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    
    nav2_params_file = os.path.join(pkg_nav, 'config', 'nav2_params.yaml')
    slam_params_file = os.path.join(pkg_nav, 'config', 'slam_toolbox.yaml')

    xacro_file = os.path.join(pkg_description, 'description', 'bot.urdf.xacro')
    robot_description_config = ParameterValue(
        Command(['xacro ', xacro_file, ' robot_name:=', robot_name]),
        value_type=str  # ✅ Forces it to be treated as string not yaml
    )

    return LaunchDescription([
        GroupAction([
            PushRosNamespace(robot_name),

            # 1. Robot State Publisher — NO frame_prefix (Gazebo handles TF)
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                parameters=[{
                    'robot_description': robot_description_config,
                    'use_sim_time': True,
                    # ✅ REMOVED frame_prefix — Gazebo TF bridge already
                    # publishes correctly prefixed frames (bot1/base_link etc.)
                }],
                remappings=[
                    # ✅ FIX: Connect namespaced joint_states from Gazebo bridge
                    ('/joint_states', 'joint_states'),
                ]
            ),

            # 2. Gazebo Bridge
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=[
                    ['/model/', robot_name, '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'],
                    ['/model/', robot_name, '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
                    ['/model/', robot_name, '/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry'],
                    ['/model/', robot_name, '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'],
                    ['/model/', robot_name, '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model'],
                    ['/model/', robot_name, '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image'],
                    ['/model/', robot_name, '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'],
                ],
                remappings=[
                    (['/model/', robot_name, '/cmd_vel'], 'cmd_vel'),
                    (['/model/', robot_name, '/scan'], 'scan'),
                    (['/model/', robot_name, '/odometry'], 'odom'),
                    (['/model/', robot_name, '/tf'], '/tf'),        # ✅ Must be global /tf
                    (['/model/', robot_name, '/joint_states'], 'joint_states'),
                    (['/model/', robot_name, '/camera/image_raw'], 'camera/image_raw'),
                    (['/model/', robot_name, '/camera/camera_info'], 'camera/camera_info'),
                ],
            ),

            Node(
                package='ros_gz_image',
                executable='image_bridge',
                arguments=['/model/', robot_name, '/camera/image_raw']
            ),


            # 3. Spawn Robot in Gazebo
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=['-topic', 'robot_description', '-name', robot_name,
                           '-x', x_pose, '-y', y_pose, '-z', z_pose],
            ),
            # 4. SLAM Toolbox
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                parameters=[
                    slam_params_file,
                    {
                        'use_sim_time': True,
                        'map_frame': 'map',
                        # ✅ Dynamically build frame names per robot
                        'odom_frame': PythonExpression(["'", robot_name, "/odom'"]),
                        'base_frame': PythonExpression(["'", robot_name, "/base_link'"]),
                        'scan_topic': PythonExpression(["'/", robot_name, "/scan'"]),

                    }
                ],
                remappings=[
                    ('/tf', '/tf'),
                    ('/tf_static', '/tf_static'),
                    ('map', '/map'),
                    ('map_metadata', '/map_metadata'),
                ]
            ),
            # 5. Nav2 Nodes
        #     Node(
        #         package='nav2_controller',
        #         executable='controller_server',
        #         parameters=[
        #             nav2_params_file,
        #             {
        #                 'use_sim_time': True,
        #                 # ✅ Full frame names
        #                 'local_costmap.local_costmap.robot_base_frame': PythonExpression(["'", robot_name, "/base_link'"]),
        #                 'FollowPath.critics': [
        #                     'RotateToGoal', 'Oscillation', 'BaseObstacle',
        #                     'GoalAlign', 'PathAlign', 'PathDist', 'GoalDist'
        #                 ],
        #             }
        #         ],
        #         remappings=[('/tf', '/tf'), ('/tf_static', '/tf_static')]
        #     ),
        #     Node(
        #         package='nav2_planner',
        #         executable='planner_server',
        #         parameters=[
        #             nav2_params_file,
        #             {
        #                 'use_sim_time': True,
        #                 'global_costmap.global_costmap.robot_base_frame': PythonExpression(["'", robot_name, "/base_link'"]),
        #             }
        #         ],
        #         remappings=[('/tf', '/tf'), ('/tf_static', '/tf_static')]
        #     ),
        #     Node(
        #         package='nav2_behaviors',
        #         executable='behavior_server',
        #         parameters=[
        #             nav2_params_file,
        #             {
        #                 'use_sim_time': True,
        #                 'robot_base_frame': PythonExpression(["'", robot_name, "/base_link'"]),
        #             }
        #         ],
        #         remappings=[('/tf', '/tf'), ('/tf_static', '/tf_static')]
        #     ),
        #     Node(
        #         package='nav2_bt_navigator',
        #         executable='bt_navigator',
        #         parameters=[
        #             nav2_params_file,
        #             {
        #                 'use_sim_time': True,
        #                 'robot_base_frame': PythonExpression(["'", robot_name, "/base_link'"]),
        #             }
        #         ],
        #         remappings=[('/tf', '/tf'), ('/tf_static', '/tf_static')]
        #     ),
        #     Node(
        #         package='nav2_lifecycle_manager',
        #         executable='lifecycle_manager',
        #         name='lifecycle_manager_navigation',
        #         parameters=[{
        #             'use_sim_time': True,
        #             'autostart': True,
        #             'node_names': [
        #                 'controller_server',
        #                 'planner_server',
        #                 'behavior_server',
        #                 'bt_navigator'
        #             ]
        #         }]
        #     ),     
         ])
    ])



