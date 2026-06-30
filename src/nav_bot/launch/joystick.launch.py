from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    joy_Params = os.path.join(
        get_package_share_directory('nav_bot'),
        'config',
        'joystick.yaml')

    joy_node = Node(
        package='joy',
        executable='joy_node',
        parameters=[joy_Params, {'use_sim_time': True}],
        )
    
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_node',
        parameters=[joy_Params, {'use_sim_time': True}],
        remappings=[('/cmd_vel','/cmd_vel_joy')],
        )
    
    twist_stamper = Node(
        package='twist_stamper',
        executable='twist_stamper',
        parameters=[{'use_sim_time': True}],
        remappings=[('/cmd_vel_in','/diff_cont/cmd_vel_unstamped'),
                    ('/cmd_vel_out','/diff_cont/cmd_vel'),
                    ]
        )


    return LaunchDescription([
        joy_node,
        teleop_node,
        twist_stamper
    ])