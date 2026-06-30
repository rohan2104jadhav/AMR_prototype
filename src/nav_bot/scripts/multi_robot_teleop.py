#!/usr/bin/env python3
# multi_robot_teleop.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import tty
import termios
import threading

MSG = """
╔══════════════════════════════════════╗
║     Multi Robot Teleop Controller    ║
╠══════════════════════════════════════╣
║  W/S  : Both robots forward/back     ║
║  A/D  : Both robots left/right       ║
║  Q/E  : Rotate both robots           ║
║                                      ║
║  1+W/S: Only bot1 forward/back       ║
║  1+A/D: Only bot1 left/right         ║
║  3+W/S: Only bot3 forward/back       ║
║  3+A/D: Only bot3 left/right         ║
║                                      ║
║  SPACE: Stop all robots              ║
║  X    : Quit                         ║
╚══════════════════════════════════════╝
"""

# Velocity settings
LINEAR_SPEED  = 0.2
ANGULAR_SPEED = 0.5

def get_key():
    """Read a single keypress from terminal."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return key


class MultiRobotTeleop(Node):
    def __init__(self):
        super().__init__('multi_robot_teleop')

        self.pub_bot1 = self.create_publisher(Twist, '/bot1/cmd_vel', 10)
        self.pub_bot3 = self.create_publisher(Twist, '/bot3/cmd_vel', 10)

        # Current velocities
        self.vel_bot1 = Twist()
        self.vel_bot3 = Twist()

        # Publish at 10Hz continuously so robots don't stop
        self.timer = self.create_timer(0.1, self.publish_velocities)

        self.get_logger().info('Multi Robot Teleop started!')
        print(MSG)

    def publish_velocities(self):
        self.pub_bot1.publish(self.vel_bot1)
        self.pub_bot3.publish(self.vel_bot3)

    def set_both(self, linear_x=0.0, angular_z=0.0):
        """Set velocity for both robots."""
        self.vel_bot1.linear.x  = linear_x
        self.vel_bot1.angular.z = angular_z
        self.vel_bot3.linear.x  = linear_x
        self.vel_bot3.angular.z = angular_z

    def set_bot1(self, linear_x=0.0, angular_z=0.0):
        """Set velocity for bot1 only."""
        self.vel_bot1.linear.x  = linear_x
        self.vel_bot1.angular.z = angular_z

    def set_bot3(self, linear_x=0.0, angular_z=0.0):
        """Set velocity for bot3 only."""
        self.vel_bot3.linear.x  = linear_x
        self.vel_bot3.angular.z = angular_z

    def stop_all(self):
        self.vel_bot1 = Twist()
        self.vel_bot3 = Twist()
        print('🛑 All robots stopped')

    def print_status(self):
        print(
            f"\r  bot1 → lin: {self.vel_bot1.linear.x:+.1f}  "
            f"ang: {self.vel_bot1.angular.z:+.1f}   "
            f"bot3 → lin: {self.vel_bot3.linear.x:+.1f}  "
            f"ang: {self.vel_bot3.angular.z:+.1f}   ",
            end='', flush=True
        )


def keyboard_loop(node: MultiRobotTeleop):
    """Run in a separate thread — reads keypresses and updates velocities."""
    selected_robot = None   # None = both, '1' = bot1, '3' = bot3

    while rclpy.ok():
        key = get_key()

        # Robot selector
        if key == '1':
            selected_robot = '1'
            print('\n🤖 Controlling: bot1 only')
            continue
        elif key == '3':
            selected_robot = '3'
            print('\n🤖 Controlling: bot3 only')
            continue
        elif key == 'b':
            selected_robot = None
            print('\n🤖 Controlling: BOTH robots')
            continue

        # Movement keys
        if selected_robot == '1':
            if   key == 'w': node.set_bot1( LINEAR_SPEED,  0.0)
            elif key == 's': node.set_bot1(-LINEAR_SPEED,  0.0)
            elif key == 'a': node.set_bot1( 0.0,  ANGULAR_SPEED)
            elif key == 'd': node.set_bot1( 0.0, -ANGULAR_SPEED)
            elif key == ' ': node.set_bot1(0.0, 0.0); print('\n🛑 bot1 stopped')

        elif selected_robot == '3':
            if   key == 'w': node.set_bot3( LINEAR_SPEED,  0.0)
            elif key == 's': node.set_bot3(-LINEAR_SPEED,  0.0)
            elif key == 'a': node.set_bot3( 0.0,  ANGULAR_SPEED)
            elif key == 'd': node.set_bot3( 0.0, -ANGULAR_SPEED)
            elif key == ' ': node.set_bot3(0.0, 0.0); print('\n🛑 bot3 stopped')

        else:  # Both robots
            if   key == 'w': node.set_both( LINEAR_SPEED,  0.0)
            elif key == 's': node.set_both(-LINEAR_SPEED,  0.0)
            elif key == 'a': node.set_both( 0.0,  ANGULAR_SPEED)
            elif key == 'd': node.set_both( 0.0, -ANGULAR_SPEED)
            elif key == 'q': node.set_both( 0.0,  ANGULAR_SPEED)
            elif key == 'e': node.set_both( 0.0, -ANGULAR_SPEED)
            elif key == ' ': node.stop_all()

        # Quit
        if key == 'x':
            print('\n👋 Quitting...')
            node.stop_all()
            rclpy.shutdown()
            break

        node.print_status()


def main():
    rclpy.init()
    node = MultiRobotTeleop()

    # Run keyboard input in separate thread
    # so ROS2 spinning doesn't block it
    kb_thread = threading.Thread(target=keyboard_loop, args=(node,), daemon=True)
    kb_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_all()
        node.destroy_node()


if __name__ == '__main__':
    main()
