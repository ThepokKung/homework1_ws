#!/usr/bin/python3

import rclpy
import sys
import termios
import tty
import numpy as np
import os
import yaml

from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_srvs.srv import Empty
from turtlesim.msg import Pose
from turtlesim_plus_interfaces.srv import GivePosition

key_bindings = {
    'w': (1, 0),   # Move forward
    's': (-1, 0),  # Move backward
    'a': (0, 1),   # Turn left
    'd': (0, -1),  # Turn right
}

class TurtleTeleop(Node):
    def __init__(self):
        super().__init__('turtle_teleop')
        self.frequency = 100
        self.turtle_pose = np.array([0.0, 0.0, 0.0])
        self.pizza_pose = np.array([0.0, 0.0, 0.0])
        self.pizza_pose_save = []
        self.pizza_count_max = 20
        self.pizza_count = 0
        self.linear_gain = 2.0
        self.angular_gain = 2.0
        self.pizzapath_file = os.path.expanduser('~/ros2_hw1-exam1/src/funny_turtle/config/pizzapath.yaml')

        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

        self.spawn_pizza_cilent = self.create_client(GivePosition, '/spawn_pizza')

        self.create_timer(1.0 / self.frequency, self.timers_callback)

        self.print_info()

    def print_info(self):
        print('Control Your TurtleSim')
        print('----------------------')
        print('Move around:')
        print('          w          ')
        print('    a     s     d    ')
        print('                     ')
        print('spacebar: force stop')
        print('e: spawn pizza')
        print('q: clear pizza')
        print('f: save path')
        print('Ctrl + C: quit')

    def cmdvel(self, v, w):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.cmd_vel_pub.publish(msg)

    def pose_callback(self, msg):
        # self.get_logger().info(f'Pose: x={msg.x}, y={msg.y}, theta={msg.theta}')
        self.turtle_pose[0] = msg.x
        self.turtle_pose[1] = msg.y
        self.turtle_pose[2] = msg.theta

    def spawn_pizza(self, x, y):
        position_request = GivePosition.Request()
        position_request.x = x
        position_request.y = y
        self.spawn_pizza_cilent.call_async(position_request)
        self.pizza_pose_save.append({'x': x, 'y': y})

    def save_pizza_positions(self):
        os.makedirs(os.path.dirname(self.pizzapath_file), exist_ok=True)
        data = {'pizzas': [{'x': pos[0], 'y': pos[1]} for pos in self.pizza_pose_save]}
        with open(self.pizzapath_file, 'w') as file:
            yaml.dump(data, file)

    def get_key(self):
        # Get the current terminal settings
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            # Set the terminal to raw mode
            tty.setraw(sys.stdin.fileno())
            key = sys.stdin.read(1)
        finally:
            # Restore the terminal to the previous settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        return key

    def keyboard_teleop(self):
        try:
            key = self.get_key()
            if key in key_bindings:
                linear = key_bindings[key][0] * self.linear_gain
                angular = key_bindings[key][1] * self.angular_gain
                self.cmdvel(linear, angular)
            elif key == 'e':
                if self.pizza_count < self.pizza_count_max:
                    self.spawn_pizza(self.turtle_pose[0], self.turtle_pose[1])
                    self.pizza_count += 1
                    print(f'Spawned pizza {self.pizza_count}/{self.pizza_count_max}')
                else:
                    print('Pizza limit reached!')
            elif key == 'f':
                # print(self.pizza_pose_save)
                self.save_pizza_positions()
                print('Pizza positions saved to config/pizzapath.yaml')
            elif key == ' ':
                self.cmdvel(0.0, 0.0)
            elif ord(key) == 3:
                print('Exiting...')
                raise KeyboardInterrupt
        except Exception as e:
            self.get_logger().error(f'Error: {str(e)}')

    def timers_callback(self):
        self.keyboard_teleop()


def main(args=None):
    rclpy.init(args=args)
    node = TurtleTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()