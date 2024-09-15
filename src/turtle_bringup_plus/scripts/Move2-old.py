#!/usr/bin/python3

import rclpy
import sys
import termios
import tty
import numpy as np
import time
import math
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_srvs.srv import Empty
from turtlesim.msg import Pose
from turtlesim_plus_interfaces.srv import GivePosition

key_bindings = {
    'w': (1, 0),  # Move forward
    's': (-1, 0),  # Move backward
    'a': (0, 1),  # Turn left
    'd': (0, -1),  # Turn right
}

class Move2(Node):
    def __init__(self):
        super().__init__('Move')
        self.frequency = 100

        self.linear_gain = 1.0
        self.angular_gain = 1.0
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.eat_pizza_client = self.create_client(Empty,'eat')
        self.spawn_pizza_cilent = self.create_client(GivePosition, '/spawn_pizza')

        self.create_timer(1.0 / self.frequency, self.timers_callback)
        self.robot_pose = np.array([0.0, 0.0, 0.0])
        self.pizza_pose = np.array([0.0, 0.0, 0.0])
        self.print_info()

    def print_info(self):
        print('Control Your TurtleSim')
        print('----------------------')
        print('Move around:')
        print('          w          ')
        print('    a     s     d    ')
        print(' ')
        print('p: spawn pizza')
        print('c: clear pizza')
        print('o: save path')


    def cmdvel(self, v, w):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.cmd_vel_pub.publish(msg)

    def pose_callback(self, msg):
        self.robot_pose[0] = msg.x
        self.robot_pose[1] = msg.y
        self.robot_pose[2] = msg.theta

    def spawn_pizza(self, x, y,msg):
        self.robot_pose[0] = msg.x
        self.robot_pose[1] = msg.y
        self.spawn_pizza_cilent.call_async(self.turtle_pose[0],self.turtle_pose[1])

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

    def eat_pizza(self):
        eat_request = Empty.Request()
        self.eat_pizza_client.call_async(eat_request)

    def keyboard_teleop(self,v,w):
        try:
            key = self.get_key()
            if key in key_bindings:
                linear = key_bindings[key][0] * self.linear_gain
                angular = key_bindings[key][1] * self.angular_gain
                self.cmdvel(v*linear, w*angular)
            elif key == 'p':
                self.spawn_pizza(self.robot_pose[0], self.robot_pose[1])
            elif key == ' ':
                self.cmdvel(0.0, 0.0)
            elif ord(key) == 3:
                print('Exiting...')
                raise KeyboardInterrupt
        except Exception as e:
            self.get_logger().error(f'Error: {str(e)}')

    def timers_callback(self):
        kpd = 2.5
        kpw = 5
        
        deltax = self.pizza_pose[0] - self.robot_pose[0]
        deltay = self.pizza_pose[1] - self.robot_pose[1]
        d = math.sqrt(deltax**2 + deltay**2)
        alpha = math.atan2(deltay, deltax)
        temp_e_theta = alpha - self.robot_pose[2] # Change here 
        e_theta = math.atan2(math.sin(temp_e_theta),math.cos(temp_e_theta))
        vx = kpd * (math.sqrt(deltax**2 + deltay**2))
        w = kpw * e_theta
        # self .cmdvel (vx ,w)
        if(d<0.5):
            self.eat_pizza()
            self.read_position = 1
        self.keyboard_teleop(vx,w)


def main(args=None):
    rclpy.init(args=args)
    node = Move2()
    #
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()