#!/usr/bin/python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
import math,sys,termios,tty
from package_teleop_interfaces.srv import Notify

key_bindings = {
    'w': (1, 0),  # Move forward
    's': (-1, 0),  # Move backward
    'a': (0, 1),  # Turn left
    'd': (0, -1),  # Turn right
}

class TeleopkeyNode(Node):
    def __init__(self):
        super().__init__('teleopkey_node')
        """Declare Parameter"""

        """PUB"""
        self.custom_cmdvel_pub = self.create_publisher(Twist,'/custom_key/cmd_vel',10) 

        """VALUE"""
        self.linear_gain = 1.0
        self.angular_gain = 1.0
        self.vx = 1.5
        self.w = 2.5

        """Clinet"""
        self.reach_notify_pub_client = self.create_client(Notify,'/teleop_notiffy')

        """Timer"""
        self.create_timer(0.1,self.timers_callback)

        """Init Process"""
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
        self.custom_cmdvel_pub.publish(msg)

    def get_key(self):
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setraw(sys.stdin.fileno())
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        return key

    def keyboard_teleop(self):
        try:
            key = self.get_key()
            if key in key_bindings:
                linear = key_bindings[key][0] * self.linear_gain
                angular = key_bindings[key][1] * self.angular_gain
                self.cmdvel(self.vx*linear, self.w*angular)
            elif key == 'p':
                # print('press p key')
                self.client_reach_notify_pub(30)
            elif key == 'c':
                print('press c key')
            elif key == 'o':
                # print('press o key')
                 self.client_reach_notify_pub(31)
            elif key == ' ':
                self.cmdvel(0.0, 0.0)
            elif ord(key) == 3:
                print('Exiting...')
                raise KeyboardInterrupt
        except Exception as e:
            self.get_logger().error(f'Error: {str(e)}')

    def timers_callback(self):
        self.keyboard_teleop()

    def client_reach_notify_pub(self, command):
        command_request = Notify.Request()
        command_request.command = command
        future = self.reach_notify_pub_client.call_async(command_request)
        future.add_done_callback(lambda future: self.reach_notify_response_callback(future, command))

    def reach_notify_response_callback(self, future, command_input):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"Service call succeeded with command {command_input}: {response.message}")
            
            else:
                self.get_logger().warn(f"Service call failed with command {command_input}: {response.message}")
                
        except Exception as e:
            self.get_logger().error(f"Service call failed with error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TeleopkeyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()
