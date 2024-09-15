#!/usr/bin/python3

import rclpy
from rclpy.node import Node

import numpy as np
import math
from std_msgs.msg import Int8
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from package_teleop_interfaces.msg import Goaltarget
from package_teleop_interfaces.srv import Notify
from turtlesim_plus_interfaces.srv import GivePosition

"""
State Note
0 is not Ready
1,4 is Ready
2,5 is Working
3,6 is done want to next target
"""

class Controllerode(Node):
    def __init__(self):
        super().__init__('controller_node')
        """Declare Parameter"""
        self.declare_parameter('turtltsim_namespace', 'test')
        self.declare_parameter('turtle_name','turtle_bibo')
        self.declare_parameter('isteleop',False)

        """Get value"""
        self.turtle_name = self.get_parameter('turtle_name').get_parameter_value().string_value
        self.turtltsim_namespace = self.get_parameter('turtltsim_namespace').get_parameter_value().string_value
        self.isteleop = self.get_parameter('isteleop').get_parameter_value().bool_value

        """SUB"""
        self.create_subscription(Pose,f'{self.get_namespace()}/{self.turtle_name}/pose',self.pose_callback,10) 
        self.create_subscription(Goaltarget,f'{self.get_namespace()}/{self.turtle_name}/target',self.Target_callback,10)
        
        """PUB"""
        self.cmdvel_pub = self.create_publisher(Twist,f'{self.get_namespace()}/{self.turtle_name}/cmd_vel',10) 

        """Value"""
        self.robot_pose = Pose()
        self.state = 0
        self.target = np.array([0.0,0.0]) # X, Y
        self.outoftarget = 0
        self.kp_linear = 2.5
        self.kp_angular = 5

        """Timer"""
        self.timer = self.create_timer(0.1,self.timer_callback) #Timer

        """Client"""
        self.spawm_pizza_client = self.create_client(GivePosition,f'{self.get_namespace()}/spawn_pizza') #Spawn pizza
        self.reach_notify_pub_client = self.create_client(Notify,f'{self.get_namespace()}/{self.turtle_name}/reach_notiffy')
        
        """For Test"""
        self.test_counter = 5

        """Start node text"""
        self.get_logger().info(f'Starting {self.get_namespace()}/{self.get_name()}') 

    def pose_callback(self,msg):
        self.robot_pose.x = msg.x 
        self.robot_pose.y = msg.y 
        self.robot_pose.theta = msg.theta

    def Target_callback(self,msg):
        if self.state == 1:
            self.target[0] = msg.x.data
            self.target[1] = msg.y.data 
            self.state = 2 
        else:
            pass

    def cmdvel(self, v, w):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.cmdvel_pub.publish(msg)
        
    def spawn_pizza(self,x,y):
        position_request = GivePosition.Request()
        position_request.x = x
        position_request.y = y
        self.spawm_pizza_client.call_async(position_request)

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
                if command_input == 4 and self.state == 0:
                    """ Set to state ready """
                    self.state = 1
                elif command_input == 5 and self.state == 1:
                    pass
                elif command_input == 16 and self.state == 0:
                    self.state = 8
                    self.outoftarget = 0
            else:
                self.get_logger().warn(f"Service call failed with command {command_input}: {response.message}")
                if command_input == 6 :
                    self.outoftarget = 1
                    self.client_reach_notify_pub(15)
                elif command_input == 5:
                    self.outoftarget = 1
                    self.client_reach_notify_pub(15)
        except Exception as e:
            self.get_logger().error(f"Service call failed with error: {e}")

    def checkoutofindex(self):
        if self.state == 1:
            self.outoftarget = 1
        
    def timer_callback(self):
        if self.state == 8 :
            self.target[0] = 10.0
            self.target[1] = 10.0
    
        if self.outoftarget == 1:
            self.cmdvel(0.0,0.0)
            self.client_reach_notify_pub(16)
            self.get_logger().info(f'Tell me state : {self.state}')
        else:
            if self.state == 0:
                """ Want schedule is ready """
                self.client_reach_notify_pub(4)
            elif self.state == 1:
                """Want tatget to go"""
                if self.isteleop == True:
                    self.state = 9
                else:
                    self.client_reach_notify_pub(5)
                    self.get_logger().info(f"Node : {self.get_namespace()}/{self.get_name()} is Ready need for target")
            # elif 
            else:
                if self.isteleop == True and self.state == 9:
                    pass
                else:
                    delta_x = self.target[0] - self.robot_pose.x #delta X position (XG - XT)
                    delta_y = self.target[1] - self.robot_pose.y #delta Y position (YG - YT)

                    d = math.sqrt(delta_x**2 + delta_y**2)

                    alpha = math.atan2(delta_y, delta_x) # Alpha in radians
                    temp_e_theta = alpha - self.robot_pose.theta # Change here 
                    e_theta = math.atan2(math.sin(temp_e_theta),math.cos(temp_e_theta))

                    vx = self.kp_linear * d
                    w = self.kp_angular * e_theta

                    self.cmdvel(vx,w)
                    if (d < 0.1) and self.state == 2:
                        self.spawn_pizza(self.target[0],self.target[1])
                        self.state = 0
                        self.client_reach_notify_pub(6)

def main(args=None):
    rclpy.init(args=args)
    node = Controllerode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
