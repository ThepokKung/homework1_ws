#!/usr/bin/python3

import rclpy
from rclpy.node import Node

import numpy as np
import os,yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
# from std_srvs.srv import Empty
from std_msgs.msg import Int64
from package_teleop_interfaces.srv import Run,SavePath,Notify,Pizzanum
from turtlesim_plus_interfaces.srv import GivePosition

class teleop_scheduler(Node):
    def __init__(self):
        super().__init__('teleop_scheduler')
        """Declare Parameter"""
        self.declare_parameter('turtle_name','turtle_bibo')
        self.declare_parameter('pizza_num',10)
        
        """Get value"""
        self.turtle_name = self.get_parameter('turtle_name').get_parameter_value().string_value
        self.pizza_num = self.get_parameter('pizza_num').get_parameter_value().integer_value

        """SUB"""
        self.create_subscription(Twist, '/custom_key/cmd_vel', self.custom_cmdvel_callback, 10)
        self.create_subscription(Pose, f'{self.get_namespace()}/{self.turtle_name}/pose', self.pose_callback, 10)
        self.create_subscription(Int64, f'{self.get_namespace()}/{self.turtle_name}/pizza_count', self.pizza_count_callback, 10)
        
        """PUB""" 
        self.cmd_vel_pub = self.create_publisher(Twist, f'{self.get_namespace()}/{self.turtle_name}/cmd_vel', 10)
        self.target_pub = self.create_publisher(Pose, f'{self.get_namespace()}/{self.turtle_name}/target', 10)

        """VAlUE"""
        self.state = 0
        self.target_pose = np.array([0.0, 0.0, 0.0])
        self.save_count = 1
        self.run = True
        self.pizza_spawn_count = 0
        self.temp_data = []
        self.temp_data_target = []

        """service"""
        self.run_service = self.create_service(Run, '/run', self.run_callback)
        self.services_save_path = self.create_service(SavePath,'/save_path',self.save_path_callback)
        self.services_pizzanum_server = self.create_service(Pizzanum,f'{self.get_namespace()}/pizza_num',self.pizza_num_callback)
        self.notify_service_server = self.create_service(Notify,f'{self.get_namespace()}/{self.turtle_name}/reach_notiffy',self.service_reach_notify_callback)
        self.teleop_notify_service_server = self.create_service(Notify,'/teleop_notiffy',self.service_teleop_notify_callback)

        """Clinet"""
        self.spawm_pizza_client = self.create_client(GivePosition,f'{self.get_namespace()}/spawn_pizza')
        self.save_path_client = self.create_client(SavePath,'/save_path')

        """YAML PATH"""
        self.yaml_file_path = os.path.join(
            get_package_share_directory('turtle_bringup_plus'),
            'config',
            'pizzapath.yaml'
        )
    
    def save_to_yaml(self):
        data = {f'save_count_{self.save_count}': self.temp_data_target}
        try:
            with open(self.yaml_file_path, 'w') as file:
                yaml.dump(data, file)
            self.get_logger().info(f"Targets saved to {self.yaml_file_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to save targets to YAML: {e}")

    def pizza_num_callback(self, request, response):
        if request.pizzanum > self.pizza_spawn_count:
            self.pizza_num = request.pizzanum
            response.success = True
            response.message = 'Add more pizza'
            self.get_logger().info(f"Now pizza num is {self.pizza_num}")
        else:
            response.success = False
            response.message = 'Pizza cant reduce'
        return response

    def run_callback(self, request, response):
        if request.command == 1:
            self.state = 1
            self.get_logger().info(f"Scheduler started. Current state: {self.state}")
            response.success = True
            response.message = 'teleop_scheduler Started'
        elif request.command == 2:
            self.state = 0
            self.get_logger().info(f"Scheduler stopped. Current state: {self.state}")
            response.success = True
            response.message = 'teleop_scheduler Stopped'
        else:
            response.success = False
            response.message = 'Invalid command'
            self.get_logger().warn('Received invalid /run command')
        return response
    
    def save_path_callback(self, request, response):
        if request.save:
            if self.save_count <= 4:
                self.save_to_yaml()
                self.save_count += 1
                response.success = True
                response.message = f'save data_count: {self.save_count}'
            else:
                response.success = False
                response.message = 'data out of range'
        else:
            response.success = False
            response.message = 'something wrong'
        return response

    def pizza_count_callback(self, msg):
        print(msg)

    def spawn_pizza(self):
        position_request = GivePosition.Request()
        position_request.x = self.target_pose[0]
        position_request.y = self.target_pose[1]
        self.spawm_pizza_client.call_async(position_request)

         # Save the current pizza position to the temporary data
        self.temp_data_target.append([self.target_pose[0], self.target_pose[1]])
        self.get_logger().info(f"Pizza spawned at: {self.target_pose[0]}, {self.target_pose[1]}")
        self.get_logger().info(f'My temp file : {self.temp_data}')

    def save_path_pub(self):
        # Create a request for the SavePath service
        msg = SavePath.Request()
        msg.save = True
        
        self.save_path_client.call_async(msg)

    def pose_callback(self, msg):
        self.target_pose[0] = msg.x
        self.target_pose[1] = msg.y
        self.target_pose[2] = msg.theta
        # self.get_logger().info(f'Check pose callback : {self.target_pose}')

    def custom_cmdvel_callback(self, msg):
        # Process the received Twist message (v = linear velocity, w = angular velocity)
        self.get_logger().info(f"Received Twist message: linear.x = {msg.linear.x}, angular.z = {msg.angular.z}")
        self.cmd_vel_pub.publish(msg)  # Publish the received command back to cmd_vel

    def cmdvel_pub(self,data_vw):
        msg = Twist()
        msg = data_vw
        self.cmd_vel_pub.publish(msg)

    def service_reach_notify_callback(self, request, response):
        if request.command == 4:
            """Set state to ready"""
            if self.run == True:    
                self.state = 1
                response.success = True
                response.message = 'Scheduler Ready'
                return response
            else:
                response.success = False
                response.message = 'Scheduler not Ready'
                return response
        elif request.command == 5 and self.state == 1:
            if self.target_index >= len(self.target):
                self.state = 3
                response.success = False
                response.message = 'Scheduler out of index'
                return response
            else:
                self.Tartget_pub(self.target[self.target_index])
                self.state = 2
                response.success = True
                response.message = 'Scheduler Get postition'
                return response
        elif request.command == 6 and self.state == 2:
            """Switct to next target"""
            self.target_index += 1
            if self.target_index >= len(self.target):
                """OUT OF INDEX"""
                self.state = 3
                response.success = False
                response.message = 'Scheduler out of index'
                return response
            else:
                """HAVE ANY VALUE"""
                self.state = 1 
                response.success = True
                response.message = 'Scheduler Getnext index'
                return response
        elif request.command == 15:
            self.clinet_empty_notift_pub(self.my_copy_index-1,1)
            self.checkallready()
            response.success = True
            response.message = 'Scheduler Check your id'
        elif request.command == 16:
            if self.idcheck.all():
                self.Tartget_pub(self.last_data)
                response.success = True
                response.message = 'Scheduler tell you time to go F'
                self.get_logger().info("It's on com 16")
        elif request.command == 20 :
            """WAKE UP BRO"""
            self.state = 1
            response.success = True
            response.message = 'Scheduler Wake up'
        self.get_logger().info(f"Node : {self.get_namespace()}/{self.get_name()} state is {self.state}")
        return response
        
    def service_teleop_notify_callback(self, request, response):
        if request.command == 30:
            """GIVE PIZZA"""
            if (self.pizza_num-self.pizza_spawn_count) > 0:
                self.spawn_pizza()
                self.pizza_spawn_count += 1
                response.success = True
                response.message = 'Spawn Pizza'
            else:
                response.success = False
                response.message = 'Out of pizza'
            self.get_logger().info(f"Spawn Pizza and pizza num : {self.pizza_num-self.pizza_spawn_count}")
        elif request.command == 31:
            """Save Path"""
            self.save_path_pub()
            # self.save_to_yaml()
            self.get_logger().info("I want to think it's save")
            """ROLBACK HERE"""
        
        return response
    
def main(args=None):
    rclpy.init(args=args)
    node = teleop_scheduler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
