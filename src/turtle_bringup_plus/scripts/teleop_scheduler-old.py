#!/usr/bin/python3

from turtle_bringup_plus.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np
from turtlesim.msg import Pose
from std_srvs.srv import Empty
from std_msgs.msg import Int8
from package_teleop_interfaces.srv import Run , Path
import yaml


class teleop_scheduler(Node):
    def __init__(self):
        super().__init__('teleop_scheduler')
        self.create_subscription(Twist, '/turtle1/cmd_vel', self.cmdvel_callback, 10)
        self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.create_subscription(Int8, '/turtle1/pizza_count', self.pizza_count_callback, 10)
        
        self.num_pizza_spawn = 0
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.target_pub = self.create_publisher(Pose, 'target', 10)

        # service
        self.run_service = self.create_service(Run, '/run', self.run_callback)
        self.save_path_service = self.create_service(Path, '/save_path', self.save_to_yaml)

        self.target_pose = np.array([0.0, 0.0, 0.0]) 
        self.yaml_file_path = '/path/to/your/pizzapath.yaml'

    
        self.data = {'name': 'PizzaBot', 'pizzas_delivered': 12, 'target': [5.2, 3.1]}  # Random data


    def save_to_yaml(self, request, response):
        try:
            with open(self.yaml_file_path, 'w') as file:
                yaml.safe_dump(self.data, file)
            self.get_logger().info(f"Data successfully written to {self.yaml_file_path}")
            response.success = True
            response.message = 'Path saved successfully'
        except Exception as e:
            self.get_logger().error(f"Error writing to YAML file: {e}")
            response.success = False
            response.message = 'Failed to save path'

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
    def clear_path_callback(self):
        clear_request=Empty.Request()
        self.clear_path_client.call_async(clear_request)
    def pizza_count_callback(self, msg):
        pizza_count = self.pizza_eat # self.num_pizza_spawn is already an attribute, so no need to call it like a function
        print(pizza_count)
    def pizza_eat(self,response):
        """Clears in-memory data without saving to the YAML file."""
        try:
            # Clear the in-memory data (e.g., reset the target path)
            self.data['target'] = []  # Clear only the 'target' data in memory
            
            self.get_logger().info(f"In-memory data cleared (target reset), but not saved to YAML")
            response.success = True
            response.message = 'In-memory data cleared successfully'
        except Exception as e:
            self.get_logger().error(f"Error clearing in-memory data: {e}")
            response.success = False
            response.message = 'Failed to clear in-memory data'
    def pose_callback(self, msg):
        self.target_pose[0] = msg.x
        self.target_pose[1] = msg.y
        self.target_pose[2] = msg.theta

    def cmdvel_callback(self, msg):
        # Process the received Twist message (v = linear velocity, w = angular velocity)
        self.get_logger().info(f"Received Twist message: linear.x = {msg.linear.x}, angular.z = {msg.angular.z}")
        self.cmd_vel_pub.publish(msg)  # Publish the received command back to cmd_vel

def main(args=None):
    rclpy.init(args=args)
    node = teleop_scheduler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
