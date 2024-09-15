#!/usr/bin/python3

import rclpy
from rclpy.node import Node

import numpy as np
import os,yaml
from ament_index_python.packages import get_package_share_directory
from package_teleop_interfaces.msg import Goaltarget
from package_teleop_interfaces.srv import Run,Notify,NotifyEmpty
from std_msgs.msg import Float64

"""
State Note
0 is not ready
1 is ready
2 is wait for command by controller
3 is Ouf of target
"""

class CopyScheduleNode(Node):
    def __init__(self):
        super().__init__('copyschedule_node')
        """Declare Parameter"""
        self.declare_parameter('turtltsim_namespace', 'test')
        self.declare_parameter('turtle_name','turtle_bibo')
        self.declare_parameter('my_copy_index', 1)  # Declare parameter with default value

        """Get value"""
        self.my_copy_index = self.get_parameter('my_copy_index').value
        self.turtle_name = self.get_parameter('turtle_name').get_parameter_value().string_value
        self.my_copy_index = self.get_parameter('my_copy_index').get_parameter_value().integer_value

        """PUB"""
        self.target_pub_client = self.create_publisher(Goaltarget,f'{self.get_namespace()}/{self.turtle_name}/target',10)

        """YAML PATH"""
        self.yaml_file_path = os.path.join(
            get_package_share_directory('turtle_bringup_plus'),
            'config',
            'pizzapath.yaml'
        )

        """VALUE"""
        self.target = self.Read_target_from_yaml()
        self.state = 89 
        self.run = False
        self.target_index = 0
        self.idcheck = np.array([False,False,False,False])
        # IDK WTF
        self.last_data = np.array([11.0,11.0])

        """Service"""
        self.run_service = self.create_service(Run, '/run', self.run_callback) #Run Service
        self.notify_service_server = self.create_service(Notify,f'{self.get_namespace()}/{self.turtle_name}/reach_notiffy',self.service_reach_notify_callback)
        self.empty_notify_server = self.create_service(NotifyEmpty,f'{self.get_namespace()}/empty_notiffy',self.service_emptynotiffy_callback)

        """Clinet"""
        self.empty_notify_pub_client = self.create_client(NotifyEmpty,f'{self.get_namespace()}/empty_notiffy')

        """Start node text"""
        self.get_logger().info(f'Starting {self.get_namespace()}/{self.get_name()}') 
        
    def Read_target_from_yaml(self):
        # Read the YAML file
        try:
            with open(self.yaml_file_path, 'r') as file:
                data = yaml.safe_load(file)
                target_data = data.get(f'save_count_{self.my_copy_index}', [])
                return target_data
        except Exception as e:
            self.get_logger().error(f"Failed to load targets from YAML: {e}")
            return []

    def Tartget_pub(self,target_pose):
        msg = Goaltarget()
        msg.x.data = target_pose[0]
        msg.y.data = target_pose[1]
        self.target_pub_client.publish(msg)
        self.get_logger().info(f'{self.get_namespace()}/{self.get_name()} is send target')

    def run_callback(self, request, response):
        # Service callback to handle the `/run` service
        if request.command == 1:
            self.state = 1  # Set state to ready
            self.run = True
            self.get_logger().info(f"Scheduler started. Current state: {self.state}")
            response.success = True
            response.message = 'Scheduler started'
        elif request.command == 2:
            self.state = 0  # Set state to not ready
            self.run = False
            self.get_logger().info(f"Scheduler stopped. Current state: {self.state}")
            response.success = True
            response.message = 'Scheduler stopped'
        else:
            response.success = False
            response.message = 'Invalid command'
            self.get_logger().warn('Received invalid /run command')
        return response
    
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
            # self.get_logger().info(f"{self.idcheck}")
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

    def service_emptynotiffy_callback(self,request, response):
        if request.command == 1:
            if request.id < len(self.idcheck):
                self.idcheck[request.id] = True
                response.success = True
                response.message = f"ID : {request.id} check "
            else :
                response.success = False
                response.message = f"ID Out of range"
        else:
            response.success = False
            response.message = "Command don't fount"
        return response
    
    def clinet_empty_notift_pub(self,id,command):
        command_request = NotifyEmpty.Request()
        command_request.id = id
        command_request.command = command
        self.empty_notify_pub_client.call_async(command_request)

    def checkallready(self):
        if self.idcheck.all():
            pass
        pass

def main(args=None):
    rclpy.init(args=args)
    node = CopyScheduleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
