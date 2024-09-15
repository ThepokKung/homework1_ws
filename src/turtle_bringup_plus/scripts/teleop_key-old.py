#!/usr/bin/python3

# from package_name.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist,PoseStamped , Pose, PoseArray
from std_msgs.msg import Float64,Int8
import numpy as np
from package_teleop_interfaces.srv import Path , Run
import yaml,time,random,os
from std_srvs.srv import Empty
from turtlesim_plus_interfaces.srv import GivePosition


class teleop_key(Node):
    def __init__(self):
        super().__init__('teleop_key')
        # client
        self.save_path_client = self.create_client(Path,'save_path')
        self.clear_path_client = self.create_client(Empty,'clear_path')
        self.make_path_client = self.create_client(Path,'make_path')
        # publisher
        self.cmd_vel_publisher = self.create_publisher(Twist,'/turtle1/cmd_vel',10)

        # sub
        self.create_subscription(PoseStamped, '/turtle1/pose',self.pose_callback,10)
        self.create_subscription(Twist, '/turtle1/cmd_vel', self.cmdvel_callback,10)

        # sevice
        self.save_path_cli = self.create_service(Path,'save_path',self.save_path_callback)
        self.clear_path_server = self.create_service(Empty,'clear_path',self.clear_path_callback)

        self.declare_parameter('num_targets', 5)  # Number of random targets
        self.declare_parameter('target_min', 0)  # Min target (use int)
        self.declare_parameter('target_max', 360)  # Max target (use int)

        self.create_subscription(Pose, 'pose',self.pose_callback ,10)

        self.robot_pose = np.array([0.0,0.0,0.0]) 

    def cmdvel_callback(self, msg):
        # Process the received Twist message (v = linear velocity, w = angular velocity)
        self.get_logger().info(f"Received Twist message: linear.x = {msg.linear.x}, angular.z = {msg.angular.z}")
        self.cmd_vel_pub.publish(msg)  # Publish the received command back to cmd_vel

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
    def save_path_callback(self , request:Path.Request,response:Path.Response):
        if request.command == 1:
            self.state = 1
            
        self.save_path = request.save_path.data

    def clear_path_callback(self):
        clear_request=Empty.Request()
        self.clear_path_client.call_async(clear_request)

    def cmdvel(self, v ,w ):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.cmd_vel_publisher.publish(msg)
        
    def pose_callback(self,msg):
        if (self.read_position == 1):
            self.robot_pose[0] = msg.pose.position.x
            self.robot_pose[1] = msg.pose.position.y
            self.robot_pose[2] = msg.pose.position.theta
            self.read_position = 0
        else:
            pass
    def generate_random_targets(self):
        # Generate a list of random integer target positions
        targets = [random.randint(self.target_min, self.target_max) for _ in range(self.num_targets)]
        return targets

    # def save_to_yaml(self, targets):
    #     # Define the path to the YAML file
    #     yaml_file_path = os.path.expanduser(self.file_yaml_path)

    #     # Structure to save in YAML format
    #     data = {'targets': targets}

    #     # Write to the YAML file
    #     try:
    #         with open(yaml_file_path, 'w') as file:
    #             yaml.dump(data, file)
    #         self.get_logger().info(f"Targets saved to {yaml_file_path}")
    #     except Exception as e:
    #         self.get_logger().error(f"Failed to save targets to YAML: {e}")

    def send_heartbeat(self):
        msg = Int8()
        msg.data = 9
        self.notify_pub.publish(msg)
        self.get_logger().info('Sent heartbeat: 9')

    def Notify_Callback(self, msg):
        # Check for the value 10 to stop the node
        if msg.data == 10:
            self.get_logger().info('Received 10, ready to shut down node.')
            self.state = 1  # Change the state to 1 to stop sending heartbeats
def main(args=None):
    rclpy.init(args=args)
    node = teleop_key()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
