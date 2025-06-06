import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from std_srvs.srv import Empty
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
import math
import sys

class Navigator(Node):

    goal_list = [
        [-8.0,-6.0,0.0],
        [-8.0,-6.0,math.radians(90)],
    ]

    def __init__(self):
        super().__init__('navigator')
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.start_server = self.create_service(
            Empty, 
            "start", 
            self.start_callback
        )
        self.initpose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, 
            '/initialpose', 
            10
        )

        self.goal_num = 0
        self.executing = False

    def set_initial_pose(self, x, y, yaw):
        msg = PoseWithCovarianceStamped()
        
        # Set header
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        # Set position
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        
        # Set orientation (convert yaw to quaternion)
        q = self.euler_to_quaternion(0, 0, yaw)
        msg.pose.pose.orientation.x = q['x']
        msg.pose.pose.orientation.y = q['y']
        msg.pose.pose.orientation.z = q['z']
        msg.pose.pose.orientation.w = q['w']
        
        # Set covariance (36 element array)
        # Values represent uncertainty in x, y, z, roll, pitch, yaw
        msg.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787
        ]
        
        self.initpose_publisher.publish(msg)
        self.get_logger().info(f'Published initial pose: x={x}, y={y}, yaw={yaw}')

    def send_goal(self, x, y, yaw):
        self.get_logger().info('Sending goal...')
        goal_msg = NavigateToPose.Goal()
    
        
        # Set position
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        
        # Set orientation (convert yaw to quaternion)
        q = self.euler_to_quaternion(0, 0, yaw)
        goal_msg.pose.pose.orientation.x = q['x']
        goal_msg.pose.pose.orientation.y = q['y']
        goal_msg.pose.pose.orientation.z = q['z']
        goal_msg.pose.pose.orientation.w = q['w']

        self.action_client.wait_for_server()

        self.send_goal_future = self.action_client.send_goal_async(
            goal_msg,
        )
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Objetivo rejeitado")
            self.rotation_in_progress = False
            return

        self.get_logger().info("Objetivo aceito")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        self.get_logger().info(f"Resultado: {result}; Status: {status}")
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.goal_num+=1
            if(self.goal_num>=len(self.goal_list)):
                self.goal_num = 0
                self.executing = False
        else:
            self.get_logger().error("Objetivo não atingido, tentando novamente...")
        self.exec_goal()


    def start_callback(self, request, response):
        self.get_logger().info("Iniciando movimento")
        self.executing = True
        self.exec_goal()
        return response
    
    def exec_goal(self):
        if(self.executing):
            self.get_logger().info(f"Goal counter: {self.goal_num}")
            x,y, yaw = Navigator.goal_list[self.goal_num]
            self.get_logger().info(f"Enviando objetivo: x = {x}; y = {y}; yaw={yaw}")
            self.send_goal(x, y, yaw)

    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles to quaternion"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = {}
        q['w'] = cr * cp * cy + sr * sp * sy
        q['x'] = sr * cp * cy - cr * sp * sy
        q['y'] = cr * sp * cy + sr * cp * sy
        q['z'] = cr * cp * sy - sr * sp * cy

        return q


    

def main(args=None):
    rclpy.init(args=args)
    nav = Navigator()
    #action_client.send_goal(float(sys.argv[1]), float(sys.argv[2]))
    nav.set_initial_pose(float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]))
    rclpy.spin(nav)