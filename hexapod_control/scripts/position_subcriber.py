#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist

class Mynode(Node):
    def __init__(self):
        
        super().__init__("position_subscriber")
        self.get_logger().info("Position Subscriber started")
        self.pose_subscriber = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.joint_callback,
            10  # Adjusted the queue size to 10
        )

    def joint_callback(self, msg: Twist):
        self.get_logger().info(f'Received message: linear={msg.linear} angular={msg.angular}')

def main(args=None):
    rclpy.init(args=args)
    node = Mynode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()