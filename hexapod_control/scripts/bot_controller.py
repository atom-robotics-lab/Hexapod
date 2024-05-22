#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import sys

class Mynode(Node):
    def __init__(self, target_positions, velocities):
        super().__init__("position_subscriber")
        self.get_logger().info("Position Subscriber started")
        
        self.joint_array = [
            'Revolute_63', 'Revolute_64', 'Revolute_65', 'Revolute_69', 
            'Revolute_70', 'Revolute_71', 'Revolute_72', 'Revolute_73', 
            'Revolute_74', 'Revolute_75', 'Revolute_76', 'Revolute_77', 
            'Revolute_78', 'Revolute_79', 'Revolute_80', 'Revolute_81', 
            'Revolute_82', 'Revolute_83'
        ]

        self.target_positions = target_positions  
        self.velocities = velocities 
        
        self.pose_subscriber = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_callback,
            10
        )

        self.cmd_vel_pub_ = self.create_publisher(Float64MultiArray, "/hexapod_controller/commands", 10)

    def joint_callback(self, msg: JointState):
        self.send_velocities = Float64MultiArray()
        self.send_velocities.data = [0.0] * 18

        for i in range(len(msg.position)):
            joint_name = msg.name[i]
            if joint_name in self.joint_array:
                joint_index = self.joint_array.index(joint_name)
                current_position = msg.position[i]
                target_position = self.target_positions[joint_index]
                velocity = self.velocities[joint_index]

                if current_position < target_position:
                    self.send_velocities.data[joint_index] = velocity  
                elif current_position > target_position:
                    self.send_velocities.data[joint_index] = -velocity  
                else:
                    self.send_velocities.data[joint_index] = 0.0 

        self.get_logger().info('Publishing: %s' % self.send_velocities.data)
        self.cmd_vel_pub_.publish(self.send_velocities)

def main(args=None):
    rclpy.init(args=args)

    if args is None:
        args = sys.argv

    target_positions = [0.0] * 18  
    velocities = [0.1] * 18  

    
    if len(args) > 1:
        target_positions = list(map(float, args[1:19]))
        velocities = list(map(float, args[19:37]))

    node = Mynode(target_positions, velocities)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
