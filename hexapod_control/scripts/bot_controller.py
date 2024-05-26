#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys

class SimpleTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('simple_trajectory_publisher')
        self.publisher_ = self.create_publisher(JointTrajectory, '/hexapod_controller/joint_trajectory', 10)
        timer_period = 1.0  # seconds
        self.timer_callback()
        self.get_logger().info('Simple Trajectory Publisher has been started.')

    def timer_callback(self):
        traj_msg = JointTrajectory()
        traj_msg.joint_names = [
            'Revolute_63', 'Revolute_64', 'Revolute_65', 'Revolute_69',
            'Revolute_70', 'Revolute_71', 'Revolute_72', 'Revolute_73',
            'Revolute_74', 'Revolute_75', 'Revolute_76', 'Revolute_77',
            'Revolute_78', 'Revolute_79', 'Revolute_80', 'Revolute_81',
            'Revolute_82', 'Revolute_83'
        ]

        point = JointTrajectoryPoint()
        point.positions = [0.0] * 18  # example positions
        point.time_from_start.sec =  0 # 2 seconds to reach the target
        point.time_from_start.nanosec = 500

        traj_msg.points.append(point)

        self.publisher_.publish(traj_msg)
        self.get_logger().info('Publishing trajectory: %s' % traj_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleTrajectoryPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
