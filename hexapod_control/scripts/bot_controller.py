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
            'Leg_1_coxa', 'Leg_1_tibia', 'Leg_1_femur', 
            'Leg_2_coxa', 'Leg_2_tibia', 'Leg_2_femur',
            'Leg_3_coxa', 'Leg_3_tibia', 'Leg_3_femur',
            'Leg_4_coxa', 'Leg_4_tibia', 'Leg_4_femur',
            'Leg_5_coxa', 'Leg_5_tibia', 'Leg_5_femur',
            'Leg_6_coxa', 'Leg_6_tibia', 'Leg_6_femur',
        ]

        point = JointTrajectoryPoint()
        point.positions = [0.5] * 18  # example positions
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
