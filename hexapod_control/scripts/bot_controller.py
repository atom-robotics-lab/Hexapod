#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys

class JointPositions:
    def __init__(self):
        self.Leg_1_coxa = 0.0
        self.Leg_1_tibia = 0.0
        self.Leg_1_femur = 0.0
        self.Leg_2_coxa = 0.0
        self.Leg_2_tibia = 0.0
        self.Leg_2_femur = 0.0
        self.Leg_3_coxa = 0.0
        self.Leg_3_tibia = 0.0
        self.Leg_3_femur = 0.0
        self.Leg_4_coxa = 0.0
        self.Leg_4_tibia = 0.0
        self.Leg_4_femur = 0.0
        self.Leg_5_coxa = 0.0
        self.Leg_5_tibia = 0.0
        self.Leg_5_femur = 0.0
        self.Leg_6_coxa = 0.0
        self.Leg_6_tibia = 0.0
        self.Leg_6_femur = 0.0

    def get_positions(self):
        return [
            self.Leg_1_coxa, self.Leg_1_tibia, self.Leg_1_femur,
            self.Leg_2_coxa, self.Leg_2_tibia, self.Leg_2_femur,
            self.Leg_3_coxa, self.Leg_3_tibia, self.Leg_3_femur,
            self.Leg_4_coxa, self.Leg_4_tibia, self.Leg_4_femur,
            self.Leg_5_coxa, self.Leg_5_tibia, self.Leg_5_femur,
            self.Leg_6_coxa, self.Leg_6_tibia, self.Leg_6_femur,
        ]

class SimpleTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('simple_trajectory_publisher')
        self.publisher_ = self.create_publisher(JointTrajectory, '/hexapod_controller/joint_trajectory', 10)
        self.joint_positions = JointPositions()
        self.timer_callback()
        self.get_logger().info('Simple Trajectory Publisher has been started.')

    def create_trajectory_point(self, positions, sec, nanosec=0):
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = sec
        point.time_from_start.nanosec = nanosec
        return point

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

        # Creating trajectory points using the helper function
        self.joint_positions.Leg_2_tibia=0.2
        point1 = self.create_trajectory_point(self.joint_positions.get_positions(), 2)
        
       
        traj_msg.points.append(point1)

        

        self.publisher_.publish(traj_msg)
        self.get_logger().info('Publishing trajectory: %s' % traj_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleTrajectoryPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
