#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math

class Hexapod(Node):
    def __init__(self):
        super().__init__('Hexapod')
        self.publisher_ = self.create_publisher(JointTrajectory, '/hexapod_controller/joint_trajectory', 10)
        timer_period = 0.2  # seconds
        self.timer_ = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Simple Trajectory Publisher has been started.')
        self.cycle_index = 0  # To track the current index in the walking cycle
        self.walk_scale = 0.1

    def clamp(self, value, min_value, max_value):
        return max(min(value, max_value), min_value)

    def inverse_kinematics(self, x, y, z, l1, l2, l3):
        """
            x, y, z: Desired foot position
            l1: Length of the coxa segment
            l2: Length of the femur segment
            l3: Length of the tibia segment
            theta1: Angle of the coxa joint
            theta2: Angle of the femur joint
            theta3: Angle of the tibia joint
        """
        epsilon = 1e-6 
        theta1 = math.atan2(y, x) 
        print()
        print("theta1",theta1)

        xy_dist = math.sqrt(x**2 + y**2) - l1
        print()
        print(xy_dist)
        
        d = math.sqrt(xy_dist**2 + z**2) + epsilon  #coxa to desired position
        
        cos_theta3 = (l2**2+l3**2+d**2) / (2 * l2 * l3)
        cos_theta3 = self.clamp(cos_theta3, -1.0, 1.0)
        theta3 = math.acos(cos_theta3)
        print()
        print("theta3",theta3)
        
        cos_theta2 = (d**2 + l2**2 - l3**2) / (2 * d * l2)

        theta2 = math.atan2(z, xy_dist) - math.acos(cos_theta2)
        print()
        print("theta2",theta2)
        
        return theta1, theta2, theta3

    def timer_callback(self):
        traj_msg = JointTrajectory()
        traj_msg.joint_names = [
            'Leg_1_coxa', 'Leg_1_femur', 'Leg_1_tibia',   
            'Leg_2_coxa', 'Leg_2_femur', 'Leg_2_tibia',   
            'Leg_3_coxa', 'Leg_3_femur', 'Leg_3_tibia',   
            'Leg_4_coxa', 'Leg_4_femur', 'Leg_4_tibia',   
            'Leg_5_coxa', 'Leg_5_femur', 'Leg_5_tibia',   
            'Leg_6_coxa', 'Leg_6_femur', 'Leg_6_tibia',   
        ]

        tripod_gait_cycle = [
            [(0.0, -5.0, -0.5), (0.0, -5.0, 0.0), (0.0, -5.0, -0.5), (0.0, 5.0, 0.0), (0.0, 5.0, -0.5), (0.0, 5.0, 0.0)],
            [(0.0, -5.0, 0.5), (0.0, 5.0, 0.0), (0.0, -5.0, 0.5), (0.0, -5.0, 0.0), (0.0, 5.0, 0.5), (0.0, -5.0, 0.0)],
            [(0.0, -5.0, 0.5), (0.0, 5.0, 0.0), (0.0, -5.0, 0.5), (0.0, -5.0, 0.0), (0.0, 5.0, 0.5), (0.0, -5.0, 0.0)],

            [(0.0, -5.0, 0.0), (0.0, -5.0, -0.5), (0.0, -5.0, 0.0), (0.0, 5.0, -0.5), (0.0, 5.0, 0.0), (0.0, 5.0, -0.5)],
            [(0.0, 5.0, 0.0), (0.0, -5.0, 0.5), (0.0, 5.0, 0.0), (0.0, 5.0, 0.5), (0.0, -5.0, 0.0), (0.0, 5.0, 0.5)],
            [(0.0, 5.0, 0.0), (0.0, -5.0, 0.5), (0.0, 5.0, 0.0), (0.0, 5.0, 0.5), (0.0, -5.0, 0.0), (0.0, 5.0, 0.5)],
        ]

        point = JointTrajectoryPoint()
        segment_lengths = [105.0, 91.0, 100.0]  # Example segment lengths for coxa, femur, tibia
        positions = []
        for pos in tripod_gait_cycle[self.cycle_index % len(tripod_gait_cycle)]:
            print()
            print(pos)
            print()
            x, y, z = pos
            angles = self.inverse_kinematics(x, y, z, *segment_lengths)
            angles = [angle * self.walk_scale for angle in angles]
            positions.extend(angles)

        point.positions = positions
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 200000000  

        traj_msg.points.append(point)

        self.publisher_.publish(traj_msg)
        self.get_logger().info(f'Publishing trajectory: {traj_msg}')
        self.cycle_index += 1  

def main(args=None):
    rclpy.init(args=args)
    node = Hexapod()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
