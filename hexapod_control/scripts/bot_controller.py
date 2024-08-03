#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math
import time

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
            self.Leg_1_coxa, self.Leg_1_femur, self.Leg_1_tibia, 
            self.Leg_2_coxa, self.Leg_2_femur, self.Leg_2_tibia, 
            self.Leg_3_coxa, self.Leg_3_femur, self.Leg_3_tibia, 
            self.Leg_4_coxa, self.Leg_4_femur, self.Leg_4_tibia, 
            self.Leg_5_coxa, self.Leg_5_femur, self.Leg_5_tibia, 
            self.Leg_6_coxa, self.Leg_6_femur, self.Leg_6_tibia, 
        ]

class SimpleTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('simple_trajectory_publisher')
        self.publisher_ = self.create_publisher(JointTrajectory, '/hexapod_controller/joint_trajectory', 10)
        self.joint_positions = JointPositions()
        self.traj_msg = JointTrajectory()
        self.traj_msg.joint_names = [
            'Leg_1_coxa', 'Leg_1_femur', 'Leg_1_tibia',
            'Leg_2_coxa', 'Leg_2_femur', 'Leg_2_tibia',
            'Leg_3_coxa', 'Leg_3_femur', 'Leg_3_tibia',
            'Leg_4_coxa', 'Leg_4_femur', 'Leg_4_tibia',
            'Leg_5_coxa', 'Leg_5_femur', 'Leg_5_tibia',
            'Leg_6_coxa', 'Leg_6_femur', 'Leg_6_tibia',
        ]
        self.reach_time_sec = 0
        self.reach_time_nanosec = 0
        self.x = 170
        self.y = 120
        self.z = 0
        self.get_logger().info('Simple Trajectory Publisher has been started.')

    def create_point(self, TIME):
        point = JointTrajectoryPoint()
        point.positions = self.joint_positions.get_positions()

        sec = int(TIME)
        nanosec = int((TIME - sec) * 1_000_000_000)

        self.reach_time_sec += sec
        self.reach_time_nanosec += nanosec

        if self.reach_time_nanosec >= 1e9:
            extra_sec = self.reach_time_nanosec // 1e9
            self.reach_time_sec += int(extra_sec)
            self.reach_time_nanosec = self.reach_time_nanosec % int(1e9)

        point.time_from_start.sec = self.reach_time_sec
        point.time_from_start.nanosec = self.reach_time_nanosec
        self.traj_msg.points.append(point)
    
    def ik(self, x, y, z):
        x=x-70
        gamma = math.atan2(z, 70 + x)

        x_prime = math.sqrt(((70 + x)**2)+(z**2))-70
        beta = math.acos((40800.25 - x_prime**2 - y**2) / 35100)
        distance = math.sqrt(x_prime**2 + y**2)
        alpha = math.radians(180) - math.acos((x_prime**2 + y**2 - 20800.25) / (200 * distance)) + math.atan2(y, x_prime)

        gamma_deg = math.degrees(gamma)
        beta_deg = math.degrees(beta)
        alpha_deg = math.degrees(alpha)
        
        return [gamma_deg, alpha_deg, beta_deg]


    def create_trajectory(self):
    # Clear the previous points to avoid piling up
        self.traj_msg.points.clear()

        self.angles = self.ik(self.x, self.y, self.z)
        self.get_logger().info(f'Publishing angles: coxa: {self.angles[0]}, femur: {self.angles[1]}, tibia: {self.angles[2]}')

        self.joint_positions.Leg_1_coxa = math.radians(self.angles[0])
        self.joint_positions.Leg_1_femur = math.radians(self.angles[1] - 146.2)
        self.joint_positions.Leg_1_tibia = -math.radians(self.angles[2] - 63.8)

        self.get_logger().info(f'Publishing sim angles: coxa: {self.joint_positions.Leg_1_coxa}, femur: {self.joint_positions.Leg_1_femur}, tibia: {self.joint_positions.Leg_1_tibia}')

        self.create_point(0.1)
        self.publisher_.publish(self.traj_msg)
        self.get_logger().info('Published trajectory')


    def update_position_and_publish(self):
        self.create_trajectory() 
        while rclpy.ok():
            direction = input("Enter direction to change (x, y, z) or 'q' to quit: ").strip().lower()
            if direction == 'q':
                break

            if direction not in ['x', 'y', 'z']:
                print("Invalid direction. Please enter 'x', 'y', or 'z'.")
                continue

            try:
                
                value = float(input(f"Enter the new value for {direction}: "))
                setattr(self, direction, value)  # Set the new value directly

                self.create_trajectory()  # Create and publish the new trajectory with the updated values

            except ValueError:
                print("Invalid value. Please enter a numeric value.")
                continue

def main(args=None):
    rclpy.init(args=args)
    node = SimpleTrajectoryPublisher()
    try:
        node.update_position_and_publish()
    except KeyboardInterrupt:
        print("\nShutting down node.")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
