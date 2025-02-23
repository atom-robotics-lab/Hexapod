#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from time import sleep

class SimpleTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('simple_trajectory_publisher')
        self.publisher_ = self.create_publisher(JointTrajectory, '/hexapod_controller/joint_trajectory', 10)
        timer_period = 1.0  # seconds
        self.timer_ = self.create_timer(1.0, self.timer_callback)  # Publish every second
        self.get_logger().info('Simple Trajectory Publisher has been started.')
        self.cycle_index = 0  # To track the current index in the walking cycle


    def timer_callback(self):
        traj_msg = JointTrajectory()
        traj_msg.joint_names = [
            'Leg_1_coxa', 'Leg_1_tibia', 'Leg_1_femur',   # 0  1  2    
            'Leg_2_coxa', 'Leg_2_tibia', 'Leg_2_femur',   # 3  4  5 
            'Leg_3_coxa', 'Leg_3_tibia', 'Leg_3_femur',   # 6  7  8
            'Leg_4_coxa', 'Leg_4_tibia', 'Leg_4_femur',   # 9  10 11 
            'Leg_5_coxa', 'Leg_5_tibia', 'Leg_5_femur',   # 12 13 14
            'Leg_6_coxa', 'Leg_6_tibia', 'Leg_6_femur',   # 15 16 17
        ]

        # Define a simple walking cycle
        walking_cycle = [
            [-0.25, -0.25, 0.0,   0.25, 0.0, 0.0,    -0.25, -0.25, 0.0,    -0.25, -0.25, 0.0,    0.25, 0.25, 0.0,    -0.25, -0.0, 0.0],
            [-0.25, 0.25, 0.0,   0.25, 0.0, 0.0,    -0.25, -0.0, 0.0,    -0.25, -0.25, 0.0,    0.25, 0.0, 0.0,    -0.25, -0.0, 0.0],
            [0.25, 0.25, 0.0,   -0.25, -0.25, 0.0,    0.25, 0.0, 0.0,    0.25, 0.25, 0.0,    -0.25, -0.0, 0.0,    0.25, 0.25, 0.0],
            [0.25, 0.0, 0.0,   -0.25, -0.0, 0.0,    0.25, 0.0, 0.0,    0.25, 0.0, 0.0,    -0.25, -0.0, 0.0,    0.25, 0.0, 0.0],
            #[0.3, -0.3, 0.0,   -0.3, -0.0, 0.0,    0.3, -0.3, 0.0,    0.3, 0.0, 0.0,    -0.3, -0.3, 0.0,    0.3, 0.0, 0.0]
            
        ]

        point = JointTrajectoryPoint()
        point.positions = walking_cycle[self.cycle_index % len(walking_cycle)]
        point.time_from_start.sec = 0  
        point.time_from_start.nanosec = 200000000   

        traj_msg.points.append(point)

        self.publisher_.publish(traj_msg)
        self.get_logger().info(f'Publishing trajectory: {traj_msg}')
        self.cycle_index += 1  # Update cycle index for the next callback



def main(args=None):
    rclpy.init(args=args)
    node = SimpleTrajectoryPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()