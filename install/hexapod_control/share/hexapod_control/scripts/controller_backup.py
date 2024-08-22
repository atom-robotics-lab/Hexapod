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

        ]

        # Define a simple walking cycle
        walking_cycle = [
            [-0.25, -0.25, 0.0],
            
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
