#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


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
            self.Leg_5_coxa, gself.Leg_5_femur, self.Leg_5_tibia, 
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
        self.create_trajectory()
        self.get_logger().info('Simple Trajectory Publisher has been started.')

    def create_point(self, TIME):
        point = JointTrajectoryPoint()
        point.positions = self.joint_positions.get_positions()

        # Update the reach time with the provided sec and nanosec
        sec = int(TIME)
        nanosec = int((TIME - sec) * 1_000_000_000)

        self.reach_time_sec += sec
        self.reach_time_nanosec += nanosec

        # Handle nanosecond overflow
        if self.reach_time_nanosec >= 1e9:
            extra_sec = self.reach_time_nanosec // 1e9
            self.reach_time_sec += int(extra_sec)
            self.reach_time_nanosec = self.reach_time_nanosec % int(1e9)

        point.time_from_start.sec = self.reach_time_sec
        point.time_from_start.nanosec = self.reach_time_nanosec
        self.traj_msg.points.append(point)

    def create_trajectory(self):
        # Bring all joints to 0 position in time 1.6 seconds
        self.create_point(1.0)

        # Sets Leg_2_tibia to 0.2 position
        self.joint_positions.Leg_2_tibia = -0.2

        # Make point in which only Leg_2_tibia was changed
        self.create_point(1.6)

        # Publishes the trajectory in which all the points are sent
        self.publisher_.publish(self.traj_msg)
        self.get_logger().info('Publishing trajectory: %s' % self.traj_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleTrajectoryPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()