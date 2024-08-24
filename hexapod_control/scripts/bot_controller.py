#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Twist
import math
from builtin_interfaces.msg import Duration 

from adafruit_servokit import ServoKit
kit = ServoKit(channels=16)
servo = 14
servoint = 1

class Hexapod(Node):
    def __init__(self):
        super().__init__('Hexapod')
        self.publisher_ = self.create_publisher(JointTrajectory, '/hexapod_controller/joint_trajectory', 10)
        self.timer_period = 0.2  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.get_logger().info('Simple Trajectory Publisher has been started.')
        self.cycle_index = 0
        self.speed = 70 #mm/s
        self.step_length = 0.0
        self.step_length_r = 0.0
        self.step_length_l = 0.0
        self.walk_scale = 1.0
        self.side_count = 0

        self.tripod_gait_cycle = [ [(0.0, 0.0, 0.0)] * 6 ]
        self.joint_names = [
            'Leg_1_coxa', 'Leg_1_femur', 'Leg_1_tibia',   
            'Leg_2_coxa', 'Leg_2_femur', 'Leg_2_tibia',   
            'Leg_3_coxa', 'Leg_3_femur', 'Leg_3_tibia',   
            'Leg_4_coxa', 'Leg_4_femur', 'Leg_4_tibia',   
            'Leg_5_coxa', 'Leg_5_femur', 'Leg_5_tibia',   
            'Leg_6_coxa', 'Leg_6_femur', 'Leg_6_tibia',   
        ]
    
        self.pose_subscriber = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.joint_callback,
            10  
        )

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

        theta1 = math.atan2(x, y+170) 
        print("theta1",theta1)

        xy_dist = math.sqrt(x**2 + (y+175)**2) - l1
        
        d = math.sqrt(xy_dist**2 + z**2) + epsilon 
        
        cos_theta3 = (l2**2+l3**2-d**2) / (2 * l2 * l3)
        theta3 = math.acos(cos_theta3)
        print("theta3",theta3)
        
        cos_theta2 = (d**2 + l2**2 - l3**2) / (2 * d * l2)

        theta2 =  math.acos(cos_theta2) - math.atan2(z, xy_dist) 
        print("theta2",theta2)
        print()
        
        if self.side_count < 6:
            self.side_count += 1 
        else:
            self.side_count = 1

        if self.side_count <= 3:
            theta2,theta3 = theta2-math.radians(146.2),  math.radians(63.8)-theta3
        else:
            theta2,theta3 = math.radians(146.2)-theta2,  math.radians(63.8)-theta3


        return theta1, theta2,  theta3

    def rest(self):
        self.tripod_gait_cycle = [ [(0.0, 0.0, 0.0)] * 6 ]

    def move_forward(self):
        self.tripod_gait_cycle = [
            [(-self.step_length_r, 0.0, self.step_length), (-self.step_length_r, 0.0, -self.step_length), (-self.step_length_r, 0.0, self.step_length), (self.step_length, 0.0, -self.step_length), (self.step_length, 0.0, self.step_length), (self.step_length, 0.0, -self.step_length)],
            [(-self.step_length_r, 0.0, -self.step_length), (self.step_length_r, 0.0, -self.step_length), (-self.step_length_r, 0.0, -self.step_length), (-self.step_length, 0.0, -self.step_length), (self.step_length, 0.0, -self.step_length), (-self.step_length, 0.0, -self.step_length)],

            [(-self.step_length_r, 0.0, -self.step_length), (-self.step_length_r, 0.0, self.step_length), (-self.step_length_r, 0.0, -self.step_length), (self.step_length, 0.0, self.step_length), (self.step_length, 0.0, -self.step_length), (self.step_length, 0.0, self.step_length)],
            [(self.step_length_r, 0.0, -self.step_length), (-self.step_length_r, 0.0, -self.step_length), (self.step_length_r, 0.0, -self.step_length), (self.step_length, 0.0, -self.step_length), (-self.step_length, 0.0, -self.step_length), (self.step_length, 0.0, -self.step_length)],
        ]
        
    
    def move_backward(self):
        self.tripod_gait_cycle = [
            [(self.step_length, 0.0, self.step_length), (self.step_length, 0.0, -self.step_length), (self.step_length, 0.0, self.step_length), (-self.step_length, 0.0, -self.step_length), (-self.step_length, 0.0, self.step_length), (-self.step_length, 0.0, -self.step_length)],
            [(self.step_length, 0.0, -self.step_length), (-self.step_length, 0.0, -self.step_length), (self.step_length, 0.0, -self.step_length), (self.step_length, 0.0, -self.step_length), (-self.step_length, 0.0, -self.step_length), (self.step_length, 0.0, -self.step_length)],

            [(self.step_length, 0.0, -self.step_length), (self.step_length, 0.0, self.step_length), (self.step_length, 0.0, -self.step_length), (-self.step_length, 0.0, self.step_length), (-self.step_length, 0.0, -self.step_length), (-self.step_length, 0.0, self.step_length)],
            [(-self.step_length, 0.0, -self.step_length), (self.step_length, 0.0, -self.step_length), (-self.step_length, 0.0, -self.step_length), (-self.step_length, 0.0, -self.step_length), (self.step_length, 0.0, -self.step_length), (-self.step_length, 0.0, -self.step_length)],
        ]

    def turn_right(self):
        self.tripod_gait_cycle = [
            [(self.step_length, 0.0, self.step_length), (self.step_length, 0.0, -self.step_length), (self.step_length, 0.0, self.step_length), (self.step_length, 0.0, -self.step_length), (self.step_length, 0.0, self.step_length), (self.step_length, 0.0, -self.step_length)],
            [(self.step_length, 0.0, -self.step_length), (-self.step_length, 0.0, -self.step_length), (self.step_length, 0.0, -self.step_length), (-self.step_length, 0.0, -self.step_length), (self.step_length, 0.0, -self.step_length), (-self.step_length, 0.0, -self.step_length)],

            [(self.step_length, 0.0, -self.step_length), (self.step_length, 0.0, self.step_length), (self.step_length, 0.0, -self.step_length), (self.step_length, 0.0, self.step_length), (self.step_length, 0.0, -self.step_length), (self.step_length, 0.0, self.step_length)],
            [(-self.step_length, 0.0, -self.step_length), (self.step_length, 0.0, -self.step_length), (-self.step_length, 0.0, -self.step_length), (self.step_length, 0.0, -self.step_length), (-self.step_length, 0.0, -self.step_length), (self.step_length, 0.0, -self.step_length)],
        ]

    def turn_left(self):
        self.tripod_gait_cycle = [
            [(-self.step_length, 0.0, self.step_length), (-self.step_length, 0.0, -self.step_length), (-self.step_length, 0.0, self.step_length), (-self.step_length, 0.0, -self.step_length), (-self.step_length, 0.0, self.step_length), (-self.step_length, 0.0, -self.step_length)],
            [(-self.step_length, 0.0, -self.step_length), (self.step_length, 0.0, -self.step_length), (-self.step_length, 0.0, -self.step_length), (self.step_length, 0.0, -self.step_length), (-self.step_length, 0.0, -self.step_length), (self.step_length, 0.0, -self.step_length)],

            [(-self.step_length, 0.0, -self.step_length), (-self.step_length, 0.0, self.step_length), (-self.step_length, 0.0, -self.step_length), (-self.step_length, 0.0, self.step_length), (-self.step_length, 0.0, -self.step_length), (-self.step_length, 0.0, self.step_length)],
            [(self.step_length, 0.0, -self.step_length), (-self.step_length, 0.0, -self.step_length), (self.step_length, 0.0, -self.step_length), (-self.step_length, 0.0, -self.step_length), (self.step_length, 0.0, -self.step_length), (-self.step_length, 0.0, -self.step_length)],
        ]

    def joint_callback(self, msg: Twist):
        self.step_length = self.speed * self.timer_period
        self.step_length_l = self.step_length
        self.step_length_r = self.step_length

        if msg.angular.z == 0 and msg.linear.x != 0:
            self.speed = abs(msg.linear.x) 
            if msg.linear.x > 0:
               self.move_forward()
            if msg.linear.x < 0:
               self.move_backward()
        if msg.linear.x == 0 and msg.angular.z != 0:
            self.speed = abs(msg.angular.z) 
            if msg.angular.z < 0:
               self.turn_right()
            if msg.angular.z > 0:
               self.turn_left()
        if msg.linear.x > 0 and msg.angular.x > 0: 
            self.step_length_r = self.step_length/2.0
            self.move_forward()
        if msg.linear.x == 0 and msg.angular.z == 0:
           self.rest()

    def timer_callback(self):
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        segment_lengths = [70.0, 100.0, 175.0]  # segment lengths in mm
        positions = []
        for pos in self.tripod_gait_cycle[self.cycle_index % len(self.tripod_gait_cycle)]:
            x, y, z = pos
            angles = self.inverse_kinematics(x, y, z, *segment_lengths)
            angles = [angle * self.walk_scale for angle in angles]
            positions.extend(angles)
        point.positions = positions
        for i in range(len(angles)):
            #print(angles[i])
            kit.servo[servoint].angle = int(angles[i])
            
        point.time_from_start = Duration(sec=int(self.timer_period), nanosec=int((self.timer_period % 1) * 1e9))

        traj_msg.points.append(point)

        self.publisher_.publish(traj_msg)
        print(angles)
        self.cycle_index += 1  

def main(args=None):
    rclpy.init(args=args)
    node = Hexapod()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
