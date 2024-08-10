#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math
from std_srvs.srv import Trigger
from std_msgs.msg import Float64  # Import the Float64 message type

class SimpleTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('simple_trajectory_publisher')
        self.publisher_ = self.create_publisher(JointTrajectory, '/hexapod_controller/joint_trajectory', 10)
        self.client = self.create_client(Trigger, '/get_joint_state')
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
            
        self.request = Trigger.Request()

        self.traj_msg = JointTrajectory()
        self.traj_msg.joint_names = [
            'leg_1_coxa_joint', 'leg_1_femur_joint', 'leg_1_tibia_joint',
            'leg_2_coxa_joint', 'leg_2_femur_joint', 'leg_2_tibia_joint',
            'leg_3_coxa_joint', 'leg_3_femur_joint', 'leg_3_tibia_joint',
            'leg_4_coxa_joint', 'leg_4_femur_joint', 'leg_4_tibia_joint',
            'leg_5_coxa_joint', 'leg_5_femur_joint', 'leg_5_tibia_joint',
            'leg_6_coxa_joint', 'leg_6_femur_joint', 'leg_6_tibia_joint',
        ]
        self.reach_time_sec = 0
        self.reach_time_nanosec = 0
        self.x_list = [0]*6
        self.y_list = [160]*6
        self.z_list = [120]*6
        self.joint_positions = [0.0]*18
        self.req_speed_time = 0.5
        self.get_logger().info(f'Current speed: {2.5/self.req_speed_time} cm/s')
        self.rate = self.create_rate(10)
        
        #use this to update velocities
        #ros2 topic pub /update_speed std_msgs/msg/Float64 "data: 1.0" --once

        self.create_subscription(Float64, '/update_speed', self.update_speed_callback, 10)
        
        self.gaits()
    
    def update_speed_callback(self, msg):
        # Update the speed time from the received message
        new_speed_time = 2.5/msg.data
        if new_speed_time > 0:
            self.req_speed_time = new_speed_time
            self.get_logger().info(f'Updated speed: {2.5/self.req_speed_time}')
        else:
            self.get_logger().warning('Received non-positive speed time. Ignoring.')

    def create_multiple_points(self, TIME):
        point = JointTrajectoryPoint()
        point.positions = self.joint_positions

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
    
    def create_point(self, TIME):
        point = JointTrajectoryPoint()
        point.positions = self.joint_positions

        sec = int(TIME)
        nanosec = int((TIME - sec) * 1_000_000_000)

        point.time_from_start.sec = sec
        point.time_from_start.nanosec = nanosec
        self.traj_msg.points.append(point)
    
    def ik(self, x, y, z):
        gamma = math.atan2(x, y)
        y_prime = math.sqrt(((x)**2)+(y**2))-54.2
        beta = math.acos((40800.25 - y_prime**2 - z**2) / 35100)
        distance = math.sqrt(y_prime**2 + z**2)
        alpha = math.radians(180) - math.acos((y_prime**2 + z**2 - 20800.25) / (200 * distance)) + math.atan2(z, y_prime)

        gamma_deg = math.degrees(gamma)
        beta_deg = math.degrees(beta)
        alpha_deg = math.degrees(alpha)
        return [gamma_deg, alpha_deg, beta_deg]
    
    def z(self, amount, legs):
        for i in legs:
            self.z_list[i-1] += amount
    
    def x(self, amount, legs):
        for i in legs:
            self.x_list[i-1] += amount

    def create_trajectory(self, trajectory_time):
        self.traj_msg.points.clear()
        for i in range(6):
            if i < 3:
                self.angles = self.ik(self.x_list[i], self.y_list[i], self.z_list[i])
            else:
                self.angles = self.ik(-self.x_list[i], self.y_list[i], self.z_list[i])
            
            self.joint_positions[i*3] = math.radians(self.angles[0])
            if i < 3:
                self.joint_positions[i*3+1] = math.radians(self.angles[1] - 146.2)
            else:
                self.joint_positions[i*3+1] = -math.radians(self.angles[1] - 146.2)

            self.joint_positions[i*3+2] = -math.radians(self.angles[2] - 63.8)

        self.create_point(trajectory_time)
        self.publisher_.publish(self.traj_msg)
        self.wait_until_time(trajectory_time)
    
    def gaits(self):
        self.z(-50, [1, 3, 5])
        self.create_trajectory(self.req_speed_time)

        self.x(-25, [1, 3, 5])
        self.x(25, [2, 4, 6])
        self.create_trajectory(self.req_speed_time)

        self.z(50, [1, 3, 5])
        self.create_trajectory(self.req_speed_time)

        while True:
            self.z(-50, [2, 4, 6])
            self.x(25, [1, 3, 5])
            self.x(-25, [2, 4, 6])
            self.create_trajectory(self.req_speed_time)
            self.get_logger().info(f'Current speed: {2.5/self.req_speed_time}')

            self.z(50, [2, 4, 6])
            self.x(25, [1, 3, 5])
            self.x(-25, [2, 4, 6])
            self.create_trajectory(self.req_speed_time)
            self.get_logger().info(f'Current speed: {2.5/self.req_speed_time}')
            
            self.z(-50, [1, 3, 5])
            self.x(25, [2, 4, 6])
            self.x(-25, [1, 3, 5])
            self.create_trajectory(self.req_speed_time)
            self.get_logger().info(f'Current speed: {2.5/self.req_speed_time}')

            self.z(50, [1, 3, 5])
            self.x(25, [2, 4, 6])
            self.x(-25, [1, 3, 5])
            self.create_trajectory(self.req_speed_time)
            self.get_logger().info(f'Current speed: {2.5/self.req_speed_time}')
    
    def wait_until_time(self, target_time):
        target_time += self.call_service()
        while rclpy.ok():
            current_time = self.call_service()
            if current_time >= target_time:
                break

    def call_service(self):
        future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            return float(future.result().message)
        else:
            self.get_logger().error('Service call failed')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleTrajectoryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down node.")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
