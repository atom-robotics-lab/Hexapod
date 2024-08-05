#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math
from std_srvs.srv import Trigger



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
            'Leg_1_coxa', 'Leg_1_femur', 'Leg_1_tibia',
            'Leg_2_coxa', 'Leg_2_femur', 'Leg_2_tibia',
            'Leg_3_coxa', 'Leg_3_femur', 'Leg_3_tibia',
            'Leg_4_coxa', 'Leg_4_femur', 'Leg_4_tibia',
            'Leg_5_coxa', 'Leg_5_femur', 'Leg_5_tibia',
            'Leg_6_coxa', 'Leg_6_femur', 'Leg_6_tibia',
        ]
        self.reach_time_sec = 0
        self.reach_time_nanosec = 0
        self.x_list = [0]*6
        self.y_list = [160]*6
        self.z_list = [120]*6
        self.joint_positions=[0.0]*18
        self.rate = self.create_rate(10)
        self.gaits()

        

    def create_point(self, TIME):
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
    
    def z(self,amount,legs):
        for i in legs:
            self.z_list[i-1]+=amount
    
    def x(self,amount,legs):
        for i in legs:
            self.x_list[i-1]+=amount
    
    
    
    



    def create_trajectory(self,trajectory_time):
        self.traj_msg.points.clear()
        self.reach_time_sec = 0
        self.reach_time_nanosec = 0


        for i in range(6):
            if i<3:
                self.angles = self.ik(self.x_list[i], self.y_list[i], self.z_list[i])
            else:
                self.angles = self.ik(-self.x_list[i], self.y_list[i], self.z_list[i])
            

            #self.get_logger().info(f'Publishing angles for leg {i+1}: coxa: {self.angles[0]}, femur: {self.angles[1]}, tibia: {self.angles[2]}')

            self.joint_positions[i*3]= math.radians(self.angles[0])
            if i<3:
                self.joint_positions[i*3+1]= math.radians(self.angles[1] - 146.2)
            else:
                self.joint_positions[i*3+1]= -math.radians(self.angles[1] - 146.2)

            self.joint_positions[i*3+2]= -math.radians(self.angles[2] - 63.8)

        self.create_point(trajectory_time)
        self.get_logger().info(f'Published trajectory at {self.call_service()}')
        self.publisher_.publish(self.traj_msg)
        self.get_logger().info(f'{self.traj_msg}')
        self.wait_until_time(trajectory_time)
    
    def gaits(self):
        
        cd5)
        
        self.z(-50, [1, 3, 5])
        self.create_trajectory(0.5)

        self.x(-25, [1, 3, 5])
        self.x(25, [2, 4, 6])
        self.create_trajectory(0.5)

        while True:

            self.z(50, [1, 3, 5])
            self.create_trajectory(0.5)

            self.z(-50, [2, 4, 6])
            self.create_trajectory(0.5)

            self.x(50, [1, 3, 5])
            self.x(-50, [2, 4, 6])
            self.create_trajectory(0.5)

            self.z(50, [2, 4, 6])
            self.create_trajectory(0.5)

            self.z(-50, [1, 3, 5])
            self.create_trajectory(0.5)

            self.x(-50, [1, 3, 5])
            self.x(50, [2, 4, 6])
            self.create_trajectory(0.5)









        




        



        

        



    
    def wait_until_time(self, target_time):
        target_time+=self.call_service()
        while rclpy.ok():
            current_time = self.call_service()
            if current_time >= target_time:
                break # Sleep for a short period to avoid busy-waiting

    def call_service(self):
        future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            return float(future.result().message)
        else:
            self.get_logger().error('Service call failed')


    #-------Interpolation function to keep the path straight ---------
    #-------Not being used right now----------------------------------
    def interpolate_and_create_trajectory(self, direction, start_value, end_value):
        step_size = 1.0 if end_value > start_value else -1.0
        current_value = start_value

        while (step_size > 0 and current_value < end_value) or (step_size < 0 and current_value > end_value):
            current_value += step_size
            setattr(self, direction, current_value)
            self.create_trajectory()

        # Ensure the final target value is reached
        
        setattr(self, direction, end_value)
        self.create_trajectory()

        

    #-------use this to publish values using terminal--------
    #-------Not being used right now-------------------------
    def update_position_and_publish(self):
        self.create_trajectory(0.5)
        self.get_logger().info('Published trajectory')

        while rclpy.ok():
            try:
                command = input("Enter command (format: | x/z [leg_numbers] amount | or | q | to quit): ").strip().lower()
                if command == 'q':
                    break

                parts = command.split()
                if len(parts) != 3:
                    print("Invalid command format. Please use: x/z [leg_numbers] amount")
                    continue

                direction = parts[0]
                legs = list(map(int, parts[1].strip('[]').split(',')))
                amount = float(parts[2])

                if direction not in ['x', 'z']:
                    print("Invalid direction. Please enter 'x' or 'z'.")
                    continue

                # Validate leg numbers
                if not all(1 <= leg <= 6 for leg in legs):
                    print("Invalid leg numbers. Please enter numbers between 1 and 6.")
                    continue

                # Call the appropriate function based on the direction
                if direction == 'x':
                    self.x(amount, legs)
                elif direction == 'z':
                    self.z(amount, legs)

                # Recompute and publish the trajectory
                self.create_trajectory(0.5)

            except ValueError:
                print("Invalid input. Please enter valid numbers.")
                continue





def main(args=None):
    rclpy.init(args=args)
    node = SimpleTrajectoryPublisher()

    try:
        rclpy.spin(node)

        #use this for publishing values through terminal
        #node.update_position_and_publish()

    except KeyboardInterrupt:
        print("\nShutting down node.")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
