#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from time import sleep

class Mynode(Node):
    def __init__(self):
        super().__init__("bot_control")
        self.cmd_vel_pub_ = self.create_publisher(Float64MultiArray, "/hexapod_controller/commands", 10)
        self.msg = Float64MultiArray()
        self.msg.data = [0.0] * 18
        self.cmd_vel_pub_.publish(self.msg)
        self.get_logger().info('Publishing: %s' % self.msg.data)
        sleep(2)

        self.msg.data[7]=0.2
        self.msg.data[9]=-0.2
        self.msg.data[11]=-0.2
        self.cmd_vel_pub_.publish(self.msg)
        self.get_logger().info('1')
        self.get_logger().info('Publishing: %s' % self.msg.data)
        sleep(2)
        while True:

            self.msg.data[1]=0.2
            self.msg.data[3]=-0.2
            self.msg.data[5]=-0.2
            self.cmd_vel_pub_.publish(self.msg)
            
            self.get_logger().info('Publishing: %s' % self.msg.data)
            sleep(2)

            self.msg.data[7]=0.0
            self.msg.data[9]=0.0
            self.msg.data[11]=0.0
            self.cmd_vel_pub_.publish(self.msg)
            self.get_logger().info('Publishing: %s' % self.msg.data)
            sleep(2)

            self.msg.data[6]=0.2
            self.msg.data[8]=0.2
            self.msg.data[10]=-0.2
            self.cmd_vel_pub_.publish(self.msg)
            
            self.get_logger().info('Publishing: %s' % self.msg.data)
            sleep(2)

            self.msg.data[1]=-0.2
            self.msg.data[3]=0.2
            self.msg.data[5]=0.2
            self.msg.data[0]=0.2
            self.msg.data[2]=0.2
            self.msg.data[4]=-0.2
            self.cmd_vel_pub_.publish(self.msg)
            
            self.get_logger().info('Publishing: %s' % self.msg.data)
            sleep(2)

            self.cmd_vel_pub_.publish(self.msg)
           
            self.msg.data[6]=0.0
            self.msg.data[8]=0.0
            self.msg.data[10]=0.0
            self.msg.data[7]=0.2
            self.msg.data[9]=-0.2
            self.msg.data[11]=-0.2
            self.cmd_vel_pub_.publish(self.msg)
            
            self.get_logger().info('Publishing: %s' % self.msg.data)
            sleep(2)
            
            self.msg.data[0]=-0.2
            self.msg.data[2]=-0.2
            self.msg.data[4]=0.2



        


        


        

def main(args=None):
    rclpy.init(args=args)
    node = Mynode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
