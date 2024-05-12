#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

class RvizToGazebo(Node):
    def __init__(self):
        super().__init__("rviz_to_gazebo")
        self.get_logger().info("Rviz to Gazebo node started")
        self.pose_subscriber = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_callback,
            100
        )
        self.cmd_vel_pub = self.create_publisher(
            Float64MultiArray,
            "/hexapod_controller/commands",
            100
        )

    def joint_callback(self, msg: JointState):
        # Extract the position data from the JointState message
        positions = msg.position
        self.get_logger().info("Subscribing %s" %positions)

        # Publish the position data to the cmd_vel_pub topic
        self.publish_positions(positions)

    def publish_positions(self, positions):
        msg = Float64MultiArray()
        msg.data = positions
        self.cmd_vel_pub.publish(msg)
        self.get_logger().info("Publishing: %s" % msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = RvizToGazebo()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
