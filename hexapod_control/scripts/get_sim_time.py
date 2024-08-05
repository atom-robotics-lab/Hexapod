#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

class JointStateService(Node):
    def __init__(self):
        super().__init__("joint_state_service")
        self.get_logger().info("Joint State Service started")
        
        # Subscribe to the /joint_states topic
        self.pose_subscriber = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_callback,
            10  # Adjusted the queue size to 10
        )
        
        # Create a service
        self.srv = self.create_service(Trigger, 'get_joint_state', self.handle_joint_state_request)
        
        # Store the last received JointState message
        self.last_joint_state = None

    def joint_callback(self, msg: JointState):
        self.last_joint_state = msg

    def handle_joint_state_request(self, request, response):
        if self.last_joint_state is not None:
            # Respond with the current JointState timestamp
            response.success = True
            response.message = f"{self.last_joint_state.header.stamp.sec+self.last_joint_state.header.stamp.nanosec/1e9} "
        else:
            response.success = False
            response.message = "No joint state received yet."
        
        return response

def main(args=None):
    rclpy.init(args=args)
    node = JointStateService()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
