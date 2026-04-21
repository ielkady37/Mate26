#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

# Import all your custom ROV messages
from rov_msgs.msg import IMU as IMUMessage
from rov_msgs.msg import Ms5 as MS5837Data
from rov_msgs.msg import Depth as DepthMsg

class EchoNode(Node):

    def __init__(self):
        super().__init__("echo_node")

        # 1. Create Subscribers for each sensor
        # Format: (Message Type, "topic_name", callback_function, queue_size)
        self.imu_sub = self.create_subscription(IMUMessage, "imu", self.imu_callback, 10)
        self.ms5_sub = self.create_subscription(MS5837Data, "sensor/ms5/data", self.ms5_callback, 10)
        
        # Note: Double check your DepthNode to ensure this topic name matches!
        self.depth_sub = self.create_subscription(DepthMsg, "sensor/depth/data", self.depth_callback, 10)

        self.get_logger().info("Echo Node initialized. Listening for sensor telemetry...")

    # 2. Callback Functions
    # These trigger automatically every time a new message arrives on the topic
    
    def imu_callback(self, msg):
        # Formatting to 2 decimal places so the terminal doesn't get flooded
        self.get_logger().info(f"[IMU] Roll: {msg.roll:.2f} | Pitch: {msg.pitch:.2f} | Yaw: {msg.yaw:.2f}")

    def ms5_callback(self, msg):
        self.get_logger().info(f"[MS5] Depth: {msg.depth:.2f}m | Temp: {msg.temperature:.2f}C")

    def depth_callback(self, msg):
        # Adjust these variable names to match what is actually inside your DepthMsg definition!
        self.get_logger().info(f"[BMP] Pressure: {msg.pressure:.2f} hPa")


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = EchoNode()
        rclpy.spin(node) # Keeps the node alive and listening
    except KeyboardInterrupt:
        print("Stopping Echo Node...")
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()