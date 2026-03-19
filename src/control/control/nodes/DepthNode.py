#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
# Import your custom driver (assuming it's in the same folder)
from control.services.Depth import Depth
# Import your custom message
from sensor_msgs.msg import Depth as DepthMsg

class DepthNode(Node):
    def __init__(self):
        super().__init__("depth_node")

        # 1. Initialize Driver (using Alexandria's 1021.0 hPa)
        try:
            self.bmp = Depth(sea_level_pa=1021.0)
            self.get_logger().info("Sensor Driver Initialized. Taring...")
            self.bmp.tare()
        except Exception as e:
            self.get_logger().error(f"Driver Failure: {e}")
            return

        # 2. Setup ROS Infrastructure
        self.publisher = self.create_publisher(DepthMsg, "depth", 10)
        self.timer = self.create_timer(0.02, self.timer_callback) # 50Hz

    def timer_callback(self):
        try:
            # Get clean data from driver
            data = self.bmp.get_readings()

            # Fill ROS message
            msg = DepthMsg()
            msg.altitude = data["altitude"]
            msg.pressure = data["pressure"]
            msg.temperature = data["temperature"]

            self.publisher.publish(msg)
        except Exception as e:
            self.get_logger().warn(f"Publishing Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DepthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()