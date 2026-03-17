#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Depth  # Make sure this path is correct
import time

class DepthNode(Node):

    def __init__(self):
        super().__init__("depth_node")

        # Initialize the BMP280 sensor
        self.depth_sensor = Depth(sea_level_pa=1013.25)
        if self.depth_sensor.sensor:
            self.get_logger().info("BMP280 sensor initialized successfully.")
        else:
            self.get_logger().warn("BMP280 sensor initialization failed.")

        # Publisher: altitude=x, pressure=y, temperature=z
        self.publisher = self.create_publisher(Depth, "depth", 10)

        # Timer: 50Hz → 0.02s
        self.timer = self.create_timer(0.02, self.run)

        self.msg = Depth()

    def run(self):
        if not self.depth_sensor.sensor:
            self.get_logger().warn("No BMP280 sensor detected.")
            return

        try:
            # Read sensor values
            altitude = self.depth_sensor.read_altitude()
            pressure = self.depth_sensor.read_pressure()
            temperature = self.depth_sensor.read_temperature()

            # Fill message
            self.msg.altitude = altitude
            self.msg.pressure = pressure
            self.msg.temperature = temperature

            self.publisher.publish(self.msg)

        except Exception as e:
            self.get_logger().warn(f"BMP280 read error: {e}")


def main(args=None):
    rclpy.init(args=args)
    try:
        node = DepthNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()