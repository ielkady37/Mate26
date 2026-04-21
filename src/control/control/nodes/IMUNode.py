#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
import time

from rov_msgs.msg import IMU as IMUMessage
from control.services.IMU import IMU as IMUDriver


class IMUNode(Node):

    def __init__(self):
        super().__init__("imu_node")

        self.get_logger().info("Starting IMU Driver...")

        # Start the driver (Relying on Software I2C via boot config for stability)
        self.imu = IMUDriver()
        
        self.imu.Calibrate(5)
        self.get_logger().info("Calibration finished")
        
        self.publisher = self.create_publisher(IMUMessage, "imu", 10)

        # 50Hz timer
        self.timer = self.create_timer(0.02, self.run)
        
        self.msg = IMUMessage()

    def run(self):
        
        try:
            roll, pitch, yaw = self.imu.getEulerAngles()

            self.msg.roll = math.degrees(roll)
            self.msg.pitch = math.degrees(pitch)
            self.msg.yaw = math.degrees(yaw)

            self.publisher.publish(self.msg)

        except Exception as e:
            # Logs the error but keeps the node alive to try again next frame
            self.get_logger().warn(f"IMU read failed, skipping frame: {e}")


def main(args=None):

    rclpy.init(args=args)

    try:
        node = IMUNode()
        rclpy.spin(node)

    except KeyboardInterrupt:
        print("Stopping...")

    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()