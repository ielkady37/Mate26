#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math

from sensor_msgs.msg import IMU
from control.services.IMU import IMU


class IMUNode(Node):

    def __init__(self):
        super().__init__("imu_node")

        self.imu = IMU()
        self.imu.Calibrate(5)
        self.get_logger().info("Calibration finished")
        self.publisher = self.create_publisher(IMU, "imu", 10)

        # 50Hz
        self.timer = self.create_timer(0.02, self.run)

        self.msg = IMU()

    def run(self):

        try:
            roll, pitch, yaw = self.imu.getEulerAngles()

            self.msg.roll = math.degrees(roll)
            self.msg.pitch = math.degrees(pitch)
            self.msg.yaw = math.degrees(yaw)

            self.publisher.publish(self.msg)

        except Exception as e:
            self.get_logger().warn(f"IMU read error: {e}")


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