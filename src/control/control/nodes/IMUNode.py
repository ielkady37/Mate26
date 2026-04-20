#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math

import time
import board
from digitalio import DigitalInOut, Direction

from rov_msgs.msg import IMU as IMUMessage
from control.services.IMU import IMU as IMUDriver


class IMUNode(Node):

    def __init__(self):
        super().__init__("imu_node")

        # 1. Perform a manual, solid hardware reset
        self.reset_pin = DigitalInOut(board.D17) 
        self.reset_pin.direction = Direction.OUTPUT
        self.reset_pin.value = False  # Turn sensor off
        time.sleep(0.1)
        self.reset_pin.value = True   # Turn sensor on
        time.sleep(1.0) # WAIT 1 FULL SECOND for the firmware to boot up

        # 2. Start the driver (Notice we DO NOT pass the pin down anymore)
        self.imu = IMUDriver()
        
        self.imu.Calibrate(5)
        self.get_logger().info("Calibration finished")
        self.publisher = self.create_publisher(IMUMessage, "imu", 10)

        # 50Hz
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
            # This will now actually trigger when the I2C bus fails!
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