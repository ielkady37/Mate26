#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

# Importing your custom message
from rov_msgs.msg import Ms5 as MS5837Data

# Importing the driver class we just created above
from control.services.Ms5 import Ms5 as Ms5Driver


class Ms5Node(Node):

    def __init__(self):
        super().__init__("ms5_node")

        # 1. Start the driver
        # Change bus=1 to bus=4 if this sensor is on the same extended bus as your IMU!
        self.ms5 = Ms5Driver(bus=1) 
        self.get_logger().info("MS5837 Driver initialized")

        # 2. Setup Publisher
        self.publisher = self.create_publisher(MS5837Data, "sensor/ms5/data", 10)

        # 3. Setup Timer (10Hz / 0.1s is standard for this depth sensor)
        self.timer = self.create_timer(0.1, self.run)
        
        self.msg = MS5837Data()

    def run(self):
        
        try:
            # Unpack the tuple returned by our driver
            temp, press, depth, t_param, p_param, density = self.ms5.read_sensor()

            # Populate the custom message
            self.msg.temperature = float(temp)
            self.msg.pressure = float(press)
            self.msg.depth = float(depth)
            self.msg.temp_param = int(t_param)
            self.msg.p_param = int(p_param)
            self.msg.fluid_density = float(density)

            self.publisher.publish(self.msg)

        except Exception as e:
            # This will now actually trigger when the I2C bus fails!
            self.get_logger().warn(f"MS5837 read failed, skipping frame: {e}")


def main(args=None):

    rclpy.init(args=args)

    try:
        node = Ms5Node()
        rclpy.spin(node)

    except KeyboardInterrupt:
        print("Stopping...")

    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()