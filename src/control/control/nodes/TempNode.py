#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import board

# Use your custom Temp message
from sensor_msgs.msg import Temp
from std_msgs.msg import Float64

# Import the driver class from the separate file
from control.services.Temp import TemperatureSensor

class TempNode(Node):
    def __init__(self):
        super().__init__("temp_node")
        
        # Initialize the hardware via the imported class
        try:
            self.hw484 = TemperatureSensor(board.D4)
            self.get_logger().info("HW-484 Sensor Started Successfully")
        except Exception as e:
            self.get_logger().error(f"Could not start sensor: {e}")
            return

        # Create Publishers using 'Temp'
        self.temp_pub = self.create_publisher(Temp, "temp", 10)
        self.hum_pub = self.create_publisher(Float64, "humidity", 10)
        
        self.timer = self.create_timer(2.0, self.run)

    def run(self):
        try:
            t, h = self.hw484.get_data()
            
            if t is not None and h is not None:
                # Publish Temperature using 'Temp'
                t_msg = Temp()
                
                # IMPORTANT: If your Temp.msg doesn't have a header, remove the next two lines!
                # t_msg.header.stamp = self.get_clock().now().to_msg()
                # t_msg.header.frame_id = "temp_sensor_link"
                
                t_msg.temperature = float(t)
                self.temp_pub.publish(t_msg)

                # Publish Humidity
                h_msg = Float64()
                h_msg.data = float(h)
                self.hum_pub.publish(h_msg)

                self.get_logger().info(f"Published - Temp: {t}°C, Hum: {h}%")

        except RuntimeError as e:
            self.get_logger().warn(f"Sensor read skipped: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TempNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Temp Node...")
    finally:
        # Safely clean up the hardware pin on shutdown
        if hasattr(node, 'hw484'):
            node.hw484.sensor.exit()
        rclpy.shutdown()

if __name__ == "__main__":
    main()