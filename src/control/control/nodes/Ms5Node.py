import rclpy
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure, Temperature

# Importing your hardware class from the services folder
from control.services.Ms5 import MS5837Service

class Ms5Node(Node):
    def __init__(self):
        super().__init__('ms5_node')
        
        # Create standard ROS 2 publishers
        self.pressure_pub = self.create_publisher(FluidPressure, 'sensor/ms5/pressure', 10)
        self.temp_pub = self.create_publisher(Temperature, 'sensor/ms5/temperature', 10)

        # Initialize the hardware service
        try:
            self.ms5_service = MS5837Service()
            self.get_logger().info("MS5837 Sensor initialized successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize MS5837: {e}")
            return # Prevent the timer from starting if hardware fails

        # Create a timer to read and publish data at 10 Hz (0.1 seconds)
        self.timer = self.create_timer(0.1, self.publish_sensor_data)

    def publish_sensor_data(self):
        data = self.ms5_service.read_data()
        
        if data:
            now = self.get_clock().now().to_msg()
            
            # --- Pressure Message ---
            # sensor_msgs/FluidPressure expects Pascals. The sensor outputs mbar.
            # 1 mbar = 100 Pascals
            p_msg = FluidPressure()
            p_msg.header.stamp = now
            p_msg.header.frame_id = 'ms5837_link'
            p_msg.fluid_pressure = float(data['pressure'] * 100.0) 
            
            # --- Temperature Message ---
            t_msg = Temperature()
            t_msg.header.stamp = now
            t_msg.header.frame_id = 'ms5837_link'
            t_msg.temperature = float(data['temperature']) # Celsius
            
            # Publish
            self.pressure_pub.publish(p_msg)
            self.temp_pub.publish(t_msg)
        else:
            self.get_logger().warning("Failed to read from MS5837 sensor.")

def main(args=None):
    rclpy.init(args=args)
    node = Ms5Node()
    
    # Only spin the node if the hardware initialized properly (timer exists)
    if hasattr(node, 'timer'):
        rclpy.spin(node)
        
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()