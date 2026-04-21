#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

# Import your PCA driver
from control.services.PCA import PCA

class MotionNode(Node):

    def __init__(self):
        super().__init__("motion_node")

        # 1. Initialize the Hardware Driver
        try:
            self.pca = PCA(i2c_address=0x40, frequency=50)
            self.get_logger().info("PCA9685 initialized. Thrusters standing by.")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize PCA9685: {e}")
            raise SystemExit

        # 2. Subscribe to the Gamepad Topic
        self.subscription = self.create_subscription(
            Joy, 
            "/Joy", 
            self.joy_callback, 
            10
        )

    def joy_callback(self, msg):
        try:
            # 1. Extract the Joystick Axes
            # Left Stick Up/Down (Surge / Forward & Backward)
            surge = msg.axes[1] 
            
            # Left Stick Left/Right (Sway / Strafe)
            sway = msg.axes[0]  
            
            # Right Stick Up/Down (Heave / Dive & Surface)
            heave = msg.axes[3] 
            
            # Right Stick Left/Right (Yaw / Turn)
            yaw = msg.axes[2]   

            # 2. THRUST MIXER (Math to combine movements)
            # This calculates the raw power for a standard ROV frame. 
            # (Adjust these formulas based on where your thrusters are physically mounted!)
            
            left_horizontal_power = surge - yaw
            right_horizontal_power = surge + yaw
            vertical_power = heave

            # 3. Normalize values so we don't exceed limits (-1.0 to 1.0)
            def normalize(val):
                return max(-1.0, min(1.0, val))

            left_motor_final = normalize(left_horizontal_power)
            right_motor_final = normalize(right_horizontal_power)
            heave_motor_final = normalize(vertical_power)

            # 4. Write to the PCA9685 Hardware
            # NOTE: Change the (0, 1), (2, 3) channel numbers to match your actual wiring!
            
            # Left Motor (Wired to PCA pins 0 and 1)
            self.pca.PWMWrite(0, 1, left_motor_final)
            
            # Right Motor (Wired to PCA pins 2 and 3)
            self.pca.PWMWrite(2, 3, right_motor_final)
            
            # Vertical/Dive Motor (Wired to PCA pins 4 and 5)
            self.pca.PWMWrite(4, 5, heave_motor_final)

        except Exception as e:
            self.get_logger().warn(f"PWM Write Error: {e}")

    def destroy_node(self):
        # Crucial Safety Feature: Stop all motors if the node crashes!
        self.get_logger().info("Shutting down node, STOPPING ALL MOTORS...")
        if hasattr(self, 'pca'):
            self.pca.stopAll()
            self.pca.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = MotionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Stopping Motion Node...")
    finally:
        # Ensure destroy_node is called to brake the motors
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()