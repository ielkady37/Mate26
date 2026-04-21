#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from control.services.PCA import PCA

class MotionNode(Node):

    def __init__(self):
        super().__init__("motion_node")

        # 1. Hardware Initialization
        try:
            self.pca = PCA(i2c_address=0x40, frequency=50)
            self.get_logger().info("PCA9685 Initialized. Button-Control Mode Active.")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize PCA9685: {e}")
            raise SystemExit

        # 2. Initialize Ramp Controllers for each movement axis
        # Smoothing 0.1 means it takes ~1 second to reach full speed.
        self.heave_ramp = ButtonRampController(smoothing=0.1)
        self.yaw_ramp = ButtonRampController(smoothing=0.1)
        self.gripper_ramp = ButtonRampController(smoothing=0.2) # Faster for gripper

        # 3. Subscriber
        self.subscription = self.create_subscription(
            Joy, "/Joy", self.joy_callback, 10
        )

    def joy_callback(self, msg):
        try:
            # --- BUTTON MAPPING ---
            # Gripper: Circle (O) to Open, Cross (X) to Close
            btn_x = msg.buttons[0] 
            btn_o = msg.buttons[1]
            
            # Rotation: L1 (Left), R1 (Right)
            btn_l1 = msg.buttons[4]
            btn_r1 = msg.buttons[5]
            
            # Vertical: L2 (Down), R2 (Up)
            btn_l2 = msg.buttons[6]
            btn_r2 = msg.buttons[7]

            # --- LOGIC & RAMPING ---
            
            # 1. HEAVE (Up/Down)
            # Use R2 for Up, L2 for Down.
            heave_pwm = self.heave_ramp.process(btn_r2 or btn_l2)
            if btn_r2:
                self.pca.pca.channels[4].duty_cycle = heave_pwm
                self.pca.pca.channels[5].duty_cycle = 0
            elif btn_l2:
                self.pca.pca.channels[4].duty_cycle = 0
                self.pca.pca.channels[5].duty_cycle = heave_pwm
            else:
                self.pca.pca.channels[4].duty_cycle = 0
                self.pca.pca.channels[5].duty_cycle = 0

            # 2. YAW (Rotate)
            yaw_pwm = self.yaw_ramp.process(btn_l1 or btn_r1)
            if btn_r1:
                # Rotate Right: Left Motor Forward, Right Motor Backward
                self.pca.pca.channels[0].duty_cycle = yaw_pwm # Left Motor
                self.pca.pca.channels[1].duty_cycle = 0
                self.pca.pca.channels[2].duty_cycle = 0       # Right Motor
                self.pca.pca.channels[3].duty_cycle = yaw_pwm
            elif btn_l1:
                # Rotate Left: Left Motor Backward, Right Motor Forward
                self.pca.pca.channels[0].duty_cycle = 0
                self.pca.pca.channels[1].duty_cycle = yaw_pwm
                self.pca.pca.channels[2].duty_cycle = yaw_pwm
                self.pca.pca.channels[3].duty_cycle = 0
            else:
                # If no rotation buttons pressed, stop horizontal motors
                self.pca.pca.channels[0].duty_cycle = 0
                self.pca.pca.channels[1].duty_cycle = 0
                self.pca.pca.channels[2].duty_cycle = 0
                self.pca.pca.channels[3].duty_cycle = 0

            # 3. GRIPPER (Open/Close)
            # Example: Gripper on PCA Channels 6 and 7
            grip_pwm = self.gripper_ramp.process(btn_o or btn_x)
            if btn_o: # Open
                self.pca.pca.channels[6].duty_cycle = grip_pwm
                self.pca.pca.channels[7].duty_cycle = 0
            elif btn_x: # Close
                self.pca.pca.channels[6].duty_cycle = 0
                self.pca.pca.channels[7].duty_cycle = grip_pwm
            else:
                self.pca.pca.channels[6].duty_cycle = 0
                self.pca.pca.channels[7].duty_cycle = 0

        except Exception as e:
            self.get_logger().warn(f"Command Execution Error: {e}")

    def destroy_node(self):
        self.get_logger().info("Stopping all motors...")
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
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()