#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

class JoyListenerNode(Node):

    def __init__(self):
        super().__init__("joy_listener_node")

        self.subscription = self.create_subscription(
            Joy, 
            "/Joy", 
            self.joy_callback, 
            10
        )
        self.get_logger().info("📡 Button-Only Control Mode Active. Waiting for commands...")

    def joy_callback(self, msg):
        try:
            # --- 1. Map the Buttons based on standard Gamepad API ---
            # Gripper
            close_gripper = msg.buttons[0] # Cross (X)
            open_gripper  = msg.buttons[1] # Circle (O)
            
            # Vertical (Heave)
            go_down = msg.buttons[6] # L2
            go_up   = msg.buttons[7] # R2
            
            # Rotation (Yaw)
            rotate_left  = msg.buttons[4] # L1
            rotate_right = msg.buttons[5] # R1
            
            # Planar Movement (D-Pad)
            move_forward  = msg.buttons[12]
            move_backward = msg.buttons[13]
            move_left     = msg.buttons[14]
            move_right    = msg.buttons[15]

            # --- 2. Filter and Build Active Commands List ---
            active_commands = []

            # Check Gripper
            if open_gripper == 1:
                active_commands.append("GRIPPER: OPENING")
            elif close_gripper == 1:
                active_commands.append("GRIPPER: CLOSING")

            # Check Vertical
            if go_up == 1:
                active_commands.append("HEAVE: UP")
            elif go_down == 1:
                active_commands.append("HEAVE: DOWN")

            # Check Rotation
            if rotate_left == 1:
                active_commands.append("YAW: LEFT")
            elif rotate_right == 1:
                active_commands.append("YAW: RIGHT")

            # Check Planar Movement
            if move_forward == 1:
                active_commands.append("MOVE: FORWARD")
            elif move_backward == 1:
                active_commands.append("MOVE: BACKWARD")
            
            if move_left == 1:
                active_commands.append("MOVE: LEFT (STRAFE)")
            elif move_right == 1:
                active_commands.append("MOVE: RIGHT (STRAFE)")

            # --- 3. Print the Output ---
            if active_commands:
                # If commands exist, print them clearly
                command_string = " | ".join(active_commands)
                self.get_logger().info(f"🚀 EXECUTING -> {command_string}")

        except IndexError:
            self.get_logger().warn("Received incomplete joystick array.")

def main(args=None):
    rclpy.init(args=args)
    try:
        node = JoyListenerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Stopping Joy Listener...")
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()