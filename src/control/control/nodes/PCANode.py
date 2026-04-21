import rclpy
from rclpy.node import Node
# Import your custom driver (assuming it's in the same folder)
from control.services.PCADriver import PCADriver
# Import your custom message
from rov_msgs.msg import PCADriver as PCADriverMsg