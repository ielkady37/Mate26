from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Start the IMU Node
        Node(
            package='control',
            executable='imu_node',
            name='imu_node',
            output='screen'
        ),
        
        # 2. Start the MS5 Depth Node
        Node(
            package='control',
            executable='ms5_node',
            name='ms5_node',
            output='screen'
        ),

        # 3. Start the BMP Depth Node
        Node(
            package='control',
            executable='depth_node',
            name='depth_node',
            output='screen'
        ),
        
        # 4. Start the Echo Node to listen to all of them!
        Node(
            package='control',
            executable='echo_node',
            name='echo_node',
            output='screen'
        ), # <--- THIS IS THE MISSING COMMA!

        # 5. Start the WebSocket Bridge for the Web GUI
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            parameters=[{'port': 9090}]
        ),
        # 6. Start the Motor Driver Node
        Node(
            package='control',
            executable='pca_node',
            name='pca_node',
            output='screen'
        ),
        Node(
            package='control',
            executable='camera_node',
            name='camera_node',
            output='screen'
        ),
        

    ])