#!/usr/bin/env python3

import cv2
import threading
import logging
import time
import rclpy
from rclpy.node import Node
from http import server
import socketserver
import numpy as np

# Import the perfected camera module from your services folder
from control.services.camera import StereoCamera

logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')

# ── Shared Memory & Synchronization ───────────────────────────────────────
latest_jpeg_left = None
latest_jpeg_right = None
frame_condition = threading.Condition()

class StreamingHandler(server.BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        pass  # Suppress spammy HTTP logs

    def do_GET(self):
        global latest_jpeg_left, latest_jpeg_right
        
        # Route the request
        if self.path == '/stream_left.mjpg':
            stream_target = 'left'
        elif self.path == '/stream_right.mjpg':
            stream_target = 'right'
        else:
            self.send_error(404)
            self.end_headers()
            return
            
        # Standard MJPEG HTTP Headers
        self.send_response(200)
        self.send_header('Age', 0)
        self.send_header('Cache-Control', 'no-cache, private')
        self.send_header('Pragma', 'no-cache')
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
        self.end_headers()
        
        try:
            while True:
                with frame_condition:
                    frame_condition.wait()
                    frame_data = latest_jpeg_left if stream_target == 'left' else latest_jpeg_right
                    
                if frame_data is None:
                    continue
                    
                self.wfile.write(b'--FRAME\r\n')
                self.send_header('Content-Type', 'image/jpeg')
                self.send_header('Content-Length', len(frame_data))
                self.end_headers()
                self.wfile.write(frame_data)
                self.wfile.write(b'\r\n')
                
        except Exception:
            return

class ThreadedHTTPServer(socketserver.ThreadingMixIn, server.HTTPServer):
    daemon_threads = True

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.get_logger().info("Initializing Dual-Stream Camera Node...")
        
        # 1. Start Network Server
        self.port = 8083
        self.start_network_server(self.port)
        
        self.TARGET_FPS = 30.0
        self.TARGET_FRAME_TIME = 1.0 / self.TARGET_FPS
        
        # Track consecutive dropped frames for the robust disconnect logic
        self.dropped_frame_count = 0 
        
        # 2. Initialize Hardware
        try:
            self.camera_instance = StereoCamera(
                camera_device_index=0, 
                hardware_width=2560,
                hardware_height=720,
                fps=int(self.TARGET_FPS),
            )
        except Exception as e:
            self.get_logger().error(f"Camera Hardware Init Failed: {e}")
            raise e

        # 3. Start Processing Thread
        self.thread = threading.Thread(target=self.camera_loop, daemon=True)
        self.thread.start()

    def start_network_server(self, port):
        server_address = ('0.0.0.0', port)
        self.httpd = ThreadedHTTPServer(server_address, StreamingHandler)
        self.get_logger().info(f"Left Stream:  http://<pi_ip>:{port}/stream_left.mjpg")
        self.get_logger().info(f"Right Stream: http://<pi_ip>:{port}/stream_right.mjpg")
        server_thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
        server_thread.start()

    def camera_loop(self):
        global latest_jpeg_left, latest_jpeg_right
        prev_time = time.perf_counter()
        fps_smoothed = self.TARGET_FPS

        while rclpy.ok():
            loop_start = time.perf_counter()
            
            # 1. Pull Frames
            left, right = self.camera_instance.retrieve_synchronized_stereo_frames()

            if left is not None and right is not None:
                # Reset drop counter on success
                self.dropped_frame_count = 0 
                
                # 2. Process & Resize individually
                left_disp = cv2.resize(left, (640, 360), interpolation=cv2.INTER_LINEAR)
                right_disp = cv2.resize(right, (640, 360), interpolation=cv2.INTER_LINEAR)

                # Overlays
                cv2.putText(left_disp, f"L-FPS: {fps_smoothed:.1f}", (10, 35), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
                cv2.putText(right_disp, f"R-FPS: {fps_smoothed:.1f}", (10, 35), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)

                # 3. Encode
                ret_l, encoded_l = cv2.imencode('.jpg', left_disp, [cv2.IMWRITE_JPEG_QUALITY, 80])
                ret_r, encoded_r = cv2.imencode('.jpg', right_disp, [cv2.IMWRITE_JPEG_QUALITY, 80])
                
                if ret_l and ret_r:
                    with frame_condition:
                        latest_jpeg_left = encoded_l.tobytes()
                        latest_jpeg_right = encoded_r.tobytes()
                        frame_condition.notify_all()
            else:
                # Increment the fail counter
                self.dropped_frame_count += 1
                
                # ONLY show the "CONNECTION LOST" screen if it drops 30 frames in a row (1 full second)
                if self.dropped_frame_count > 30:
                    blank = np.zeros((360, 640, 3), dtype=np.uint8)
                    cv2.putText(blank, "CONNECTION LOST", (150, 180), 
                                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
                    ret, encoded = cv2.imencode('.jpg', blank, [cv2.IMWRITE_JPEG_QUALITY, 50])
                    
                    if ret:
                        with frame_condition:
                            latest_jpeg_left = latest_jpeg_right = encoded.tobytes()
                            frame_condition.notify_all()
            
            # 4. Precise Throttle
            time_spent = time.perf_counter() - loop_start
            time_left = self.TARGET_FRAME_TIME - time_spent
            if time_left > 0:
                time.sleep(time_left)

            now = time.perf_counter()
            fps_smoothed = 0.9 * fps_smoothed + 0.1 * (1.0 / max(now - prev_time, 0.001))
            prev_time = now

def main(args=None):
    rclpy.init(args=args)
    try:
        node = CameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals() and hasattr(node, 'camera_instance'):
            node.camera_instance.release_stereo_camera_hardware()
        rclpy.shutdown()

if __name__ == "__main__":
    main()