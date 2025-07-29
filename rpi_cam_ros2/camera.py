#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_default, qos_profile_sensor_data 
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import Image, CameraInfo
from camera_info_manager import CameraInfoManager
from cv_bridge import CvBridge
import subprocess
import threading
import numpy as np
import cv2
import time
import os

# Class for ha<ndle RPi Camera Module (v2)
class RPiCamSensor(Node):
    def __init__(self):
        super().__init__('rpi_camera_sensor')

        # Declare and get parameters
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('framerate', 30)
        self.declare_parameter('codec', 'mjpeg')
        self.declare_parameter('flip', True)
        self.declare_parameter('profile', 'best_effort') # QoS profile: best_effort or reliable

        self.width = self.get_parameter('width').get_parameter_value().integer_value
        self.height = self.get_parameter('height').get_parameter_value().integer_value
        self.framerate = self.get_parameter('framerate').get_parameter_value().integer_value
        self.codec = self.get_parameter('codec').get_parameter_value().string_value
        self.flip = self.get_parameter('flip').get_parameter_value().bool_value

        # Set up ROS publiser(s)
        if self.get_parameter('profile').get_parameter_value().string_value.lower() == 'reliable':
            qos_profile = qos_profile_default
        else:
            qos_profile = qos_profile_sensor_data
            qos_profile.depth = 1
        self.image_pub = self.create_publisher(Image, 'camera/image', qos_profile)
        self.info_pub = self.create_publisher(CameraInfo, 'camera/info', qos_profile)
        self.bridge = CvBridge()

        # Read CameraInfo parameters
        self.camera_info_msg = CameraInfo()
        try:
            url = os.path.join(get_package_share_directory('rpi_cam_ros2'), 'config', 'calibration.yaml')
            camera_info_manager = CameraInfoManager(self, cname = 'rpi_camera', url = f'file://{url}')
            camera_info_manager.loadCameraInfo()
            self.camera_info_msg = camera_info_manager.getCameraInfo()
        except Exception as e:
            self.get_logger().warn(f"Could not load calibration, using default instead.")        
            self.camera_info_msg.width = int(self.width)
            self.camera_info_msg.height = int(self.height)
            self.camera_info_msg.k = [1, 0, 0, 0, 1, 0, 0, 0, 1]  # Dummy
            self.camera_info_msg.p = [1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0]  # Dummy
            self.camera_info_msg.distortion_model = 'plumb_bob'
            self.camera_info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]

        # Start sepearte thread for reading camera frames
        self.frame = None
        self.lock = threading.Lock()
        self.running = True
        self.thread = threading.Thread(target = self.capture_process)
        self.thread.start()        

        # Timer based on framerate
        timer_period = 1.0 / float(self.framerate)
        self.timer = self.create_timer(timer_period, self.timer_callback)

    # Subprocess for reding camera buffer
    def capture_process(self):
        cmd = [
            'rpicam-vid',
            '--width', str(self.width),
            '--height', str(self.height),
            '--framerate', str(self.framerate),
            '--codec', self.codec,
            '--timeout', '0',   # Run indefinitely
            '--output', '-'     # Output to stdout
        ]

        # Start rpicam-vid process
        self.get_logger().info("Starting camera process... ")
        self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
        buffer = b''

        # Run frame capture loop
        while self.running and self.proc.poll() is None:
            byte = self.proc.stdout.read(1)
            if not byte:
                continue
            buffer += byte

            # MJPEG frame delimiter
            if buffer[-2:] == b'\xff\xd9':
                try:
                    np_arr = np.frombuffer(buffer, dtype=np.uint8)
                    frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                    if frame is not None:
                        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                        with self.lock:
                            self.frame = frame
                except Exception as e:
                    self.get_logger().warn(f"Failed to decode frame: {e}")
                buffer = b''
 
        
    # Timer callback for publishing images 
    def timer_callback(self):
        with self.lock:
            img = self.frame.copy() if self.frame is not None else None

        # Sned latest fram as image
        if img is not None:
            try:
                if self.flip:
                    img = cv2.flip(cv2.flip(img, 0), 1)
                now = self.get_clock().now().to_msg()
                image_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
                image_msg.header.stamp = now
                image_msg.header.frame_id = 'camera_frame'

                self.camera_info_msg.header.stamp = now
                self.camera_info_msg.header.frame_id = 'camera_frame'
                
                self.image_pub.publish(image_msg)
                self.info_pub.publish(self.camera_info_msg)

            except Exception as e:
                self.get_logger().error(f"Failed to publish image: {e}")

    # Clean up
    def destroy_node(self):
        self.get_logger().info("Shutting down camera process... ")
        self.running = False
        if hasattr(self, 'proc') and self.proc:
            self.proc.terminate()
        if hasattr(self, 'thread') and self.thread:
            self.thread.join()
        super().destroy_node()        

 # Main function       
def main(args = None):
    rclpy.init(args = args)
    node = RPiCamSensor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
