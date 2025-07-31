#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo, LaserScan
from image_geometry import PinholeCameraModel
from std_msgs.msg import Header
from cv_bridge import CvBridge
import numpy as np
import math
import cv2

class RPiImageToLaserScan(Node):
    def __init__(self):
        super().__init__('rpi_image_to_laserscan')

        # Declare and get parameters
        self.declare_parameter('height', 0.08) # meters above the floor

        # Camera paramters
        self.camera_model = PinholeCameraModel()
        self.camera_height = self.get_parameter('height').get_parameter_value().double_value  
        self.got_info = False

        # Laser scan parameters
        self.fov = math.radians(60)  # ~60 degrees
        self.angle_min = -self.fov / 2
        self.angle_max = self.fov / 2
        self.num_ranges = 120
        self.range_min = 0.01
        self.range_max = 3.00
                
        # Set up ROS publiser and subscriber(s)
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image,
            'camera/image',
            self.image_callback,
            qos_profile_sensor_data
        )
        self.info_sub = self.create_subscription(
            CameraInfo,
            'camera/info',
            self.info_callback,
            qos_profile_sensor_data
        )
        self.scan_pub = self.create_publisher(
            LaserScan,
            'scan',
            qos_profile_sensor_data
        )

    # Callback function for revecing camera information
    def info_callback(self, msg):
        if not self.got_info:
            self.camera_model.fromCameraInfo(msg)
            self.got_info = True
            self.get_logger().info("Camera intrinsics received.")

    # Callback function for revecing and converting camera images
    def image_callback(self, msg):
        if not self.got_info:
            return

        # Convert to OpenCV image
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            height, width, _ = image.shape
        except Exception as e:
            self.get_logger().error(f"CvBridge error: {e}")
            return
        
        # Convert to HSV for color thresholding
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Detect red in HSV (red wraps around 0/180)
        lower_th1 = np.array([0, 100, 100])
        upper_th1 = np.array([10, 255, 255])
        lower_th2 = np.array([160, 100, 100])
        upper_th2 = np.array([180, 255, 255])
        mask = cv2.bitwise_or(
            cv2.inRange(hsv, lower_th1, upper_th1),
            cv2.inRange(hsv, lower_th2, upper_th2)
        )
        
        # Find pixel coordinates
        pixels = np.column_stack(np.where(mask > 0)) 
        
        # Initialize scan ranges
        ranges = [float('inf')] * self.num_ranges
        angle_increment = (self.angle_max - self.angle_min) / self.num_ranges
        
        # Project pixels to rays
        for v, u in pixels:
            # Back-project to ray
            x = (u - self.camera_model.cx()) / self.camera_model.fx()
            y = -(v - self.camera_model.cy()) / self.camera_model.fy()
            z = 1.0
            ray = np.array([x, y, z])
            ray = ray / np.linalg.norm(ray)

            if ray[1] >= 0:
                continue  # ray going up

            t = -self.camera_height / ray[1]
            point = t * ray
            distance = np.linalg.norm(point)
            
            # Angle in image plane (X-Z)
            angle = math.atan2(point[0], point[2])
            if self.angle_min <= angle <= self.angle_max and self.range_min <= distance <= self.range_max:
                index = int((angle - self.angle_min) / angle_increment)
                if 0 <= index < self.num_ranges:
                    if distance < ranges[index]:
                        ranges[index] = distance
                
        # Publish laser scan
        scan = LaserScan()
        scan.header = Header()
        scan.header.stamp = msg.header.stamp
        scan.header.frame_id = self.camera_model.tfFrame()
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = angle_increment
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = ranges
        
        self.scan_pub.publish(scan)
        
# Main function       
def main(args = None):
    rclpy.init(args = args)
    node = RPiImageToLaserScan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

