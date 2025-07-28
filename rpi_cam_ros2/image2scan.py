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
        self.declare_parameter('height', 0.1) # meters above the floor

        # Camera paramters
        self.camera_model = PinholeCameraModel()
        self.camera_height = self.get_parameter('height').get_parameter_value().double_value  
        self.got_info = False
        
        # Set up ROS publiser and subscriber(s)
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/camera/image', self.image_callback, qos_profile_sensor_data)
        self.info_sub = self.create_subscription(CameraInfo, '/camera/info', self.info_callback, qos_profile_sensor_data)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)

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
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        height, width, _ = image.shape
        self.get_logger().info(f"Git image: {width} x {height}.")

        # Convert to HSV for color thresholding
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Detect red in HSV (red wraps around 0/180)
        lower_th1 = np.array([0, 70, 50])
        upper_th1 = np.array([10, 255, 255])
        lower_th2 = np.array([170, 70, 50])
        upper_th2 = np.array([180, 255, 255])
        mask1 = cv2.inRange(hsv, lower_th1, upper_th1)
        mask2 = cv2.inRange(hsv, lower_th2, upper_th2)
        red_mask = cv2.bitwise_or(mask1, mask2)

        # Morphological cleaning
        kernel = np.ones((3, 3), np.uint8)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)

        # Only process bottom half of image
        red_mask_roi = red_mask[height // 2 :, :]
        offset_v = height // 2

        # Camera intrinsics
        fx = self.camera_model.fx()
        fy = self.camera_model.fy()
        cx = self.camera_model.cx()
        cy = self.camera_model.cy()

        # Collect 2D scan points
        scan_points = []
        for v in range(red_mask_roi.shape[0]):
            for u in range(0, red_mask_roi.shape[1], 2):
                if red_mask_roi[v, u] == 0:
                    continue

                pixel_v = v + offset_v

                # Project pixel to normalized ray
                x = (u - cx) / fx
                y = (pixel_v - cy) / fy
                ray = np.array([x, y, 1.0])
                ray /= np.linalg.norm(ray)

                if ray[1] >= 0:
                    continue  # skip if ray goes upward

                scale = -self.camera_height / ray[1]
                point = ray * scale  # intersection with ground

                r = math.sqrt(point[0] ** 2 + point[2] ** 2)
                theta = math.atan2(point[0], point[2])

                if 0.1 < r < 10.0:
                    scan_points.append((theta, r))

        # Saftey check
        if not scan_points:
            return

        # Convert to LaserScan format
        scan_points.sort()
        angle_min = -math.pi / 4
        angle_max = math.pi / 4
        angle_increment = math.radians(1.0)
        num_readings = int((angle_max - angle_min) / angle_increment)
        ranges = [float('inf')] * num_readings

        for theta, r in scan_points:
            idx = int((theta - angle_min) / angle_increment)
            if 0 <= idx < num_readings:
                ranges[idx] = min(ranges[idx], r)

        # Publish LaserScan
        scan_msg = LaserScan()
        scan_msg.header = Header()
        scan_msg.header.stamp = msg.header.stamp
        scan_msg.header.frame_id = self.camera_model.tfFrame()
        scan_msg.angle_min = angle_min
        scan_msg.angle_max = angle_max
        scan_msg.angle_increment = angle_increment
        scan_msg.time_increment = 0.0
        scan_msg.range_min = 0.1
        scan_msg.range_max = 10.0
        scan_msg.ranges = ranges

        self.scan_pub.publish(scan_msg)

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

