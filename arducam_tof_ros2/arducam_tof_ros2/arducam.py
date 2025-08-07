import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import ArducamDepthCamera as ac
import cv2


class ArducamNode(Node):
    def __init__(self):
        super().__init__('arducam_tof_node')

        # Get ROS param
        self.declare_parameter('camera_mode', 'depth')  # Can be "raw" or "depth"
        mode_str = self.get_parameter('camera_mode').get_parameter_value().string_value.lower()
        self.frame_type = ac.FrameType.RAW if mode_str == 'raw' else ac.FrameType.DEPTH
        self.get_logger().info(f"Selected ArduCam mode: {mode_str.upper()}")

        # Inatailze the ArduCam camera
        self.cam = ac.ArducamCamera()
        self.initialized = False
        self.init()

        # Setup ROS publisher
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, 'arducam/image', 10)
        self.timer = self.create_timer(1.0 / 30.0, self.capture)

    # Open and initialize the ArduCam camera
    def init(self) -> bool:
        self.get_logger().info(f"ArduCam SDK version: {ac.__version__}")
        ret = self.cam.open(ac.Connection.CSI)
        if ret != 0:
            self.get_logger().error(f"Failed to open ArduCam. Error code: {ret}")
        else:
            ret = self.cam.start(self.frame_type)
            if ret != 0:
                self.get_logger().error(f"Failed to start camera. Error code: {ret}")
                self.cam.close()
            else:
                if self.frame_type == ac.FrameType.DEPTH:
                    self.cam.setControl(ac.Control.RANGE, 4000)
                    self.range = self.cam.getControl(ac.Control.RANGE)
                self.get_logger().info("ArduCam initialized successfully.")
                info = self.cam.getCameraInfo()
                self.get_logger().info(f"Camera resolution: {info.width}x{info.height}")
                self.get_logger().info(f"Camera type: {info.device_type}")
                if self.frame_type == ac.FrameType.DEPTH:
                    self.get_logger().info(f"Camera depth range: {self.range / 1000.0} m")
                self.initialized = True

    # Cature frame from the ArduCam camera
    def capture(self):
        if self.initialized:
            frame = self.cam.requestFrame(200)
            if frame is not None:
                try: 
                    if self.frame_type == ac.FrameType.RAW and isinstance(frame, ac.RawData):
                        buf = frame.raw_data
                        buf = (buf / (1 << 4)).astype(np.uint8)
                        msg = self.bridge.cv2_to_imgmsg(buf, encoding='mono8')
                        self.publisher.publish(msg)

                    elif self.frame_type == ac.FrameType.DEPTH and isinstance(frame, ac.DepthData):
                        depth_buf = frame.depth_data
                        confidence_buf = frame.confidence_data
            
                        # Scale and colorize
                        scaled = (depth_buf * (255.0 / self.range)).astype(np.uint8)
                        colored = cv2.applyColorMap(scaled, cv2.COLORMAP_RAINBOW)
                        colored[confidence_buf < 30] = (0, 0, 0)  # Confidence filter

                        msg = self.bridge.cv2_to_imgmsg(colored, encoding='bgr8')
                        self.publisher.publish(msg)
                        
                finally:
                    self.cam.releaseFrame(frame)

    # Clean up
    def destroy_node(self):
        if hasattr(self, 'cam'):
            self.cam.stop()
            self.cam.close()
        super().destroy_node()


# Main fn
def main(args=None):
    rclpy.init(args=args)
    node = ArducamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

