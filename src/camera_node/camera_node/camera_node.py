import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import os

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.declare_parameter('mode', 'testing')  # Default mode is 'testing'
        self.mode = self.get_parameter('mode').get_parameter_value().string_value
        
        self.publisher_ = self.create_publisher(Image, 'camera_frames', 10)
        self.timer = self.create_timer(0.1, self.publish_frame)  # 10 FPS
        self.video_file_path = 'test/surfing.mp4'
        self.video_capture = None
        self.bridge = CvBridge()
        
        if self.mode == 'testing':
            self.get_logger().info('Camera node running in testing mode with synthetic data')
            if os.path.exists(self.video_file_path):
                self.video_capture = cv2.VideoCapture(self.video_file_path)
                self.get_logger().info(f"Video file found: {self.video_file_path}")
            else:
                self.get_logger().info(f"Current working directory: {os.getcwd()}")
                self.get_logger().warn(f"Video file not found at {self.video_file_path}")
        else:
            self.get_logger().info('Camera node running in deployment mode with real camera')
            self.video_capture = cv2.VideoCapture(0)  # Use a real camera
        
    def publish_frame(self):
        ret, frame = self.video_capture.read()
        if not ret:
            self.get_logger().warn('No frame captured')
            if self.mode == 'testing':
                self.video_capture.set(cv2.CAP_PROP_POS_FRAMES, 0)  # Loop video
            return
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.publisher_.publish(msg)
        self.get_logger().debug('Published frame')

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
