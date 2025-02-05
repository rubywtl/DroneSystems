import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import os

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.declare_parameter('mode', 'simulation')  # Default mode is 'simulation'
        self.mode = self.get_parameter('mode').get_parameter_value().string_value
        
        self.publisher_ = self.create_publisher(Image, 'camera_frames', 10)
        self.timer = self.create_timer(0.1, self.publish_frame) 
        self.video_file_path = 'test/surfing.mp4'
        self.video_capture = None
        self.bridge = CvBridge()
        
        left_pipeline = (
            'libcamerasrc camera-name="/base/axi/pcie@120000/rp1/i2c@88000/imx219@10" ! video/x-raw,width=640,height=480,framerate=30/1,format=NV12 ! '
            'videoconvert ! appsink'
        )

        right_pipeline = (
            'libcamerasrc camera-name="/base/axi/pcie@120000/rp1/i2c@80000/imx219@10" ! video/x-raw,width=640,height=480,framerate=30/1,format=NV12 ! '
            'videoconvert ! appsink'
        )
        
        if self.mode == 'simulation':
            self.get_logger().info('Camera node running in simulation mode with synthetic data')
            if os.path.exists(self.video_file_path):
                self.video_capture = cv2.VideoCapture(self.video_file_path)
                self.get_logger().info(f"Video file found: {self.video_file_path}")
                if not self.video_capture.isOpened():
                    self.get_logger().error("Failed to open video capture.")
            else:
                self.get_logger().warn(f"Video file not found at {self.video_file_path}")
        else:
            self.get_logger().info('Camera node running in deployment mode with real camera')
            # self.video_capture = cv2.VideoCapture(0)  
            self.left_cap = cv2.VideoCapture(left_pipeline, cv2.CAP_GSTREAMER)
            self.right_cap = cv2.VideoCapture(right_pipeline, cv2.CAP_GSTREAMER)
        
    def publish_frame(self):
        if self.left_cap:
            ret, frame = self.left_cap.read()
            if not ret:
                self.get_logger().warn('Error: Could not read left frame')
                if self.mode == 'simulation':
                    self.left_cap.set(cv2.CAP_PROP_POS_FRAMES, 0)  
                return
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher_.publish(msg)
            self.get_logger().debug('Published frame')
        
        if self.right_cap:
            ret, frame = self.right_cap.read()
            if not ret:
                self.get_logger().warn('Error: Could not read right frame')
                if self.mode == 'simulation':
                    self.right_cap.set(cv2.CAP_PROP_POS_FRAMES, 0)  
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
