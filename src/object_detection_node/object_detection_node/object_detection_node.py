import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import cv2
import numpy as np

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        self.subscription = self.create_subscription(
            Image, 'camera_frames', self.process_frame, 10
        )
        self.get_logger().info("Object detection node started.")
        self.bridge = CvBridge()

        # Instead of loading a model, inform the user that no model is loaded
        self.get_logger().info("No model loaded. Using placeholder processing.")

    def process_frame(self, msg):
        try:
            # Convert ROS image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Placeholder for object detection
            self.detect_humans(cv_image)

        except Exception as e:
            self.get_logger().error(f"Error processing frame: {e}")

    def detect_humans(self, cv_image):
        """
        Perform placeholder human detection.
        """
        # Placeholder logic: Draw a rectangle at the center of the image
        h, w = cv_image.shape[:2]
        startX, startY = w // 2 - 50, h // 2 - 50
        endX, endY = w // 2 + 50, h // 2 + 50
        
        # Draw a placeholder rectangle to simulate detection
        cv2.rectangle(cv_image, (startX, startY), (endX, endY), (0, 255, 0), 2)
        cv2.putText(cv_image, "Placeholder Human", (startX, startY - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Print a message for human detection
        self.get_logger().info(f"Human detected at ({startX}, {startY}) with dimensions ({endX-startX}, {endY-startY})")

        # Placeholder for further processing or saving the image
        # This can be processed further without displaying in a GUI

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
