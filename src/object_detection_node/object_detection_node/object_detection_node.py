import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        self.subscription = self.create_subscription(
            Image, 'camera_frames', self.process_frame, 10
        )
        self.get_logger().info("Object detection node started.")
        self.bridge = CvBridge()

        # Load pre-trained MobileNet SSD model for human detection
        try:
            self.net = cv2.dnn.readNetFromCaffe(
                'deploy.prototxt',  # Path to the prototxt file
                'mobilenet_iter_73000.caffemodel'  # Path to the Caffe model
            )
            self.get_logger().info("Human detection model loaded.")
        except cv2.error as e:
            self.get_logger().error(f"Failed to load model: {e}")

    def process_frame(self, msg):
        try:
            # Convert ROS image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Preprocess the image
            blob = self.preprocess_image(cv_image)
            
            # Perform human detection
            self.detect_humans(cv_image, blob)

        except Exception as e:
            self.get_logger().error(f"Error processing frame: {e}")

    def preprocess_image(self, cv_image):
        """
        Preprocess the image to prepare it for object detection.
        """
        h, w = cv_image.shape[:2]
        # Prepare image for object detection
        blob = cv2.dnn.blobFromImage(cv_image, 1.0, (300, 300), (104.0, 177.0, 123.0))
        return blob

    def detect_humans(self, cv_image, blob):
        """
        Perform human detection on the preprocessed image.
        """
        self.net.setInput(blob)
        detections = self.net.forward()

        # Get the width and height of the image
        h, w = cv_image.shape[:2]

        self.get_logger().info(f"Detection output shape: {detections.shape}")

        # Loop through all detections and check if human is detected
        for i in range(detections.shape[2]):
            confidence = detections[0, 0, i, 2]
            self.get_logger().info(f"Confidence: {confidence:.2f}")

            if confidence > 0.5:  # Confidence threshold for detection
                box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                (startX, startY, endX, endY) = box.astype("int")
                label = "Human"
                cv2.rectangle(cv_image, (startX, startY), (endX, endY), (0, 255, 0), 2)
                cv2.putText(cv_image, label, (startX, startY - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                self.get_logger().info(f"Human detected with confidence: {confidence:.2f}")

        # Show the image with detections
        cv2.imshow("Detection", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
