import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def image_callback(msg):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")

    # Example image processing: convert to grayscale
    gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    cv2.imshow("Processed Image", gray_image)
    cv2.waitKey(1)

def start_image_processing():
    rospy.init_node('image_processing_node', anonymous=True)
    rospy.Subscriber("/camera/image_raw", Image, image_callback)
    rospy.spin()

if __name__ == '__main__':
    start_image_processing()
