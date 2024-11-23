#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class CameraDriver {
public:
    CameraDriver(ros::NodeHandle& nh) : it_(nh) {
        image_pub_ = it_.advertise("/camera/image_raw", 1);
        cap_.open(0);  // Open the default camera
        if (!cap_.isOpened()) {
            ROS_ERROR("Camera not found");
        }
    }

    void publishImage() {
        cv::Mat frame;
        cap_ >> frame;  // Capture image
        if (frame.empty()) return;

        // Convert OpenCV image to ROS image message
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        image_pub_.publish(msg);
    }

private:
    ros::Publisher image_pub_;
    cv::VideoCapture cap_;
    image_transport::ImageTransport it_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "camera_driver_node");
    ros::NodeHandle nh;
    CameraDriver driver(nh);

    ros::Rate loop_rate(10);  // 10 Hz
    while (ros::ok()) {
        driver.publishImage();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
