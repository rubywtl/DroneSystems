#ifndef CONTROL_NODE__CONTROL_NODE_HPP_
#define CONTROL_NODE__CONTROL_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <map>
#include <lgpio.h>  // Import lgpio

class ControlNode : public rclcpp::Node {
public:
    ControlNode();

    // Function to validate the angle input (natural number check)
    bool isValidAngle(const std::string& input);

    // Function to set the angle for a specific servo
    void setServoAngle(const std::string& servo_name, int angle);

    // Run method to interact with the user and publish messages
    void run();

    ~ControlNode();  // Destructor for cleaning up GPIO pins

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    std::string mode_;  // Mode of the node (testing or deployment)

    // Map to store servo pins
    std::map<std::string, int> servo_pins_;
};

#endif  // CONTROL_NODE__CONTROL_NODE_HPP_
