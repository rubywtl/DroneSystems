#ifndef CONTROL_NODE__CONTROL_NODE_HPP_
#define CONTROL_NODE__CONTROL_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

class ControlNode : public rclcpp::Node {
public:
    ControlNode();

    // Function to validate the angle input (natural number check)
    bool isValidAngle(const std::string& input);

    // Run method to interact with the user and publish messages
    void run();

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    std::string mode_;  // Mode of the node (testing or deployment)
};

#endif  // CONTROL_NODE__CONTROL_NODE_HPP_
