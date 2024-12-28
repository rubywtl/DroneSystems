#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <iostream>
#include <string>
#include "control_node/control_node.hpp"

ControlNode::ControlNode() : Node("control_node") {
    this->declare_parameter<std::string>("mode", "testing");
    mode_ = this->get_parameter("mode").as_string();

    publisher_ = this->create_publisher<std_msgs::msg::String>("control_commands", 10);
    RCLCPP_INFO(this->get_logger(), "Control node running in '%s' mode", mode_.c_str());
}

bool ControlNode::isValidAngle(const std::string& input) {
    // Check if the input is a valid natural number (non-negative integer)
    if (input.empty()) return false;

    for (char c : input) {
        if (!isdigit(c)) {
            return false;  // Return false if any character is not a digit
        }
    }

    // Convert input to integer and check if it's non-negative
    int angle = std::stoi(input);
    return angle >= 0;  // Valid if non-negative
}

void ControlNode::run() {
    while (rclcpp::ok()) {
        std::string angle;
        bool validInput = false;

        while (!validInput) {
            std::cout << "Enter angle (non-negative integer): ";
            std::cin >> angle;

            // Validate the input
            if (isValidAngle(angle)) {
                validInput = true;  // If valid, break out of the loop
            } else {
                std::cout << "Invalid input. Please enter a non-negative integer for the angle." << std::endl;
            }
        }

        // Create the message with the valid input
        auto message = std_msgs::msg::String();
        message.data = "Angle: " + angle;

        if (mode_ == "testing") {
            RCLCPP_INFO(this->get_logger(), "Simulated output: '%s'", message.data.c_str());
        } else {
            // Send to GPIO in deployment mode
            RCLCPP_INFO(this->get_logger(), "Sent to GPIO: '%s'", message.data.c_str());
            // TODO: send the values to the function, ex: SendToPin(angle);
        }
        
        publisher_->publish(message);
    }
}

/// TODO: functions for actually controlling the output pins
// void ControlNode::SendToPin(int angle)

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControlNode>();
    node->run();  // Call the run function
    rclcpp::shutdown();
    return 0;
}
