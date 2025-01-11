#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <iostream>
#include <string>
#include <csignal>
#include <atomic>
#include "control_node/control_node.hpp"

std::atomic<bool> keep_running(true);

void signalHandler(int signum) {
    keep_running = false;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Signal (%d) received. Shutting down...", signum);
    rclcpp::shutdown();
}

ControlNode::ControlNode() : Node("control_node") {
    this->declare_parameter<std::string>("mode", "simulation");
    mode_ = this->get_parameter("mode").as_string();

    publisher_ = this->create_publisher<std_msgs::msg::String>("control_commands", 10);
    RCLCPP_INFO(this->get_logger(), "Control node running in '%s' mode", mode_.c_str());
}

bool ControlNode::isValidAngle(const std::string& input) {
    if (input.empty()) return false;

    for (char c : input) {
        if (!isdigit(c)) {
            return false;
        }
    }
    int angle = std::stoi(input);
    return angle >= 0;  // Valid if non-negative
}

void ControlNode::run() {
    while (rclcpp::ok() && keep_running) {
        std::string angle;
        bool validInput = false;

        while (!validInput && keep_running) {
            std::cout << "Enter angle (non-negative integer): ";
            std::cin >> angle;

            if (isValidAngle(angle)) {
                validInput = true;
            } else {
                std::cout << "Invalid input. Please enter a non-negative integer for the angle." << std::endl;
            }
        }

        if (!keep_running) break;

        auto message = std_msgs::msg::String();
        message.data = "Angle: " + angle;

        if (mode_ == "simulation") {
            RCLCPP_INFO(this->get_logger(), "Simulated output: '%s'", message.data.c_str());
        } else {
            RCLCPP_INFO(this->get_logger(), "Sent to GPIO: '%s'", message.data.c_str());
            // TODO: send the values to the function, e.g., SendToPin(angle);
        }

        publisher_->publish(message);
    }
}

int main(int argc, char *argv[]) {
    std::signal(SIGINT, signalHandler);  // Register signal handler for Ctrl+C
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControlNode>();
    node->run();
    rclcpp::spin(node);  // Keep the node running
    return 0;
}
