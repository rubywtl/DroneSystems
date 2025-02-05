#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <iostream>
#include <string>
#include <atomic>
#include "control_node/control_node.hpp"
#include <lgpio.h>  // Import lgpio

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

    // Initialize lgpio library
    if (lgInit() < 0) {
        RCLCPP_ERROR(this->get_logger(), "Unable to initialize lgpio.");
        exit(1);
    }

    // Setup GPIO pins
    servo_pins_ = {
        { "servo1", 12 },  // GPIO pin for servo 1
        { "servo2", 13 },  // GPIO pin for servo 2
        { "servo3", 18 }   // GPIO pin for servo 3
    };

    for (const auto& [servo_name, pin] : servo_pins_) {
        lgSetMode(pin, LG_TX_PWM);
        lgWrite(pin, 1500);      // Neutral position (1500 us)
    }
}

void ControlNode::setServoAngle(const std::string& servo_name, int angle) {
    if (servo_pins_.find(servo_name) == servo_pins_.end()) {
        RCLCPP_ERROR(this->get_logger(), "Invalid servo name: %s", servo_name.c_str());
        return;
    }

    int pulse_width = 500 + (angle / 180.0) * 2000;
    lgWrite(servo_pins_[servo_name], pulse_width);
    RCLCPP_INFO(this->get_logger(), "Set %s to angle %d degrees (Pulse width: %d microseconds)", 
                servo_name.c_str(), angle, pulse_width);
}

int main(int argc, char *argv[]) {
    std::signal(SIGINT, signalHandler);  // Register signal handler for Ctrl+C
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControlNode>();
    rclcpp::spin(node);  // Keep the node running
    return 0;
}
