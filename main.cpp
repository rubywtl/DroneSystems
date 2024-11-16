#include <iostream>
#include <thread>
#include <atomic>
#include <chrono>
// #include <opencv2/opencv.hpp>
#include <unistd.h>

// Mock motor control class
class MotorControl {
public:
    MotorControl() : motor_angle(0) {}
    
    // Function to simulate motor angle changes
    void moveMotor(int angle) {
        std::cout << "Motor moving to angle: " << angle << std::endl;
        motor_angle = angle;
    }
    
    // Function to run motor control logic (can be extended)
    void controlMotor() {
        while (true) {
            if (motor_angle != last_angle) {
                moveMotor(motor_angle);
                last_angle = motor_angle;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }

    // Function to update motor angle (to be called by main loop)
    void setMotorAngle(int angle) {
        motor_angle = angle;
    }

private:
    int motor_angle;      // Current motor angle
    int last_angle = -1;  // Last motor angle to detect changes
};

// Camera control class for calling Python script
class CameraControl {
public:
    void startImageProcessing() {
        while (true) {
            // Simulate Python camera processing by calling Python script
            std::cout << "Calling Python script for image processing..." << std::endl;
            system("python3 camera_processing.py");
            std::this_thread::sleep_for(std::chrono::seconds(1));  // Control interval for camera processing
        }
    }
};

// Main control loop
int main() {
    MotorControl motorControl;
    CameraControl cameraControl;

    // Start motor control in a separate thread
    std::thread motor_thread(&MotorControl::controlMotor, &motorControl);

    // Start image processing in a separate thread
    std::thread camera_thread(&CameraControl::startImageProcessing, &cameraControl);

    // Main loop to simulate motor angle updates
    int angle = 0;
    while (true) {
        motorControl.setMotorAngle(angle);
        angle = (angle + 10) % 180;  // Simulate motor angle updates (0 to 180 degrees)
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }

    // Join threads before exiting
    motor_thread.join();
    camera_thread.join();

    return 0;
}
