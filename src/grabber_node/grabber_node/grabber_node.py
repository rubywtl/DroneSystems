from std_msgs.msg import String, Float32
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

import pigpio
import signal

class GrabberNode(Node):
    def __init__(self):
        super().__init__('control_node')
        
        self.declare_parameter('mode', 'simulation')
        self.mode = self.get_parameter('mode').get_parameter_value().string_value

        self.publisher_ = self.create_publisher(String, 'control_commands', 10)
        self.get_logger().info(f"Control node running in '{self.mode}' mode")

        # Connect to pigpio daemon
        self.pi = pigpio.pi()

        if not self.pi.connected:
            self.get_logger().error("Unable to connect to pigpio daemon.")
            rclpy.shutdown()
            return

        # Setup GPIO pins for servos
        self.servo_pins = {
            "servo1": 12,  # GPIO pin for servo 1
            "servo2": 13,  # GPIO pin for servo 2
            "servo3": 18   # GPIO pin for servo 3
        }

        for servo_name, pin in self.servo_pins.items():
            self.pi.set_mode(pin, pigpio.OUTPUT)
            self.pi.hardware_PWM(pin, 50, 100 * 1000)  # Neutral position (1500 us)

        # ROS 2 subscribers for controlling each servo
        self.subscribers = {
            'servo1': self.create_subscription(Float32, 'servo1_angle', self.servo1_callback, 10),
            'servo2': self.create_subscription(Float32, 'servo2_angle', self.servo2_callback, 10),
            'servo3': self.create_subscription(Float32, 'servo3_angle', self.servo3_callback, 10)
        }

    def set_servo_angle(self, servo_name, angle):
        if servo_name not in self.servo_pins:
            self.get_logger().error(f"Invalid servo name: {servo_name}")
            return

        # Map angle (0-180 degrees) to pulse width (500-2500 microseconds)
        pulse_width = 500 + (angle / 180.0) * 2000
        self.pi.hardware_PWM(self.servo_pins[servo_name], 50, int(pulse_width * 1000))
        self.get_logger().info(f"Set {servo_name} to angle {angle} degrees (Pulse width: {pulse_width} microseconds)")

    def servo1_callback(self, msg):
        self.set_servo_angle('servo1', msg.data)

    def servo2_callback(self, msg):
        self.set_servo_angle('servo2', msg.data)

    def servo3_callback(self, msg):
        self.set_servo_angle('servo3', msg.data)

    def shutdown_node(self):
        self.get_logger().info('Shutting down GrabberNode...')
        for pin in self.servo_pins.values():
            self.pi.hardware_PWM(pin, 0, 0)  # Disable PWM on each pin
        self.pi.stop()  # Stop pigpio connection
        rclpy.shutdown()


def main(args=None):
    signal.signal(signal.SIGINT, lambda signum, frame: (rclpy.shutdown(), exit(0)))
    rclpy.init(args=args)
    node = GrabberNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received. Exiting...')
    finally:
        node.shutdown_node()

if __name__ == '__main__':
    main()
