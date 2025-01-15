import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import pigpio

class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller')

        # GPIO pin setup for servos
        self.servo_pins = {
            'servo1': 17,  # GPIO pin for servo 1
            'servo2': 27,  # GPIO pin for servo 2
            'servo3': 22   # GPIO pin for servo 3
        }

        # Initialize pigpio library
        self.pi = pigpio.pi()
        if not self.pi.connected:
            self.get_logger().error("Unable to connect to pigpio daemon.")
            exit(1)

        # Initialize servos to neutral positions (1500 microseconds)
        for pin in self.servo_pins.values():
            self.pi.set_servo_pulsewidth(pin, 1500)

        # ROS 2 subscribers for each servo
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
        self.pi.set_servo_pulsewidth(self.servo_pins[servo_name], pulse_width)
        self.get_logger().info(f"Set {servo_name} to angle {angle} degrees (Pulse width: {pulse_width} microseconds)")

    def servo1_callback(self, msg):
        self.set_servo_angle('servo1', msg.data)

    def servo2_callback(self, msg):
        self.set_servo_angle('servo2', msg.data)

    def servo3_callback(self, msg):
        self.set_servo_angle('servo3', msg.data)

    def destroy_node(self):
        # Turn off servos before exiting
        for pin in self.servo_pins.values():
            self.pi.set_servo_pulsewidth(pin, 0)  # 0 disables PWM on the pin
        self.pi.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ServoController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()