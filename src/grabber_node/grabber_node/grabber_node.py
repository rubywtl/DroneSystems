from std_msgs.msg import String
import rclpy
from rclpy.node import Node
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

        # Setup GPIO pins
        self.servo_pins = {
            "servo1": 12,
            "servo2": 13, 
            "servo3": 18   
        }

        for servo_name, pin in self.servo_pins.items():
            self.pi.set_mode(pin, pigpio.OUTPUT)
            self.pi.hardware_PWM(pin, 50, 1500 * 1000)  # Neutral position (1500 us)

    def set_servo_angle(self, servo_name, angle):
        if servo_name not in self.servo_pins:
            self.get_logger().error(f"Invalid servo name: {servo_name}")
            return

        pulse_width = 500 + (angle / 180.0) * 2000
        self.pi.hardware_PWM(self.servo_pins[servo_name], 50, int(pulse_width * 1000))
        self.get_logger().info(f"Set {servo_name} to angle {angle} degrees (Pulse width: {pulse_width} microseconds)")

    def shutdown_node(self):
        self.pi.stop()  # Ensure the pigpio connection is stopped when shutting down
        rclpy.shutdown()

def main(args=None):
    signal.signal(signal.SIGINT, lambda signum, frame: (rclpy.shutdown(), exit(0)))
    rclpy.init(args=args)
    node = GrabberNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
