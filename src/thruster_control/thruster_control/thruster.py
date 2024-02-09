import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import pigpio
import time

class ThrusterControl(Node):
    def __init__(self):
        super().__init__('thruster_control')
        self.pi = pigpio.pi()
        if not self.pi.connected:
            self.get_logger().error('Not connected to Raspberry Pi GPIO. Exiting...')
            rclpy.shutdown()

        self.joy_subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10)
        
        self.current_pulsewidth = 1500  # Start with neutral pulse width
        self.pi.set_servo_pulsewidth(12, self.current_pulsewidth)  # Assume channel 12 for ESC
        
        self.get_logger().info('Thruster control initialized.')

    def joy_callback(self, msg):
        axis_value = msg.axes[1]
        # Scale the input, for example:
        adjustment = axis_value * 100  # Scale factor for adjusting sensitivity
        new_pulsewidth = max(1000, min(2000, self.current_pulsewidth + adjustment))
        if new_pulsewidth != self.current_pulsewidth:
            self.pi.set_servo_pulsewidth(12, new_pulsewidth)
            self.current_pulsewidth = new_pulsewidth
            self.get_logger().info(f'Updated pulsewidth: {new_pulsewidth}')
        self.delay(0.1);
    
    def delay(self, seconds: float):
        time.sleep(seconds)

def main(args=None):
    rclpy.init(args=args)
    thruster_control = ThrusterControl()
    rclpy.spin(thruster_control)
    thruster_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
