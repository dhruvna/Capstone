# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Joy
# import pigpio
# import time

# class ThrusterControl(Node):
#     def __init__(self):
#         super().__init__('thruster_control')
#         self.pi = pigpio.pi()
#         if not self.pi.connected:
#             self.get_logger().error('Not connected to Raspberry Pi GPIO. Exiting...')
#             rclpy.shutdown()

#         self.joy_subscription = self.create_subscription(
#             Joy,
#             '/joy',
#             self.joy_callback,
#             10)
        
#         self.current_pulsewidth = 1500  # Start with neutral pulse width
#         self.pi.set_servo_pulsewidth(12, self.current_pulsewidth)  # Assume channel 12 for ESC
        
#         self.get_logger().info('Thruster control initialized.')

#     def joy_callback(self, msg):
#         axis_value = msg.axes[1]
#         # Scale the input, for example:
#         adjustment = axis_value * 100  # Scale factor for adjusting sensitivity
#         new_pulsewidth = max(1000, min(2000, self.current_pulsewidth + adjustment))
#         if new_pulsewidth != self.current_pulsewidth:
#             self.pi.set_servo_pulsewidth(12, new_pulsewidth)
#             self.current_pulsewidth = new_pulsewidth
#             self.get_logger().info(f'Updated pulsewidth: {new_pulsewidth}')
#         self.delay(0.1);
    
#     def delay(self, seconds: float):
#         time.sleep(seconds)

# def main(args=None):
#     rclpy.init(args=args)
#     thruster_control = ThrusterControl()
#     rclpy.spin(thruster_control)
#     self.pi.set_servo_pulsewidth(12, 1500)
#     thruster_control.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Joy
import pigpio

class ThrusterControl(Node):
    def __init__(self):
        super().__init__('thruster_control')
        self.pi = pigpio.pi() # connect to pi
        if not self.pi.connected:
            self.get_logger().error('Not connected to Raspberry Pi GPIO. Exiting...')
            rclpy.shutdown()

        self.joy_subscription = self.create_subscription( # subscribes to the joystick input from joy node
            Joy,
            '/joy',
            self.joy_callback,
            10,
            callback_group=ReentrantCallbackGroup())
        
        self.current_pulsewidth = 1500  # Initial startup
        self.delay(0.7)
        self.axis_value = 0  # Gets and stores latest joystick position
        self.pi.set_servo_pulsewidth(12, self.current_pulsewidth)  # gpio pin is 12
        
        self.timer = self.create_timer(0.1, self.timer_callback)  # Adjust PWM vaue at most every 0.1 seconds
        
        self.get_logger().info('Thruster control initialized.')

    def joy_callback(self, msg):
        # Sets axis to be vertical joystick axis only
        self.axis_value = msg.axes[1]

    def timer_callback(self):
        # Use the stored axis value to adjust PWM value
        adjustment = self.axis_value * 10  # Smaller scale factor for smoother adjustment
        new_pulsewidth = max(1100, min(1900, self.current_pulsewidth + adjustment))
        if new_pulsewidth != self.current_pulsewidth: #updates PWM value if joystick moved
            self.pi.set_servo_pulsewidth(12, new_pulsewidth)
            self.current_pulsewidth = new_pulsewidth
            self.get_logger().info(f'Updated pulsewidth: {new_pulsewidth}')

def main(args=None):
    rclpy.init(args=args)
    thruster_control = ThrusterControl()
    executor = MultiThreadedExecutor()
    rclpy.spin(thruster_control, executor=executor)
    thruster_control.pi.set_servo_pulsewidth(12, 1500)  # Reset to neutral on shutdown
    thruster_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
