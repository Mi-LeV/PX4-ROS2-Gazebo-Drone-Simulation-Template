"""
This node controls a servo gripper based on received commands and supports both real hardware operation and simulation mode.

Subscriptions:
   /gripper (std_msgs.msg.String)
       Accepts commands 'open' or 'close' to control the gripper.

Parameters:
    use_sim - Boolean flag to enable simulation mode (default: False)

Hardware Control:
    - Uses PWM on Raspberry Pi GPIO pin 18 to control a servo motor.
    - Converts received commands into servo angles (0° for open, 90° for close).
    - In simulation mode, only logs actions without controlling hardware.

Shutdown Behavior:
    - Cleans up GPIO resources if running on real hardware.

Author: Florian Stanilewicz
Version: 18/02/25
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

import time

class GripperNode(Node):
    def __init__(self):
        super().__init__('gripper_node')
        
        # Declare and get parameters
        self.declare_parameter('use_sim', True)
        self.use_sim = self.get_parameter('use_sim').get_parameter_value().bool_value
        
        # GPIO setup
        self.servo_pin = 18  # Change if needed\


        if not self.use_sim:
            import RPi.GPIO as GPIO

            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.servo_pin, GPIO.OUT)
            self.pwm = GPIO.PWM(self.servo_pin, 50)  # 50Hz PWM frequency
            self.pwm.start(0)
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriber for gripper commands
        self.subscription = self.create_subscription(
            String,
            '/gripper',
            self.gripper_callback,
            qos_profile)
        self.get_logger().info('Gripper Node Started. Waiting for commands on /gripper.')
        
    def gripper_callback(self, msg):
        command = msg.data.lower()
        if command == 'open':
            self.set_servo_angle(0)
            self.get_logger().info('Gripper Opened')
        elif command == 'close':
            self.set_servo_angle(90)
            self.get_logger().info('Gripper close')
        else:
            self.get_logger().warn(f'Unknown command: {command}')
        
    def set_servo_angle(self, angle):
        if self.use_sim:
            self.get_logger().info(f'Simulation mode: Setting servo to {angle} degrees (no hardware action)')
        else:
            duty_cycle = (angle / 18) + 2  # Convert angle to duty cycle
            GPIO.output(self.servo_pin, True)
            self.pwm.ChangeDutyCycle(duty_cycle)
            time.sleep(0.5)
            GPIO.output(self.servo_pin, False)
            self.pwm.ChangeDutyCycle(0)
        
    def destroy_node(self):
        if not self.use_sim:
            self.pwm.stop()
            GPIO.cleanup()
        super().destroy_node()
        

def main():
    rclpy.init()
    node = GripperNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
