import time
import zenoh
import Adafruit_PCA9685
import json
from constants import MOTOR_PORTS, MOTOR_REVERSED, SERVO_MIN, SERVO_MAX, SERVO_NEUTRAL

class DriveBase:
    def __init__(self, address=0x40, busnum=1):
        self.pwm = Adafruit_PCA9685.PCA9685(address=address, busnum=busnum)
        self.pwm.set_pwm_freq(60)
        
        # Motor configuration
        self.SERVO_MIN = SERVO_MIN
        self.SERVO_MAX = SERVO_MAX
        self.SERVO_NEUTRAL = SERVO_NEUTRAL
        
        # Motor ports and reversals
        self.MOTOR_PORTS = MOTOR_PORTS
        self.MOTOR_REVERSED = MOTOR_REVERSED
        
    def set_motor(self, port, speed):
        """
        Set motor speed (-1.0 to 1.0)
        """
        # Reverse speed if motor is configured as reversed
        for motor_name, motor_port in self.MOTOR_PORTS.items():
            if motor_port == port and self.MOTOR_REVERSED[motor_name]:
                speed = -speed
        
        # Convert -1.0 to 1.0 range to servo pulse range
        pulse = int(self.SERVO_NEUTRAL + (speed * (self.SERVO_MAX - self.SERVO_MIN) / 2))
        pulse = max(self.SERVO_MIN, min(self.SERVO_MAX, pulse))
        self.pwm.set_pwm(port, 0, pulse)
        print(f"Set motor {port} to {pulse}")
    
    def drive(self, x, theta):
        """
        Drive using arcade drive
        x: forward/backward (-1.0 to 1.0)
        theta: rotation (-1.0 to 1.0)
        """
        left = x - theta
        right = x + theta
        
        # Normalize speeds if they exceed [-1, 1]
        max_magnitude = max(abs(left), abs(right))
        if max_magnitude > 1.0:
            left /= max_magnitude
            right /= max_magnitude

        # clip to -0.5 to 0.5
        left = max(-0.25, min(0.25, left))
        right = max(-0.25, min(0.25, right))

        # Set motor speeds
        self.set_motor(self.MOTOR_PORTS['front_left'], left)
        self.set_motor(self.MOTOR_PORTS['back_left'], left)
        self.set_motor(self.MOTOR_PORTS['front_right'], right)
        self.set_motor(self.MOTOR_PORTS['back_right'], right)