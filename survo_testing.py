#!/usr/bin/env python3
"""
SG90 Tower Pro Micro Servo Testing Script
This script is optimized for the SG90 9g micro servo with precise timing
"""

import time
import gpiod

# SG90 Tower Pro Micro Servo specific configuration
SERVO_PIN = 18  # GPIO pin for servo control
PWM_FREQUENCY = 50  # Standard servo frequency (50Hz)
MIN_PULSE_WIDTH = 0.5  # Minimum pulse width in ms (0 degrees) - SG90 optimized
MAX_PULSE_WIDTH = 2.4  # Maximum pulse width in ms (180 degrees) - SG90 optimized
NEUTRAL_PULSE_WIDTH = 1.45  # Neutral position pulse width in ms (90 degrees) - SG90 optimized
CYCLE_TIME = 20  # 20ms cycle time for 50Hz
SIGNAL_REPEATS = 50  # Number of signal repeats for SG90 stability

class SG90ServoController:
    def __init__(self, pin=SERVO_PIN):
        """Initialize SG90 servo controller with specified GPIO pin"""
        self.pin = pin
        self.chip = gpiod.Chip('gpiochip0')
        self.line = self.chip.get_line(pin)
        self.line.request(consumer="SG90_SERVO", type=gpiod.LINE_REQ_DIR_OUT)
        self.current_angle = None
        print(f"SG90 Tower Pro Micro Servo initialized on GPIO pin {pin}")
        
    def set_angle(self, angle):
        """
        Set SG90 servo to specific angle (0-180 degrees)
        Optimized for SG90 timing characteristics
        """
        if angle < 0:
            angle = 0
        elif angle > 180:
            angle = 180
            
        # SG90-specific pulse width calculation
        pulse_width = MIN_PULSE_WIDTH + (angle / 180.0) * (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH)
        
        # Calculate timing for precise PWM
        period = CYCLE_TIME / 1000.0  # Convert to seconds
        duty_cycle = pulse_width / 1000.0  # Convert ms to seconds
        off_time = period - duty_cycle
        
        # Send PWM signal optimized for SG90
        for _ in range(SIGNAL_REPEATS):
            self.line.set_value(1)
            time.sleep(duty_cycle)
            self.line.set_value(0)
            time.sleep(off_time)
            
        self.current_angle = angle
        
    def sweep_test(self):
        """
        Perform a sweep test optimized for SG90 servo
        Tests the full range of motion with proper delays
        """
        print("Starting SG90 servo sweep test...")
        positions = [0, 45, 90, 135, 180, 135, 90, 45, 0]
        
        for angle in positions:
            print(f"Moving to {angle} degrees...")
            self.set_angle(angle)
            time.sleep(0.8)  # SG90 needs ~0.6s to move 180°, add safety margin
            
        print("Sweep test completed!")
        
    def center_servo(self):
        """Center the servo to 90 degrees (neutral position)"""
        print("Centering servo to 90 degrees...")
        self.set_angle(90)

    def cleanup(self):
        """Clean up GPIO resources"""
        self.line.release()
        self.chip.close()

if __name__ == "__main__":
    print("SG90 Tower Pro Micro Servo Testing Script")
    print("=========================================")
    print("Optimized for SG90 9g micro servo")
    print("Enter angles (0-180 degrees) to control the servo")
    print("Commands:")
    print("  'test' or 'sweep' - run a sweep test")
    print("  'center' - move to 90 degrees")
    print("  'quit' or 'exit' - stop the program")
    
    servo = None
    try:
        # Initialize SG90 servo controller
        servo = SG90ServoController()
        
        # Center servo on startup
        servo.center_servo()
        time.sleep(1)
        
        while True:
            try:
                user_input = input("\nEnter angle (0-180) or command: ").strip().lower()
                
                # Check for exit commands
                if user_input in ['quit', 'exit', 'q']:
                    print("Exiting servo control...")
                    break
                
                # Check for test command
                if user_input in ['test', 'sweep']:
                    servo.sweep_test()
                    continue
                    
                # Check for center command
                if user_input in ['center', 'c']:
                    servo.center_servo()
                    continue
                
                # Try to parse angle input
                try:
                    angle = float(user_input)
                    if angle < 0 or angle > 180:
                        print("Error: Angle must be between 0 and 180 degrees")
                        continue
                    
                    print(f"Setting SG90 servo to {angle} degrees...")
                    servo.set_angle(angle)
                    print("Done!")
                    
                except ValueError:
                    print("Error: Please enter a valid number between 0 and 180")
                    print("Valid commands: 'test', 'sweep', 'center', 'quit', 'exit'")
                    
            except KeyboardInterrupt:
                print("\nReceived Ctrl+C, exiting...")
                break
                
    except Exception as e:
        print(f"Error initializing SG90 servo: {e}")
        print("Make sure you're running this script with appropriate permissions")
        print("You may need to run with sudo for GPIO access")
        print("Check that the SG90 servo is properly connected to GPIO pin 18")
        
    finally:
        if servo:
            servo.cleanup()
            print("GPIO resources cleaned up")
       