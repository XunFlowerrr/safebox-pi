#!/usr/bin/env python3
"""
Continuous Rotation Servo Testing Script
This script provides functions to control continuous rotation servos
"""

from gpiozero import Servo
from time import sleep

# Servo configuration
SERVO_PIN = 18  # GPIO pin for servo control

class ContinuousServoController:
    def __init__(self, pin=SERVO_PIN):
        """Initialize continuous servo controller"""
        self.servo = Servo(pin)
        
    def stop(self):
        """Stop the servo (center position)"""
        self.servo.value = 0
        print("Servo stopped")
        
    def rotate_clockwise(self, speed=0.5):
        """Rotate clockwise at specified speed (0.0-1.0)"""
        if speed < 0:
            speed = 0
        elif speed > 1:
            speed = 1
        self.servo.value = speed
        print(f"Rotating clockwise at {speed*100}% speed")
        
    def rotate_counterclockwise(self, speed=0.5):
        """Rotate counterclockwise at specified speed (0.0-1.0)"""
        if speed < 0:
            speed = 0
        elif speed > 1:
            speed = 1
        self.servo.value = -speed
        print(f"Rotating counterclockwise at {speed*100}% speed")
        
    def rotate_for_time(self, direction, speed=0.5, duration=2.0):
        """Rotate for a specific duration then stop"""
        if direction.lower() in ['cw', 'clockwise']:
            self.rotate_clockwise(speed)
        elif direction.lower() in ['ccw', 'counterclockwise']:
            self.rotate_counterclockwise(speed)
        else:
            print("Invalid direction. Use 'cw' or 'ccw'")
            return
            
        sleep(duration)
        self.stop()
        print(f"Rotated {direction} for {duration} seconds")
     
    def release(self):
        """Release GPIO resources"""
        self.servo.close()
        
if __name__ == "__main__":
    print("Continuous Servo Testing Script")
    print("===============================")
    print("Commands:")
    print("  stop - Stop rotation")
    print("  cw [speed] - Rotate clockwise (speed: 0.0-1.0)")
    print("  ccw [speed] - Rotate counterclockwise (speed: 0.0-1.0)")
    print("  time [cw/ccw] [speed] [duration] - Rotate for specific time")
    print("  exit/quit - Exit program")
    
    servo = ContinuousServoController()
    
    try:
        while True:
            user_input = input("\nEnter command: ").strip().lower().split()
            
            if not user_input:
                continue
                
            command = user_input[0]
            
            if command in ['exit', 'quit', 'q']:
                print("Exiting...")
                break
            elif command == 'stop':
                servo.stop()
            elif command == 'cw':
                speed = float(user_input[1]) if len(user_input) > 1 else 0.5
                servo.rotate_clockwise(speed)
            elif command == 'ccw':
                speed = float(user_input[1]) if len(user_input) > 1 else 0.5
                servo.rotate_counterclockwise(speed)
            elif command == 'time':
                try:
                    direction = user_input[1] if len(user_input) > 1 else 'cw'
                    speed = float(user_input[2]) if len(user_input) > 2 else 0.5
                    duration = float(user_input[3]) if len(user_input) > 3 else 2.0
                    servo.rotate_for_time(direction, speed, duration)
                except (IndexError, ValueError):
                    print("Usage: time [cw/ccw] [speed] [duration]")
            else:
                print("Unknown command. Type 'exit' to quit.")
                
    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
    except Exception as e:
        print(f"Unexpected error: {e}")
    finally:
        servo.stop()  # Make sure to stop before releasing
        servo.release()
        print("GPIO resources released")