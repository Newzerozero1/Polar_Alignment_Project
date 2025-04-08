from gpiozero import OutputDevice
from time import sleep
import os
import threading
import board
import busio
from adafruit_nunchuk import Nunchuk
import queue
import time

# Ensure the stepper_data directory exists
stepper_data_dir = os.path.expanduser("~/stepper_data")
os.makedirs(stepper_data_dir, exist_ok=True)
position_file = os.path.join(stepper_data_dir, "step_position.txt")

# Constants for 1/32 microstepping (original settings)
STEPS_PER_REV = 8000    # 1/32 microstepping
LEAD_SCREW_TRAVEL_PER_REV = 8
STEPS_PER_MM = STEPS_PER_REV / LEAD_SCREW_TRAVEL_PER_REV
MAX_STEPS = 132000      # Original limit

# Define GPIO pins
DIR_PIN = 13
STEP_PIN = 19
ENABLE_PIN = 12
MODE_PIN_1 = 16
MODE_PIN_2 = 17
MODE_PIN_3 = 20

# Set up GPIO
dir_pin = OutputDevice(DIR_PIN)
step_pin = OutputDevice(STEP_PIN)
enable_pin = OutputDevice(ENABLE_PIN)
mode_pin_1 = OutputDevice(MODE_PIN_1)
mode_pin_2 = OutputDevice(MODE_PIN_2)
mode_pin_3 = OutputDevice(MODE_PIN_3)

# Set mode pins for 1/32 microstepping
mode_pin_1.on()
mode_pin_2.on()
mode_pin_3.on()

current_position = 0  # Global position tracking

def initialize_nunchuk():
    global nc
    try:
        i2c = busio.I2C(board.SCL, board.SDA)
        nc = Nunchuk(i2c)
        return True
    except Exception as e:
        print(f"Nunchuk initialization error: {e}")
        return False

def save_position(position):
    with open(position_file, "w") as f:
        f.write(str(position))

def load_position():
    if not os.path.exists(position_file):
        save_position(0)
    try:
        with open(position_file, "r") as f:
            return int(f.read().strip())
    except ValueError:
        return 0

def move_steps(steps, delay, soft_start_end=False):
    global current_position
    step_direction = 1 if steps > 0 else -1
    dir_pin.on() if step_direction > 0 else dir_pin.off()
    sleep(0.005)  # Keep 5ms setup time
    
    enable_pin.on()
    sleep(0.005)
    
    # More dramatic speed calculation based on joystick position
    speed_factor = min(abs(steps) / 100.0, 1.0)  # Normalize to 0-1 range
    min_delay = 0.00005  # 50 microseconds at full deflection
    max_delay = 0.002    # 2000 microseconds near center
    adjusted_delay = max_delay * (1 - speed_factor)  # Inverse relationship
    
    # Normal stepping with speed-based timing
    for step_count in range(abs(steps)):
        if not (-MAX_STEPS <= current_position + step_direction <= MAX_STEPS):
            print("Movement limit reached. Stopping.")
            break
            
        step_pin.on()
        sleep(0.0002)
        step_pin.off()
        sleep(0.0002)
        sleep(adjusted_delay)  # Variable speed delay
        
        current_position += step_direction
        
        if abs(current_position) % 1000 == 0:
            save_position(current_position)

def nunchuk_control():
    global nc, current_position
    print("Nunchuk control thread started")
    
    last_time = time.time()
    
    while True:
        try:
            current_time = time.time()
            x = nc.joystick[0]
            x_adjusted = x - 128
            
            if (current_time - last_time) >= 0.1:
                if abs(x_adjusted) > 30:  # Dead zone
                    # More dramatic step scaling based on position
                    position_factor = abs(x_adjusted) / 128.0
                    base_steps = 50  # Minimum steps at center
                    max_steps = 200   # Maximum steps at full deflection
                    steps = int(base_steps + (position_factor * max_steps))
                    steps = steps if x_adjusted > 0 else -steps
                    
                    if steps != 0:
                        move_steps(steps, 0.0002)
                
                last_time = current_time
            
            sleep(0.02)  # Faster polling
            
        except Exception as e:
            print(f"Nunchuk error: {str(e)}")
            sleep(1)

def main_control():
    global current_position
    current_position = load_position()
    print(f"Starting at position: {current_position}")
    
    print("Motor test - enter numbers for steps")
    
    while True:
        try:
            command_input = input(f"Current position: {current_position} (steps). Enter command: ").strip().lower()
            
            if command_input == 'end':
                break
                
            try:
                steps = int(command_input)
                if -MAX_STEPS <= current_position + steps <= MAX_STEPS:
                    print(f"Moving {steps} steps...")
                    move_steps(steps, 0.0001)  # Changed to 100 microseconds between steps
                else:
                    print("Movement would exceed limits")
            except ValueError:
                print("Invalid input. Use numbers for steps or 'end' to quit")
        except KeyboardInterrupt:
            break

# Start the program
try:
    main_control()
except KeyboardInterrupt:
    print("\nProgram interrupted by user.")
finally:
    enable_pin.off()  # Ensure motor is disabled on exit
    print("Stepper motor driver disabled.")