from gpiozero import OutputDevice
from time import sleep
import os
import board
import busio
from adafruit_nunchuk import Nunchuk
import sys

# Motor constants and GPIO setup (same as original for consistency)
STEPS_PER_REV = 8000    # 1/32 microstepping
MAX_STEPS = 132000      # Limit
DEAD_ZONE = 30          # Joystick dead zone

# GPIO pins
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

# Set 1/32 microstepping mode
mode_pin_1.on()
mode_pin_2.on()
mode_pin_3.on()

def setup_nunchuk():
    """Initialize the Nunchuk"""
    try:
        i2c = busio.I2C(board.SCL, board.SDA)
        nunchuk = Nunchuk(i2c)
        print("Nunchuk initialized successfully")
        return nunchuk
    except Exception as e:
        print(f"Failed to initialize Nunchuk: {e}")
        sys.exit(1)

def move_motor(direction, speed, nunchuk):
    """Move motor based on joystick input with manual control timing"""
    # Set direction with proper setup time
    dir_pin.on() if direction > 0 else dir_pin.off()
    sleep(0.005)  # Keep 5ms setup time
    
    # Enable motor
    enable_pin.on()
    sleep(0.005)  # Setup time for enable
    
    # Adjusted step calculation for smoother motion
    base_steps = 150    # Increased base steps
    max_steps = 450     # Reduced max steps for better control
    steps = max(100, int(base_steps + (speed * max_steps)))  # Higher minimum steps
    
    # Modified timing for smoother operation
    for _ in range(steps):
        step_pin.on()
        sleep(0.0003)  # Increased to 300 microseconds ON
        step_pin.off()
        sleep(0.0003)  # Increased to 300 microseconds OFF
        sleep(0.0003)  # Increased delay between steps
        
        # Check dead zone
        if abs(nunchuk.joystick[0] - 128) < DEAD_ZONE:
            break

def main():
    print("Initializing Wii Nunchuk Stepper Control...")
    nunchuk = setup_nunchuk()
    last_direction = 0
    
    try:
        while True:
            x = nunchuk.joystick[0] - 128
            
            if abs(x) < DEAD_ZONE:
                sleep(0.01)
                continue
            
            direction = 1 if x > 0 else -1
            speed = abs(x) / 128.0  # Simple linear scaling
            
            # Move motor with current settings
            move_motor(direction, speed, nunchuk)
            
            # No extra delay between movements
            
    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
    finally:
        enable_pin.off()
        print("Motor disabled")

if __name__ == "__main__":
    main()