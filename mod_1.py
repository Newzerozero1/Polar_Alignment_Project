import RPi.GPIO as GPIO
from time import sleep, time
import board
import busio
from adafruit_nunchuk import Nunchuk
import sys
import threading
import os

# Motor constants and GPIO setup
STEPS_PER_REV = 800    # Changed from 200 to 800 for 1/4 microstepping
MAX_STEPS = 1320000      # Limit
DEAD_ZONE = 10          # Joystick dead zone

# GPIO pins
DIR_PIN = 13
STEP_PIN = 19
ENABLE_PIN = 12 # PWM controls enable
MODE_PIN_1 = 16
MODE_PIN_2 = 17
MODE_PIN_3 = 20
PWM_PIN = 18 # PWM pin for reduced holding current

# Set up GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(DIR_PIN, GPIO.OUT)
GPIO.setup(STEP_PIN, GPIO.OUT)
GPIO.setup(ENABLE_PIN, GPIO.OUT) # PWM controls enable
GPIO.setup(MODE_PIN_1, GPIO.OUT)
GPIO.setup(MODE_PIN_2, GPIO.OUT)
GPIO.setup(MODE_PIN_3, GPIO.OUT)
GPIO.setup(PWM_PIN, GPIO.OUT) # PWM pin setup

# Set 1/4 step mode (Mode1=LOW, Mode2=HIGH, Mode3=LOW)
GPIO.output(MODE_PIN_1, GPIO.LOW)
GPIO.output(MODE_PIN_2, GPIO.HIGH)
GPIO.output(MODE_PIN_3, GPIO.LOW)

# PWM setup
PWM_FREQUENCY = 1000 # Increased PWM frequency
pwm = GPIO.PWM(PWM_PIN, PWM_FREQUENCY) # frequency
pwm.start(0) # Start with 0% duty cycle

# Current Limit (adjust to 80% of rated current)
CURRENT_LIMIT = 1.6 # Amps (80% of 2.0A)

# Ensure the stepper_data directory exists
stepper_data_dir = os.path.expanduser("~/stepper_data")
os.makedirs(stepper_data_dir, exist_ok=True)
position_file = os.path.join(stepper_data_dir, "step_position.txt")

# Constants for conversion
STEPS_PER_REV_CALC = int(800 * 10 * 0.96)  # Changed from 200 to 800 for 1/4 microstepping
LEAD_SCREW_TRAVEL_PER_REV = 8  # 8mm per full revolution
STEPS_PER_MM = STEPS_PER_REV_CALC / LEAD_SCREW_TRAVEL_PER_REV  # Steps per mm
ARCSECONDS_PER_MM = (3600 * 360) / (LEAD_SCREW_TRAVEL_PER_REV * 45)  # Arcseconds per mm
STEPS_PER_ARCSECOND = STEPS_PER_MM / ARCSECONDS_PER_MM  # Steps per arcsecond

# Function to save the current position
def save_position(position):
    with open(position_file, "w") as f:
        f.write(str(position))

# Function to load the current position from the file
def load_position():
    if not os.path.exists(position_file):
        save_position(0)  # Create the file with default position 0 if missing
    try:
        with open(position_file, "r") as f:
            return int(f.read().strip())
    except ValueError:
        return 0  # Default position if file content is invalid

current_position = load_position()

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

def move_motor(direction, speed, speed_half):
    """Move motor based on joystick input using the same timing as manual mode"""
    global current_position
    
    # Set direction with proper setup time
    GPIO.output(DIR_PIN, GPIO.HIGH if direction > 0 else GPIO.LOW)
    sleep(0.001)  # Reduced setup time

    # Use the EXACT SAME delay calculation as in move_steps
    manual_base_delay = 0.001  # Same base delay as in move_steps
    
    if speed_half:
        # Half speed - ensure it moves by limiting the maximum delay
        # First calculate the delay as if it were full speed
        full_speed_value = 0.1 + (speed * 0.9)  # Normal full speed calculation
        delay = manual_base_delay / full_speed_value
        
        # Reduce multiplier for 1/4 microstepping to get more reasonable half-speed
        delay = delay * 2  # Reduced from 3 to 2 for better motion with microstepping
    else:
        # Full speed - use full speed value with minimum (unchanged)
        effective_speed = 0.1 + (speed * 0.9)  # Range: 0.1 to 1.0
        delay = manual_base_delay / effective_speed

    # Apply minimum/maximum delay constraints
    min_manual_delay = 0.0002  # Increased from 0.0001 for more reliable operation
    max_manual_delay = 0.02    # Reduced from 0.03 for better half-speed motion
    
    if delay < min_manual_delay:
        delay = min_manual_delay
    elif delay > max_manual_delay:
        delay = max_manual_delay

    # Calculate pulse duration exactly like in move_steps
    pulse_duration = delay / 2
    
    # For smoother operation with microstepping, reduce batch size to make movement more responsive
    steps = 50  # Reduced from 100 for more responsive movement
    
    for _ in range(steps):
        # Check position limit
        if abs(current_position + direction) > MAX_STEPS:
            print("Position limit reached!")
            return
            
        # Generate step using EXACT SAME timing as move_steps
        GPIO.output(STEP_PIN, GPIO.HIGH)
        sleep(pulse_duration)  # Same as in move_steps
        GPIO.output(STEP_PIN, GPIO.LOW)
        sleep(pulse_duration)  # Same as in move_steps
        
        # Update position counter
        current_position += direction

def move_steps(steps, speed):
    """Move a specific number of steps for manual input"""
    global current_position
    step_direction = 1 if steps > 0 else -1
    GPIO.output(DIR_PIN, GPIO.HIGH if step_direction > 0 else GPIO.LOW)

    GPIO.output(ENABLE_PIN, GPIO.HIGH)
    sleep(0.001)

    # FASTER manual mode
    manual_base_delay = 0.001  # Back to original value
    delay = manual_base_delay / speed  # Original speed calculation
    min_manual_delay = 0.0001  # Back to original value
    if delay < min_manual_delay:
        delay = min_manual_delay

    pulse_duration = delay / 2
    for _ in range(abs(steps)):
        if abs(current_position + step_direction) > MAX_STEPS:
            print("Position limit reached during manual move!")
            break

        GPIO.output(STEP_PIN, GPIO.HIGH)
        sleep(pulse_duration)
        GPIO.output(STEP_PIN, GPIO.LOW)
        sleep(pulse_duration)
        current_position += step_direction

    GPIO.output(ENABLE_PIN, GPIO.LOW)
    save_position(current_position)
    print(f"\nManual move finished. Final position: {current_position}")

def reset_position():
    """Move motor to position 0 with slower speed"""
    global current_position
    
    # Save the initial position to calculate steps correctly
    initial_position = current_position
    steps_to_zero = -initial_position
    
    if steps_to_zero == 0:
        print("Motor is already at position 0.")
        return
    
    print(f"Moving motor to position 0 ({abs(steps_to_zero)} steps)...")
    steps_to_move = abs(steps_to_zero)
    
    # Use a fixed, reliable approach
    GPIO.output(DIR_PIN, GPIO.HIGH if steps_to_zero > 0 else GPIO.LOW)
    GPIO.output(ENABLE_PIN, GPIO.HIGH)
    sleep(0.01)  # Short setup time
    
    # Simple, reliable parameters
    delay = 0.002  # Fixed delay for reliable operation
    step_direction = 1 if steps_to_zero > 0 else -1
    
    print("Starting move to position 0...")
    
    # Move in simple steps with fixed delay
    for i in range(steps_to_move):
        # Extra check to ensure we stop at exactly zero
        if (step_direction > 0 and current_position >= 0) or (step_direction < 0 and current_position <= 0):
            print(f"Reached or passed position 0! Current position: {current_position}")
            break
            
        # Check position limit
        if abs(current_position + step_direction) > MAX_STEPS:
            print("Position limit reached during move to zero!")
            break
            
        # Simple step generation
        GPIO.output(STEP_PIN, GPIO.HIGH)
        sleep(0.0001)  # Fixed pulse width
        GPIO.output(STEP_PIN, GPIO.LOW)
        sleep(delay)  # Fixed delay between steps
        
        # Update position
        current_position += step_direction
        
        # Show progress occasionally
        if i % 1000 == 0 and i > 0:
            print(f"Homing progress: {i}/{steps_to_move} steps completed")
    
    # Force position to be exactly zero to prevent drifting
    current_position = 0
            
    GPIO.output(ENABLE_PIN, GPIO.LOW)
    save_position(current_position)
    print("Motor is now at position 0.")

# Convert arcminutes and arcseconds to steps
def arc_to_steps(arcmin_sec):
    arcmin = int(abs(arcmin_sec))  # Extract arcminutes
    arcsec = (abs(arcmin_sec) - arcmin) * 100  # Extract arcseconds
    total_arcseconds = (arcmin * 60) + arcsec
    steps = int(round(total_arcseconds * STEPS_PER_ARCSECOND))
    return steps if arcmin_sec >= 0 else -steps  # Preserve direction

# Convert steps to arcminutes and arcseconds
def steps_to_arc(steps):
    arcseconds = abs(steps) / STEPS_PER_ARCSECOND
    arcminutes = int(arcseconds // 60)
    arcseconds = arcseconds % 60
    return arcminutes, arcseconds

def main():
    print("Initializing Wii Nunchuk Stepper Control...")
    nunchuk = setup_nunchuk()
    last_z_press_time = 0
    last_c_press_time = 0
    speed_half = False
    global current_position

    try:
        while True:
            GPIO.output(ENABLE_PIN, GPIO.LOW)
            command_input = input(f"Current position: {current_position} (steps). Enter arcmin.arcsec (e.g., '1.23' for manual control, 'j' for joystick, 'gt0' or 'go to zero', 'reset' to reset position: ").strip().lower()

            if command_input == 'end':
                break
            if command_input in ['gt0', 'go to zero']:
                reset_position()
                continue
            if command_input == 'reset':
                reset_prompt = input("Are you sure you want to reset the current position to 0? (y/n): ").strip().lower()
                if reset_prompt == 'y':
                    current_position = 0
                    save_position(current_position)
                    print("Current position has been reset to 0.")
                else:
                    print("Current position reset cancelled.")
                continue
                
            # Go back to the original non-threaded joystick control
            if command_input == 'j':
                print("Entering Joystick Control Mode... Press Ctrl+C to exit.")
                motor_enabled = False
                try:
                    # Main joystick control loop
                    while True:
                        x = nunchuk.joystick[0] - 128
                        buttons = nunchuk.buttons
                        z_button = buttons.Z
                        c_button = buttons.C

                        # Exit on Z+C press
                        if z_button and c_button:
                            print("\n*** Z+C buttons pressed - Exiting program ***")
                            GPIO.output(ENABLE_PIN, GPIO.LOW)
                            pwm.stop()
                            GPIO.cleanup()
                            save_position(current_position)
                            sys.exit(0)

                        # Handle Z button (double-tap to reset)
                        if z_button:
                            current_time = time()
                            if last_z_press_time != 0 and (current_time - last_z_press_time) < 0.5:
                                print("Double Tap Z Detected! Resetting position.")
                                if motor_enabled:
                                    GPIO.output(ENABLE_PIN, GPIO.LOW)
                                    motor_enabled = False
                                reset_position()
                                last_z_press_time = 0
                            else:
                                last_z_press_time = current_time
                            while nunchuk.buttons.Z:
                                sleep(0.01)
                                
                        # Handle C button (double-tap to toggle speed)
                        if c_button:
                            current_time = time()
                            if last_c_press_time != 0 and (current_time - last_c_press_time) < 0.5:
                                speed_half = not speed_half
                                print(f"Double Tap C Detected! Speed is now {'half' if speed_half else 'full'}")
                                last_c_press_time = 0
                            else:
                                last_c_press_time = current_time
                            while nunchuk.buttons.C:
                                sleep(0.01)
                        
                        # Go back to simple, working joystick movement with your original move_motor
                        if abs(x) < DEAD_ZONE:
                            if motor_enabled:
                                GPIO.output(ENABLE_PIN, GPIO.LOW)
                                motor_enabled = False
                                save_position(current_position)
                            sleep(0.05)
                        else:
                            if not motor_enabled:
                                GPIO.output(ENABLE_PIN, GPIO.HIGH) 
                                motor_enabled = True
                                sleep(0.001)
                            
                            direction = 1 if x > 0 else -1
                            joystick_value = abs(x)
                            normalized = joystick_value / 128.0
                            speed = normalized
                            move_motor(direction, speed, speed_half)
                
                except KeyboardInterrupt:
                    print("\nExiting Joystick Control Mode...")
                    if motor_enabled:
                        GPIO.output(ENABLE_PIN, GPIO.LOW)
                    print(f"Exited Joystick mode. Current position: {current_position}")
                    continue
            try:
                parts = command_input.split()
                manual_speed_setting = 0.5
                if len(parts) == 1:
                    target_arcmin_sec = float(parts[0])
                elif len(parts) == 2 and parts[0] in ['s', 'slow']:
                    manual_speed_setting = 0.1
                    target_arcmin_sec = float(parts[1])
                else:
                    raise ValueError("Invalid command format")
                steps_to_move = arc_to_steps(target_arcmin_sec)
                print(f"Manual command: Move {steps_to_move} steps at speed setting {manual_speed_setting}")
                move_steps(steps_to_move, manual_speed_setting)
            except ValueError as e:
                print(f"Invalid input: {e}. Please enter a valid command format.")
                continue
    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
    finally:
        GPIO.output(ENABLE_PIN, GPIO.LOW)
        pwm.stop()
        GPIO.cleanup()
        save_position(current_position)
        print("Motor disabled, GPIO cleaned up.")

if __name__ == "__main__":
    main()