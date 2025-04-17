#!/usr/bin/env python3
import os.path
import threading
import time
import board
import busio
import adafruit_nunchuk
import os
import sys
from gpiozero import DigitalOutputDevice

# -------------------------------
# Configuration constants
# -------------------------------

# GPIO pins
DIR_PIN = 13
STEP_PIN = 19
ENABLE_PIN = 12
MODE_PIN_1 = 16
MODE_PIN_2 = 17
MODE_PIN_3 = 20

# Position serialization
STEPPER_DATA_DIR = os.path.expanduser("~/stepper_data")
os.makedirs(STEPPER_DATA_DIR, exist_ok=True)
POSITION_FILE = os.path.join(STEPPER_DATA_DIR, "step_position.txt")

MIN_STEPS = -10000    # Minimum allowed step count
MAX_STEPS = 10000     # Maximum allowed step count

# Motor timing parameters (seconds)
STEP_DELAY = 0.005     

# Nunchuk configuration
JOYSTICK_DEADZONE = 0.2   # deadzone threshold (normalized value between 0 and 1)

# -------------------------------
# Motor Controller Class
# -------------------------------
class StepperMotorController:
    def __init__(self, step_pin, dir_pin, enable_pin, min_steps, max_steps, position_file):
        self.step_pin = DigitalOutputDevice(step_pin)
        self.dir_pin = DigitalOutputDevice(dir_pin)
        self.enable_pin = DigitalOutputDevice(enable_pin)
        self.enable_pin.on()  

        self.min_steps = min_steps
        self.max_steps = max_steps
        self.position_file = position_file
        self.position_lock = threading.Lock()
        self.cancel_event = threading.Event()

        self.current_position = self._load_position()
        self.last_saved_position = self.current_position

        # A lock to ensure only one move command runs at a time.
        self.move_lock = threading.Lock()

    def _load_position(self):
        if os.path.exists(self.position_file):
            try:
                with open(self.position_file, "r") as f:
                    data = f.read().strip()
                    return int(data)
            except ValueError:
                return 0
            except Exception as e:
                print(f"Error reading position file: {e}")
        return 0

    def _save_position(self):
        try:
            with open(self.position_file, "w") as f:
                f.write(str(self.current_position))
            self.last_saved_position = self.current_position
        except Exception as e:
            print(f"Error writing position file: {e}")

    def abort_current_move(self):
        self.cancel_event.set()

    def clear_abort(self):
        self.cancel_event.clear()

    def _move_steps(self, steps, delay):
        direction = 1 if steps > 0 else -1
        self.dir_pin.value = (direction == 1)
        steps_to_move = abs(steps)

        for _ in range(steps_to_move):
            if self.cancel_event.is_set():
                print("Move aborted!")
                break

            # Enforce limits
            with self.position_lock:
                next_position = self.current_position + direction
                if next_position < self.min_steps or next_position > self.max_steps:
                    print("Step limit reached; aborting move")
                    self.cancel_event.set()
                    break

            # Pulse the step pin: high then low
            self.step_pin.on()
            time.sleep(delay / 2)
            self.step_pin.off()
            time.sleep(delay / 2)

            with self.position_lock:
                self.current_position += direction

        self._save_position()

    def move_relative(self, steps, microstep=False):
        # When a new command arrives, abort any ongoing movement.
        with self.move_lock:
            self.abort_current_move()
            # Give time for any ongoing movement to notice the cancel signal
            time.sleep(0.05)
            self.clear_abort()
            self._perform_steps(steps, STEP_DELAY)

            print(f"New position: {self.current_position}")

    def get_position(self):
        with self.position_lock:
            return self.current_position

    def disable_motor(self):
        with self.move_lock:
            self.abort_current_move()
            time.sleep(0.05)
            self.enable_pin.off()
            self.clear_abort()
            print("Motor disabled")

# -------------------------------
# CLI Input Thread
# -------------------------------
class CLIInput(threading.Thread):
    def __init__(self, motor_controller: StepperMotorController):
        super().__init__(daemon=True)
        self.motor_controller = motor_controller

    def run(self):
        print("CLI input thread started")

        while True:
            try:
                line = sys.stdin.readline()
                if not line:
                    break  # EOF
                line = line.strip()
                if not line:
                    continue

                # When a new command comes in, abort any running movement.
                self.motor_controller.abort_current_move()

                tokens = line.split()
                if tokens[0].upper() == "REL":
                    try:
                        steps = int(tokens[1])
                        micro = (len(tokens) > 2 and tokens[2].lower() == "micro")
                        self.motor_controller.move_relative(steps, micro)
                    except Exception as e:
                        print("Error processing relative command:", e)
                elif tokens[0].upper() == "ABS":
                    try:
                        hms = tokens[1]
                        micro = (len(tokens) > 2 and tokens[2].lower() == "micro")
                        self.motor_controller.move_absolute(hms, micro)
                    except Exception as e:
                        print("Error processing absolute command:", e)
                else:
                    print("Unknown command. Use REL or ABS.")
            except Exception as e:
                print("CLI error:", e)
                continue

# -------------------------------
# Wii Nunchuk Input Thread
# -------------------------------
class WiiNunchuk(threading.Thread):
    def __init__(self, motor_controller: StepperMotorController):
        super().__init__(daemon=True)
        self.motor_controller = motor_controller
        self.running = True
        
        i2c = busio.I2C(board.SCL, board.SDA)
        self.nunchuk = adafruit_nunchuk.Nunchuk(i2c)
        print("Nunchuk initialized successfully")

    def run(self):
        print("Wii Nunchuk thread started.")
        poll_interval = 0.1  # seconds
        while self.running:
            try:
                x, y = self.nunchuk.joystick
                z_pressed = self.nunchuk.button_z

                # Normalize joystick values (approximate range: 0-255)
                x_norm = (x - 128) / 128.0  # Centered at 0
                y_norm = (y - 128) / 128.0

                # Deadzone check
                if abs(x_norm) > 0.2:
                    steps = int(10 * (x_norm / abs(x_norm)))  # +10 or -10 steps
                    micro = z_pressed  # Use microstepping if Z button is pressed
                    print(f"Nunchuk command: {steps} steps ({'micro' if micro else 'full'} stepping)")
                    self.motor_controller.abort_current_move()
                    self.motor_controller.move_relative(steps, micro)
            except Exception as e:
                print(f"Nunchuk read error: {e}")
            time.sleep(poll_interval)


# -------------------------------
# Main Application
# -------------------------------
def main():
    motor_controller = StepperMotorController(
        step_pin=STEP_PIN,
        dir_pin=DIR_PIN,
        enable_pin=ENABLE_PIN,
        min_steps=MIN_STEPS,
        max_steps=MAX_STEPS,
        position_file=POSITION_FILE
    )

    # Create the input threads
    cli_thread = CLIInput(motor_controller)
    nunchuk_thread = WiiNunchuk(motor_controller)

    # Start the threads
    cli_thread.start()
    nunchuk_thread.start()

    print("Polar alignment motor control server running. Press Ctrl+C to exit.")

    try:
        # Keep the main thread alive while daemons run.
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Exiting...")
        motor_controller.disable_motor()
        nunchuk_thread.running = False

if __name__ == "__main__":
    main()
