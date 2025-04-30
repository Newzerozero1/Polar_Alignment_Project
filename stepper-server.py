import os
import sys
import threading
import logging
from time import sleep, time
from queue import Queue, Empty

import RPi.GPIO as GPIO
import board
import busio
from adafruit_nunchuk import Nunchuk

# Configure logging
logging.basicConfig(level=logging.INFO,
                    format='%(asctime)s [%(levelname)s] %(message)s',
                    datefmt='%Y-%m-%d %H:%M:%S')
logger = logging.getLogger(__name__)

class StepperController:
    # GPIO pin definitions
    DIR_PIN = 13
    STEP_PIN = 19
    ENABLE_PIN = 12
    MODE_PINS = (16, 17, 20)
    PWM_PIN = 18

    # Motion constants
    STEPS_PER_REV = 800
    MAX_STEPS = 1_320_000
    DEAD_ZONE = 10

    # Conversion constants
    LEAD_SCREW_TRAVEL_PER_REV = 8  # mm

    # File path for position persistence
    DATA_DIR = os.path.expanduser("~/stepper_data")
    POSITION_FILE = os.path.join(DATA_DIR, "step_position.txt")

    def __init__(self):
        self._lock = threading.Lock()
        self.current_position = 0
        self.motor_enabled = False

        # Setup persistence directory
        os.makedirs(self.DATA_DIR, exist_ok=True)
        self.current_position = self._load_position()

        # GPIO setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.DIR_PIN, GPIO.OUT)
        GPIO.setup(self.STEP_PIN, GPIO.OUT)
        GPIO.setup(self.ENABLE_PIN, GPIO.OUT)
        for pin in self.MODE_PINS:
            GPIO.setup(pin, GPIO.OUT)
        GPIO.setup(self.PWM_PIN, GPIO.OUT)

        # Microstepping mode: 1/4
        GPIO.output(self.MODE_PINS[0], GPIO.LOW)
        GPIO.output(self.MODE_PINS[1], GPIO.HIGH)
        GPIO.output(self.MODE_PINS[2], GPIO.LOW)

        # PWM for holding current
        self.pwm = GPIO.PWM(self.PWM_PIN, 1000)
        self.pwm.start(0)

        # Derived constants
        self.steps_per_mm = (self.STEPS_PER_REV * 10 * 0.96) / self.LEAD_SCREW_TRAVEL_PER_REV
        self.arcsec_per_mm = (3600 * 360) / (self.LEAD_SCREW_TRAVEL_PER_REV * 45)
        self.steps_per_arcsec = self.steps_per_mm / self.arcsec_per_mm

        logger.info("StepperController initialized at position %d", self.current_position)

    def _save_position(self):
        with open(self.POSITION_FILE, 'w') as f:
            f.write(str(self.current_position))

    def _load_position(self):
        if not os.path.exists(self.POSITION_FILE):
            return 0
        try:
            with open(self.POSITION_FILE) as f:
                return int(f.read())
        except ValueError:
            return 0

    def enable(self):
        with self._lock:
            if not self.motor_enabled:
                GPIO.output(self.ENABLE_PIN, GPIO.HIGH)
                self.motor_enabled = True
                logger.debug("Motor enabled")

    def disable(self):
        with self._lock:
            if self.motor_enabled:
                GPIO.output(self.ENABLE_PIN, GPIO.LOW)
                self.motor_enabled = False
                self._save_position()
                logger.debug("Motor disabled, position saved: %d", self.current_position)

    def move_motor(self, direction: int, speed: float, half_speed: bool = False):
        """
        Move a fixed batch of steps based on joystick input.
        """
        with self._lock:
            self.enable()
            GPIO.output(self.DIR_PIN, GPIO.HIGH if direction > 0 else GPIO.LOW)
            sleep(0.001)

            base_delay = 0.001
            if half_speed:
                delay = (base_delay / (0.1 + speed * 0.9)) * 2
            else:
                delay = base_delay / (0.1 + speed * 0.9)

            delay = max(min(delay, 0.02), 0.0002)
            pulse = delay / 2

            # Batch of steps
            batch = 50
            for _ in range(batch):
                if abs(self.current_position + direction) > self.MAX_STEPS:
                    logger.warning("Position limit reached at %d", self.current_position)
                    break
                GPIO.output(self.STEP_PIN, GPIO.HIGH)
                sleep(pulse)
                GPIO.output(self.STEP_PIN, GPIO.LOW)
                sleep(pulse)
                self.current_position += direction

    def move_steps(self, steps: int, speed: float = 1.0):
        with self._lock:
            self.enable()
            direction = 1 if steps > 0 else -1
            GPIO.output(self.DIR_PIN, GPIO.HIGH if direction > 0 else GPIO.LOW)
            delay = max(0.0001, 0.001 / speed)
            pulse = delay / 2
            for _ in range(abs(steps)):
                if abs(self.current_position + direction) > self.MAX_STEPS:
                    logger.warning("Limit reached during manual move at %d", self.current_position)
                    break
                GPIO.output(self.STEP_PIN, GPIO.HIGH)
                sleep(pulse)
                GPIO.output(self.STEP_PIN, GPIO.LOW)
                sleep(pulse)
                self.current_position += direction
            logger.info("Manual move complete, position: %d", self.current_position)
            self.disable()

    def reset_position(self):
        """Home to zero position."""
        with self._lock:
            steps_to_zero = -self.current_position
            if steps_to_zero == 0:
                logger.info("Already at zero position")
                return
            logger.info("Resetting position: %d steps to zero", steps_to_zero)
            self.enable()
            direction = 1 if steps_to_zero > 0 else -1
            GPIO.output(self.DIR_PIN, GPIO.HIGH if direction > 0 else GPIO.LOW)
            for _ in range(abs(steps_to_zero)):
                GPIO.output(self.STEP_PIN, GPIO.HIGH)
                sleep(0.0001)
                GPIO.output(self.STEP_PIN, GPIO.LOW)
                sleep(0.002)
                self.current_position += direction
            self.current_position = 0
            logger.info("Homing complete, position set to zero")
            self.disable()

    def arc_to_steps(self, arcmin_sec: float) -> int:
        arcmin = int(abs(arcmin_sec))
        arcsec = (abs(arcmin_sec) - arcmin) * 100
        total_sec = arcmin * 60 + arcsec
        steps = int(round(total_sec * self.steps_per_arcsec))
        return steps if arcmin_sec >= 0 else -steps

    def steps_to_arc(self, steps: int):
        arcseconds = abs(steps) / self.steps_per_arcsec
        arcminutes = int(arcseconds // 60)
        arcsec = arcseconds % 60
        return arcminutes, arcsec

    def shutdown(self):
        self.disable()
        self.pwm.stop()
        GPIO.cleanup()
        logger.info("GPIO cleaned up and PWM stopped")


class NunchukInputThread(threading.Thread):
    def __init__(self, controller: StepperController, stop_event: threading.Event):
        super().__init__(daemon=True)
        self.controller = controller
        self.stop_event = stop_event
        self.nunchuk = self._init_nunchuk()
        self.last_z = 0
        self.last_c = 0

    def _init_nunchuk(self):
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            return Nunchuk(i2c)
        except Exception as e:
            logger.error("Failed to initialize Nunchuk: %s", e)
            sys.exit(1)

    def run(self):
        logger.info("Nunchuk thread started")
        while not self.stop_event.is_set():
            x, = self.nunchuk.joystick[:1]
            x -= 128
            buttons = self.nunchuk.buttons
            z, c = buttons.Z, buttons.C

            # Exit both on Z+C
            if z and c:
                logger.info("Z+C pressed, stopping application")
                self.stop_event.set()
                break

            # Double-tap Z to reset
            now = time()
            if z:
                if self.last_z and now - self.last_z < 0.5:
                    logger.info("Double Z detected, resetting")
                    self.controller.reset_position()
                    self.last_z = 0
                else:
                    self.last_z = now

            # Double-tap C to toggle half-speed
            if c:
                if self.last_c and now - self.last_c < 0.5:
                    self.half_speed = not getattr(self, 'half_speed', False)
                    logger.info("Speed toggled to %s", 'half' if self.half_speed else 'full')
                    self.last_c = 0
                else:
                    self.last_c = now

            # Joystick movement
            if abs(x) < self.controller.DEAD_ZONE:
                self.controller.disable()
            else:
                direction = 1 if x > 0 else -1
                speed = abs(x) / 128.0
                half = getattr(self, 'half_speed', False)
                self.controller.move_motor(direction, speed, half)

            sleep(0.05)

        logger.info("Nunchuk thread exiting")


class CLIInputThread(threading.Thread):
    def __init__(self, controller: StepperController, stop_event: threading.Event):
        super().__init__(daemon=True)
        self.controller = controller
        self.stop_event = stop_event

    def run(self):
        logger.info("CLI thread started")
        while not self.stop_event.is_set():
            try:
                cmd = input("Enter command> ").strip().lower()
            except EOFError:
                self.stop_event.set()
                break

            if cmd in ('end', 'exit'):
                self.stop_event.set()
                break
            elif cmd in ('gt0', 'go to zero'):
                self.controller.reset_position()
            elif cmd == 'reset':
                confirm = input("Reset position to 0? (y/n)> ").strip().lower()
                if confirm == 'y':
                    self.controller.current_position = 0
                    self.controller._save_position()
                    logger.info("Position manually reset to 0")
            else:
                try:
                    # Manual arcmin.arcsec input
                    speed = 0.5
                    parts = cmd.split()
                    if len(parts) == 2 and parts[0] in ('s', 'slow'):
                        speed = 0.1
                        arc = float(parts[1])
                    else:
                        arc = float(parts[0])
                    steps = self.controller.arc_to_steps(arc)
                    logger.info("CLI moving %d steps at speed %.2f", steps, speed)
                    self.controller.move_steps(steps, speed)
                except Exception as e:
                    logger.error("Unrecognized command '%s': %s", cmd, e)

        logger.info("CLI thread exiting")


def main():
    controller = StepperController()
    stop_event = threading.Event()

    # Launch input threads
    nunchuk_thread = NunchukInputThread(controller, stop_event)
    cli_thread = CLIInputThread(controller, stop_event)
    nunchuk_thread.start()
    cli_thread.start()

    # Wait for CLI to finish (end/exit)
    cli_thread.join()
    # Signal nunchuk to stop
    stop_event.set()
    nunchuk_thread.join()

    controller.shutdown()
    logger.info("Application terminated")


if __name__ == '__main__':
    main()