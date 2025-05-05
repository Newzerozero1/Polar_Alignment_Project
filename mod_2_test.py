import os
import sys
import threading
import logging
from time import sleep, time
import RPi.GPIO as GPIO
import board
import busio
from adafruit_nunchuk import Nunchuk
import select

# Configure logging
logging.basicConfig(level=logging.INFO,
                    format='%(asctime)s [%(levelname)s] %(message)s',
                    datefmt='%Y-%m-%d %H:%M:%S')
logger = logging.getLogger(__name__)

class StepperController:
    # GPIO pin definitions - EXACTLY match mod_1
    DIR_PIN = 13
    STEP_PIN = 19
    ENABLE_PIN = 12
    MODE_PIN_1 = 16
    MODE_PIN_2 = 17
    MODE_PIN_3 = 20
    PWM_PIN = 18

    # Motion constants - EXACTLY match mod_1
    STEPS_PER_REV = 800  # For 1/4 microstepping
    MAX_STEPS = 1_320_000
    DEAD_ZONE = 10
    
    # PWM frequency - REDUCED even further for smoother operation
    PWM_FREQUENCY = 100  # Changed from 500Hz to 100Hz for smoother operation
    
    # Conversion constants - EXACTLY match mod_1
    LEAD_SCREW_TRAVEL_PER_REV = 8  # mm
    
    # File path - EXACTLY match mod_1
    DATA_DIR = os.path.expanduser("~/stepper_data")
    POSITION_FILE = os.path.join(DATA_DIR, "step_position.txt")

    def __init__(self):
        self._lock = threading.Lock()
        self.current_position = 0
        self.motor_enabled = False
        self.last_joystick_dir = 0
        self.half_speed_mode = False

        # Setup persistence directory
        os.makedirs(self.DATA_DIR, exist_ok=True)
        self.current_position = self._load_position()
        
        # Reset GPIO completely first
        GPIO.cleanup()

        # GPIO setup - EXACTLY match mod_1
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.DIR_PIN, GPIO.OUT)
        GPIO.setup(self.STEP_PIN, GPIO.OUT)
        GPIO.setup(self.ENABLE_PIN, GPIO.OUT)
        GPIO.setup(self.MODE_PIN_1, GPIO.OUT)
        GPIO.setup(self.MODE_PIN_2, GPIO.OUT)
        GPIO.setup(self.MODE_PIN_3, GPIO.OUT)
        GPIO.setup(self.PWM_PIN, GPIO.OUT)

        # Microstepping mode: 1/4 - EXACTLY match mod_1
        GPIO.output(self.MODE_PIN_1, GPIO.LOW)
        GPIO.output(self.MODE_PIN_2, GPIO.HIGH)
        GPIO.output(self.MODE_PIN_3, GPIO.LOW)
        logger.info("Set to 1/4 step mode like mod_1")

        # PWM setup - EXACTLY match mod_1
        self.pwm = GPIO.PWM(self.PWM_PIN, self.PWM_FREQUENCY)
        self.pwm.start(0)
        logger.info("PWM initialized at 0% duty cycle")

        # Enable PWM for smoother operation
        self.pwm.start(80)  # 80% duty cycle for holding current
        logger.info("PWM enabled at 80% for smoother motion")

        # Derived constants - EXACTLY match mod_1
        self.STEPS_PER_REV_CALC = int(800 * 10 * 0.96)
        self.STEPS_PER_MM = self.STEPS_PER_REV_CALC / self.LEAD_SCREW_TRAVEL_PER_REV
        self.ARCSECONDS_PER_MM = (3600 * 360) / (self.LEAD_SCREW_TRAVEL_PER_REV * 45)
        self.STEPS_PER_ARCSECOND = self.STEPS_PER_MM / self.ARCSECONDS_PER_MM

        logger.info("StepperController initialized at position %d", self.current_position)
        
        # CRITICAL: Set initial state of enable pin - EXACTLY match mod_1
        GPIO.output(self.ENABLE_PIN, GPIO.LOW)
        logger.info("Motor initially disabled (like mod_1)")

        # Initialize the continuous movement thread
        self.continuous_active = False
        self.continuous_direction = 0
        self.continuous_delay = 0.01
        self.movement_thread = threading.Thread(target=self._continuous_movement_loop)
        self.movement_thread.daemon = True
        self.movement_thread.start()
        logger.info("Continuous movement thread initialized")

    def _save_position(self):
        """Save current position to file."""
        try:
            with open(self.POSITION_FILE, 'w') as f:
                f.write(str(self.current_position))
        except Exception as e:
            logger.error("Failed to save position: %s", e)

    def _load_position(self):
        """Load saved position from file."""
        try:
            with open(self.POSITION_FILE, 'r') as f:
                return int(f.read().strip())
        except ValueError:
            return 0
        except FileNotFoundError:
            return 0
        except Exception as e:
            logger.error("Failed to load position: %s", e)
            return 0

    def move_motor(self, direction: int, speed: float, half_speed: bool = False):
        """EXACTLY match mod_1's joystick control function with improved smoothness"""
        with self._lock:
            # We don't need to enable/disable repeatedly - only change state when direction changes
            # Store last direction as a class variable (add to __init__: self.last_joystick_dir = 0)
            
            # Only toggle enable if we weren't already enabled or direction changed
            if not hasattr(self, 'last_joystick_dir') or direction != self.last_joystick_dir:
                # CRITICAL: In mod_1, joystick mode uses OPPOSITE enable logic from manual mode
                GPIO.output(self.ENABLE_PIN, GPIO.HIGH)  # This ENABLES in joystick mode
                sleep(0.001)
                
                # Set direction only when changed
                GPIO.output(self.DIR_PIN, GPIO.HIGH if direction > 0 else GPIO.LOW)
                sleep(0.001)  # Same as mod_1
                
                # Update last direction
                self.last_joystick_dir = direction

            # Use EXACTLY the same delay calculation as in mod_1
            manual_base_delay = 0.001
            
            if half_speed:
                # Half speed calculation - EXACTLY match mod_1
                full_speed_value = 0.1 + (speed * 0.9)
                delay = manual_base_delay / full_speed_value
                delay = delay * 2
            else:
                # Full speed calculation - EXACTLY match mod_1
                effective_speed = 0.1 + (speed * 0.9)
                delay = manual_base_delay / effective_speed

            # Apply same constraints as mod_1
            min_manual_delay = 0.0002
            max_manual_delay = 0.02
            
            if delay < min_manual_delay:
                delay = min_manual_delay
            elif delay > max_manual_delay:
                delay = max_manual_delay

            # Calculate pulse duration EXACTLY like in mod_1
            pulse_duration = delay / 2
            
            # Use a MUCH larger batch size for smoother motion - closer to manual mode
            steps = 50  # Large batch for continuous motion
            
            logger.info(f"Moving with joystick: dir={direction}, delay={delay*1000:.2f}ms, ENABLE=HIGH")
            
            # Use continuous step generation for smoother motion
            for _ in range(steps):
                # Check position limit
                if abs(self.current_position + direction) > self.MAX_STEPS:
                    logger.warning("Position limit reached!")
                    return
                    
                # Generate step with exact mod_1 timing
                GPIO.output(self.STEP_PIN, GPIO.HIGH)
                sleep(pulse_duration)
                GPIO.output(self.STEP_PIN, GPIO.LOW)
                sleep(pulse_duration)
                
                # Update position counter
                self.current_position += direction

            # DON'T disable the motor or change PWM after each batch - this creates the jitter
            # Leave the motor enabled until direction changes or joystick returns to center

    def move_steps(self, steps: int, speed: float = 1.0):
        """EXACTLY match mod_1's manual move function with proper stopping"""
        with self._lock:
            step_direction = 1 if steps > 0 else -1
            GPIO.output(self.DIR_PIN, GPIO.HIGH if step_direction > 0 else GPIO.LOW)

            # CRITICAL: Enable logic is OPPOSITE for manual mode compared to joystick
            GPIO.output(self.ENABLE_PIN, GPIO.HIGH)  # This ENABLES in manual mode
            sleep(0.001)

            # Increase current during manual movement
            self.pwm.ChangeDutyCycle(90)

            # EXACTLY match mod_1's timing
            manual_base_delay = 0.001
            delay = manual_base_delay / speed
            min_manual_delay = 0.0001
            if delay < min_manual_delay:
                delay = min_manual_delay

            pulse_duration = delay / 2
            logger.info(f"Manual move: {steps} steps, delay={delay*1000:.2f}ms")
            
            target_position = self.current_position + steps
            steps_moved = 0
            
            for _ in range(abs(steps)):
                # Check if we've reached the target
                if (step_direction > 0 and self.current_position >= target_position) or \
                   (step_direction < 0 and self.current_position <= target_position):
                    logger.info(f"Target position reached at {self.current_position}")
                    break

                if abs(self.current_position + step_direction) > self.MAX_STEPS:
                    logger.warning("Position limit reached during manual move!")
                    break

                GPIO.output(self.STEP_PIN, GPIO.HIGH)
                sleep(pulse_duration)
                GPIO.output(self.STEP_PIN, GPIO.LOW)
                sleep(pulse_duration)
                self.current_position += step_direction
                steps_moved += 1
                
                # Add progress indicator for long moves
                if steps_moved % 500 == 0:
                    logger.info(f"Manual move progress: {steps_moved}/{abs(steps)} steps")

    def reset_position(self):
        """Reset current position to zero without moving."""
        with self._lock:
            old_pos = self.current_position
            self.current_position = 0
            self._save_position()
            logger.info(f"Position reset from {old_pos} to 0")

    def arc_to_steps(self, arcmin_sec: float) -> int:
        """Convert from arc minutes.seconds to steps."""
        arcmin = int(abs(arcmin_sec))
        arcsec = (abs(arcmin_sec) - arcmin) * 100
        total_sec = arcmin * 60 + arcsec
        steps = int(round(total_sec * self.STEPS_PER_ARCSECOND))
        return steps if arcmin_sec >= 0 else -steps

    def steps_to_arc(self, steps: int):
        """Convert from steps to arc minutes and seconds."""
        arcseconds = abs(steps) / self.STEPS_PER_ARCSECOND
        arcminutes = int(arcseconds // 60)
        arcsec = arcseconds % 60
        return arcminutes, arcsec

    def get_position_arc(self):
        """Get current position in arc format."""
        arcmin, arcsec = self.steps_to_arc(self.current_position)
        sign = '-' if self.current_position < 0 else ''
        return f"{sign}{arcmin}'{arcsec:.2f}\""

    def shutdown(self):
        """Clean shutdown of controller."""
        GPIO.output(self.ENABLE_PIN, GPIO.LOW)  # DISABLE with LOW like mod_1
        self.pwm.stop()
        self._save_position()  # Ensure position is saved
        GPIO.cleanup()
        logger.info("GPIO cleaned up and PWM stopped")

    def start_continuous_movement(self, direction, speed, half_speed=False):
        """Start continuous movement with improved settings that eliminate grinding"""
        with self._lock:
            # Check if we're already moving in this direction
            if (hasattr(self, 'continuous_active') and 
                self.continuous_active and 
                self.continuous_direction == direction and
                self.half_speed_mode == half_speed):  # Also check if speed mode changed
                return  # Skip unnecessary updates
            
            # Store half-speed mode for reference
            self.half_speed_mode = half_speed
            
            # Adjust PWM based on speed mode
            if half_speed:
                # Setup for half speed mode to eliminate buzzing
                self.pwm.ChangeDutyCycle(50)  # Dramatically lower duty cycle for slow mode
                
                # Use much higher microstepping for slow mode
                GPIO.output(self.MODE_PIN_1, GPIO.HIGH)  # 1/16 step mode
                GPIO.output(self.MODE_PIN_2, GPIO.HIGH)
                GPIO.output(self.MODE_PIN_3, GPIO.HIGH)
                
                # Sleep briefly to let settings take effect
                sleep(0.01)
            else:
                # Full speed keeps 1/4 stepping with high duty cycle
                self.pwm.ChangeDutyCycle(85)
                
                # Standard 1/4 step mode for full speed
                GPIO.output(self.MODE_PIN_1, GPIO.LOW)
                GPIO.output(self.MODE_PIN_2, GPIO.HIGH)
                GPIO.output(self.MODE_PIN_3, GPIO.LOW)
            
            # Enable the motor
            GPIO.output(self.ENABLE_PIN, GPIO.HIGH)
            
            # Set direction
            GPIO.output(self.DIR_PIN, GPIO.HIGH if direction > 0 else GPIO.LOW)
            
            # Calculate timing using manual mode's proven timing approach
            delay = self._calculate_manual_delay(speed, half_speed)
            
            # Store movement parameters
            self.continuous_direction = direction
            self.continuous_delay = delay
            self.continuous_active = True
            
            # Log the start
            logger.info(f"Starting movement: dir={direction}, delay={delay*1000:.2f}ms, {'HALF' if half_speed else 'FULL'} speed")
            
            # Use more gradual ramp-up for smoother start (like in manual mode)
            for i in range(8):
                # Progressively shorter delays for smooth acceleration
                pulse_delay = max(delay, 0.002 - (i * 0.0002))
                GPIO.output(self.STEP_PIN, GPIO.HIGH)
                sleep(pulse_delay / 2)  # Divide by 2 like in manual mode
                GPIO.output(self.STEP_PIN, GPIO.LOW)
                sleep(pulse_delay / 2)
                self.current_position += direction

    def _calculate_manual_delay(self, speed, half_speed=False):
        """Calculate delay using the manual mode's timing approach which doesn't grind"""
        # Use much faster base delay for full speed mode
        manual_base_delay = 0.0005  # Reduced from 0.001 for 2x faster operation
        
        if half_speed:
            # Half speed calculation
            speed_value = max(0.1, min(1.0, speed))
            effective_speed = 0.1 + (speed_value * 0.9)
            delay = (manual_base_delay / effective_speed) * 4.0  # Multiply by 4x from the new base
        else:
            # Full speed calculation - now 2x faster
            speed_value = max(0.1, min(1.0, speed))
            effective_speed = 0.1 + (speed_value * 0.9)
            delay = manual_base_delay / effective_speed
        
        # Apply tighter constraints for faster operation
        min_delay = 0.00015  # Reduced from 0.0002 for faster maximum speed
        max_delay = 0.01    # Maximum delay for slow mode
        
        if delay < min_delay:
            delay = min_delay
        elif delay > max_delay:
            delay = max_delay
        
        return delay

    def stop_continuous_movement(self):
        """Stop the continuous movement with proper cleanup"""
        with self._lock:
            if self.continuous_active:  # Fixed this line - removed the duplicate "active:"
                # Disable continuous movement flag
                self.continuous_active = False
                # Disable motor
                GPIO.output(self.ENABLE_PIN, GPIO.LOW)
                # Always reset microstepping mode to 1/4 (standard mode)
                GPIO.output(self.MODE_PIN_1, GPIO.LOW)  # 1/4 step mode
                GPIO.output(self.MODE_PIN_2, GPIO.HIGH)
                GPIO.output(self.MODE_PIN_3, GPIO.LOW)
                
                # Reset PWM to holding current
                self.pwm.ChangeDutyCycle(80)
                logger.info("Movement stopped")

    def _continuous_movement_loop(self):
        """Background thread that keeps stepping as long as continuous mode is active."""
        logger.info("Continuous movement thread is running!")
        last_step_time = 0
        steps_taken = 0
        while True:
            # Quick check if we should be moving (without lock for better performance)
            if not hasattr(self, 'continuous_active') or not self.continuous_active:
                sleep(0.005)  # Short sleep when inactive
                continue
            
            # Get timing parameters once, with minimal lock time
            with self._lock:
                if not self.continuous_active:
                    continue
                            
                direction = self.continuous_direction
                delay = self.continuous_delay
            
            current_time = time()
            time_since_last = current_time - last_step_time
            
            # Only step if enough time has passed since last step
            if time_since_last >= delay:
                with self._lock:
                    if self.continuous_active:
                        # Check position limits
                        if abs(self.current_position + direction) > self.MAX_STEPS:
                            logger.warning("Position limit reached - stopping")
                            self.continuous_active = False
                            GPIO.output(self.ENABLE_PIN, GPIO.LOW)
                            continue
                        
                        # Use manual mode's pulse pattern with equal high/low times
                        pulse_time = delay / 2  # Same as manual mode's pulse_duration calculation
                        
                        # Generate step with manual mode's timing
                        GPIO.output(self.STEP_PIN, GPIO.HIGH)
                        sleep(pulse_time)
                        GPIO.output(self.STEP_PIN, GPIO.LOW)
                        sleep(pulse_time)
                        
                        self.current_position += direction
                        last_step_time = time()
                        steps_taken += 1
                        
                        # Less frequent logging to avoid slowing down stepping
                        if steps_taken % 500 == 0:
                            logger.info(f"Continuous movement: {steps_taken} steps taken")
            else:
                # Sleep just enough time for next step
                sleep_time = max(0.0001, delay - time_since_last)
                sleep(sleep_time)

    def go_to_zero(self):
        """Move the motor to the physical zero position"""
        with self._lock:
            # First stop any ongoing movement
            if hasattr(self, 'continuous_active'):
                self.continuous_active = False
            
            # Get current position
            old_pos = self.current_position
            
            # Move the opposite direction to reach zero
            steps_to_move = -old_pos
            
            # Log the operation
            logger.info(f"Zeroing: Moving {steps_to_move} steps from current position {old_pos}")
            
            # Set direction
            direction = 1 if steps_to_move > 0 else -1
            GPIO.output(self.DIR_PIN, GPIO.HIGH if direction > 0 else GPIO.LOW)
            
            # Enable motor
            GPIO.output(self.ENABLE_PIN, GPIO.HIGH)
            sleep(0.01)
            
            # Use standard settings for zeroing
            self.pwm.ChangeDutyCycle(85)
            
            # Standard 1/4 step mode
            GPIO.output(self.MODE_PIN_1, GPIO.LOW)
            GPIO.output(self.MODE_PIN_2, GPIO.HIGH)
            GPIO.output(self.MODE_PIN_3, GPIO.LOW)
            
            # Calculate delay time
            manual_base_delay = 0.001
            speed = 0.3  # Use faster speed for zeroing
            delay = manual_base_delay / speed
            pulse_duration = delay / 2
            
            # Now move the steps
            steps_moved = 0
            for _ in range(abs(steps_to_move)):
                GPIO.output(self.STEP_PIN, GPIO.HIGH)
                sleep(pulse_duration)
                GPIO.output(self.STEP_PIN, GPIO.LOW)
                sleep(pulse_duration)
                
                # Update position (instead of letting movement thread do it)
                self.current_position += direction
                steps_moved += 1
                
                # Show progress
                if steps_moved % 500 == 0:
                    logger.info(f"Zeroing progress: {steps_moved}/{abs(steps_to_move)} steps")
            
            # Force the position to exactly zero
            self.current_position = 0
            self._save_position()
            
            # Disable the motor
            GPIO.output(self.ENABLE_PIN, GPIO.LOW)
            
            logger.info("Zeroing complete - position is now 0")

class NunchukInputThread(threading.Thread):
    def __init__(self, controller: StepperController, stop_event: threading.Event):
        super().__init__(daemon=True)
        self.controller = controller
        self.stop_event = stop_event
        self.nunchuk = self._init_nunchuk()
        self.last_z = 0
        self.last_c = 0
        self.last_x = 0

    def _init_nunchuk(self):
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            for attempt in range(3):
                try:
                    nunchuk = Nunchuk(i2c)
                    test_value = nunchuk.joystick[0]
                    logger.info("Successfully initialized Nunchuk: test value %d", test_value)
                    return nunchuk
                except Exception as e:
                    logger.warning("Nunchuk init attempt %d failed: %s", attempt+1, e)
                    sleep(1)
            logger.error("Failed to initialize Nunchuk after multiple attempts")
            return None
        except Exception as e:
            logger.error("Critical Nunchuk I2C error: %s", e)
            return None

    def run(self):
        logger.info("Nunchuk thread started")
        if self.nunchuk is None:
            logger.error("Nunchuk not available - joystick control disabled")
            while not self.stop_event.is_set():
                sleep(1)
            return
        half_speed = False
        while not self.stop_event.is_set():
            try:
                x = self.nunchuk.joystick[0] - 128
                buttons = self.nunchuk.buttons
                z_button = buttons.Z
                c_button = buttons.C

                # Check for Z+C combination (exit program)
                if z_button and c_button:
                    logger.info("Z+C pressed together - initiating shutdown")
                    self.stop_event.set()
                    break

                # Z button (double-tap to reset position)
                if z_button:
                    current_time = time()
                    if self.last_z != 0 and (current_time - self.last_z) < 0.5:
                        logger.info("Double tap Z detected - zeroing position")
                        # Use the dedicated zeroing function
                        self.controller.go_to_zero()
                        
                        # Reset last_z to prevent repeated triggers
                        self.last_z = 0
                        
                        # Wait for button release to prevent immediate re-trigger
                        while self.nunchuk.buttons.Z and not self.stop_event.is_set():
                            sleep(0.05)
                    else:
                        self.last_z = current_time
                        # Wait for button release
                        while self.nunchuk.buttons.Z and not self.stop_event.is_set():
                            sleep(0.01)
                else:
                    # Clear last_z if button has been released long enough
                    if self.last_z != 0 and (time() - self.last_z) > 0.7:
                        self.last_z = 0

                # C button (double-tap to toggle speed)
                if c_button:
                    current_time = time()
                    if self.last_c != 0 and (current_time - self.last_c) < 0.5:
                        half_speed = not half_speed
                        logger.info("Double tap C detected - speed now %s", "half" if half_speed else "full")
                        
                        # Wait for button release to avoid multiple toggles
                        self.last_c = 0
                        while self.nunchuk.buttons.C and not self.stop_event.is_set():
                            sleep(0.05)
                    else:
                        self.last_c = current_time
                        # Wait for button release
                        while self.nunchuk.buttons.C and not self.stop_event.is_set():
                            sleep(0.01)
                else:
                    # Clear last_c if button has been released long enough
                    if self.last_c != 0 and (time() - self.last_c) > 0.7:
                        self.last_c = 0

                # Joystick movement with fixed speeds (no variable speed)
                current_x = x  # Store current position
                if abs(current_x) < self.controller.DEAD_ZONE:
                    # Only stop if we were previously moving
                    if abs(self.last_x) >= self.controller.DEAD_ZONE:
                        self.controller.stop_continuous_movement()
                    self.last_x = current_x
                else:
                    # Outside deadzone - we're moving
                    direction = 1 if current_x > 0 else -1
                    # Use fixed speed regardless of joystick position
                    speed = 0.9  # Increased from 0.7 for faster operation
                    
                    # Start new movement if direction changed or we're starting
                    start_new = False
                    if abs(self.last_x) < self.controller.DEAD_ZONE:
                        # Starting from center
                        start_new = True
                    elif (current_x * self.last_x) < 0:
                        # Direction changed
                        start_new = True
                        
                    # Update movement if needed
                    if start_new:
                        self.controller.start_continuous_movement(direction, speed, half_speed)
                                        
                    # Always update last position
                    self.last_x = current_x

                sleep(0.01)  # Small sleep to avoid CPU hogging
            except Exception as e:
                logger.error("Error in Nunchuk thread: %s", e)
                sleep(0.1)  # Sleep on error to avoid rapid looping
        
        logger.info("Nunchuk thread exiting")

class CLIInputThread(threading.Thread):
    def __init__(self, controller: StepperController, stop_event: threading.Event):
        super().__init__(daemon=True)
        self.controller = controller
        self.stop_event = stop_event

    def input_with_timeout(self, prompt, timeout=10):
        """Get input with timeout."""
        print(prompt, end='', flush=True)
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            return sys.stdin.readline().strip()
        print("\nInput timed out")
        return None

    def run(self):
        logger.info("CLI thread started")
        self.show_help()
        while not self.stop_event.is_set():
            try:
                position_arc = self.controller.get_position_arc()
                cmd = self.input_with_timeout(
                    f"Position: {self.controller.current_position} steps ({position_arc})\n> ")
                if cmd is None:  # Timeout
                    continue
                self.process_command(cmd.lower())
            except EOFError:
                self.stop_event.set()
            except Exception as e:
                logger.error("Input error: %s", e)

    def show_help(self):
        print("\nStepper Motor Control Commands:")
        print("------------------------------")
        print("  <number>       - Move specified arc minutes.seconds (e.g., 1.30 = 1'30\")")
        print("  s <number>     - Move at slow speed (e.g., s 1.30)")
        print("  gt0, go to zero - Return to zero position")
        print("  reset          - Reset current position to zero")
        print("  hardware       - Run comprehensive hardware test")
        print("  help           - Show this help")
        print("  test           - Run motor test sequence")
        print("  mod1           - Run mod_1 test sequence")
        print("  end, exit, quit - Exit program")
        print("  Joystick is always active")
        print("  Double-tap Z button to reset position")
        print("  Double-tap C button to toggle speed")
        print("  Z+C together to exit program")
        print("------------------------------")

    def process_command(self, cmd):
        if not cmd:
            return
        if cmd in ['help', '?']:
            self.show_help()
            return
        if cmd in ['end', 'exit', 'quit']:
            logger.info("Exit command received")
            self.stop_event.set()
            return
        if cmd in ['gt0', 'go to zero']:
            self.controller.go_to_zero()
            return
        if cmd == 'reset':
            confirm = self.input_with_timeout(
                "Are you sure you want to reset the current position to 0? (y/n): ")
            if confirm and confirm.lower() == 'y':
                self.controller.reset_position()
            else:
                logger.info("Reset cancelled")
            return
        if cmd == 'test':
            self.run_test_sequence()
            return
        if cmd == 'hardware':
            self.hardware_test()
            return
        if cmd == 'mod1':
            self.mod1_test()
            return
        try:
            parts = cmd.split()
            speed = 0.5  # Default speed
            if len(parts) == 2 and parts[0] in ['s', 'slow']:
                speed = 0.1
                arc = float(parts[1])
            else:
                arc = float(parts[0])
            steps = self.controller.arc_to_steps(arc)
            logger.info("Moving %d steps (%.2f arc) at speed %.2f", steps, arc, speed)
            self.controller.move_steps(steps, speed)
        except ValueError:
            logger.error("Invalid command: %s", cmd)

    def mod1_test(self):
        """Test using exact mod_1 parameters"""
        logger.info("Testing with exact mod_1 parameters")
        c = self.controller
        # Enable with LOW only
        GPIO.output(c.ENABLE_PIN, GPIO.LOW)
        sleep(0.1)
        # Use known working mod_1 timing only
        for i in range(20):
            GPIO.output(c.STEP_PIN, GPIO.HIGH)
            sleep(0.001)  # 1ms HIGH
            GPIO.output(c.STEP_PIN, GPIO.LOW)
            sleep(0.01)   # 10ms LOW
            logger.info(f"mod_1 style step {i+1}/20")
        # Disable with HIGH only
        GPIO.output(c.ENABLE_PIN, GPIO.HIGH)
        logger.info("mod_1 test complete")

    def run_test_sequence(self):
        logger.info("Running motor test sequence")
        c = self.controller
        logger.info("Testing enable pin HIGH/LOW")
        GPIO.output(c.ENABLE_PIN, GPIO.HIGH)
        sleep(0.5)
        GPIO.output(c.ENABLE_PIN, GPIO.LOW)
        sleep(0.5)
        for enable_state, state_name in [(GPIO.HIGH, "HIGH"), (GPIO.LOW, "LOW")]:
            logger.info(f"Testing steps with enable {state_name}")
            GPIO.output(c.ENABLE_PIN, enable_state)
            GPIO.output(c.DIR_PIN, GPIO.HIGH)
            for i in range(10):
                GPIO.output(c.STEP_PIN, GPIO.HIGH)
                sleep(0.001)  # 1ms HIGH
                GPIO.output(c.STEP_PIN, GPIO.LOW)
                sleep(0.01)   # 10ms LOW
                logger.info(f"Test step {i+1} with enable {state_name}")
        logger.info("Test complete")

    def hardware_test(self):
        """Run extensive hardware test with different timings"""
        logger.info("RUNNING COMPREHENSIVE HARDWARE TEST")
        c = self.controller
        for enable in [GPIO.HIGH, GPIO.LOW]:
            logger.info(f"\n*** TESTING WITH ENABLE={enable} ***")
            GPIO.output(c.ENABLE_PIN, enable)
            sleep(0.5)  # Longer pause between tests
            for direction in [GPIO.HIGH, GPIO.LOW]:
                logger.info(f"** Testing with DIR={direction} **")
                GPIO.output(c.DIR_PIN, direction)
                sleep(0.1)
                for high in [0.005, 0.01, 0.02, 0.05]:
                    for low in [0.005, 0.01, 0.02, 0.05, 0.1]:
                        logger.info(f"Testing: HIGH={high*1000:.1f}ms, LOW={low*1000:.1f}ms")
                        for _ in range(2):
                            GPIO.output(c.STEP_PIN, GPIO.HIGH)
                            sleep(high)
                            GPIO.output(c.STEP_PIN, GPIO.LOW)
                            sleep(low)
                        moved = input(f"Did the motor move with HIGH={high*1000:.1f}ms, LOW={low*1000:.1f}ms? (y/n): ").strip()
                        if moved.lower() == 'y':
                            logger.info(f"MOVEMENT DETECTED with HIGH={high*1000:.1f}ms, LOW={low*1000:.1f}ms")
        logger.info("Hardware test complete")

def main():
    controller = StepperController()
    stop_event = threading.Event()
    try:
        nunchuk_thread = NunchukInputThread(controller, stop_event)
        cli_thread = CLIInputThread(controller, stop_event)
        nunchuk_thread.start()
        cli_thread.start()
        cli_thread.join()
    except KeyboardInterrupt:
        logger.info("Keyboard interrupt received")
        stop_event.set()
    finally:
        controller.shutdown()
        logger.info("Program terminated")


if __name__ == '__main__':
    main()