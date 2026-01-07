#!/usr/bin/env python3
"""
Z-axis motor controller with manual distance control.
Combined Motor, Controller, and API functions in one file.
"""
import threading
import time

try:
    import RPi.GPIO as GPIO
except Exception:
    # Stub for dev 
    class _FakePWM:
        def __init__(self, pin, freq):
            self.pin = pin
            self.freq = freq
        def start(self, duty): pass
        def ChangeDutyCycle(self, duty): pass
        def stop(self): pass

    class _FakeGPIO:
        BCM = 'BCM'
        OUT = 'OUT'
        HIGH = True
        LOW = False
        def setmode(self, mode): pass
        def setwarnings(self, flag): pass
        def setup(self, pin, mode): pass
        def output(self, pin, val): pass
        def PWM(self, pin, freq): return _FakePWM(pin, freq)
        def cleanup(self): pass

    GPIO = _FakeGPIO()

# Z-axis motor pins (BCM)
Z_PWM_PIN = 13
Z_DIR_PIN = 19
PWM_FREQ = 1000  # Hz
Z_SPEED = 100  # percent duty cycle

# Ramping parameters
RAMP_TIME = 0.5
RAMP_STEPS = 50
STEP_DELAY = RAMP_TIME / RAMP_STEPS

# Z-axis speed calibration
Z_SPEED_AT_100_PERCENT = 10  # cm/sec at 100% duty cycle

# Global motor and controller instances
_z_motor = None
_z_controller = None


class Motor:
    """Motor control class for Z-axis"""
    def __init__(self, pwm_pin, dir_pin):
        self.pwm_pin = pwm_pin
        self.dir_pin = dir_pin
        self.current_speed = 0.0
        self.target_speed = 0.0
        self.ramp_thread = None
        self.stop_requested = False

        GPIO.setup(self.pwm_pin, GPIO.OUT)
        GPIO.setup(self.dir_pin, GPIO.OUT)
        GPIO.output(self.dir_pin, GPIO.LOW)

        self.pwm = GPIO.PWM(self.pwm_pin, PWM_FREQ)
        self.pwm.start(0)

    def set_direction(self, direction):
        GPIO.output(self.dir_pin, GPIO.HIGH if direction == 1 else GPIO.LOW)

    def ramp_to_speed(self, target_speed, direction):
        self.stop_requested = True
        if self.ramp_thread and self.ramp_thread.is_alive():
            self.ramp_thread.join(timeout=0.1)
        self.stop_requested = False

        self.target_speed = max(0.0, min(100.0, float(abs(target_speed))))
        self.set_direction(direction)

        def ramp():
            start = self.current_speed
            diff = self.target_speed - start
            for step in range(RAMP_STEPS + 1):
                if self.stop_requested:
                    break
                frac = step / RAMP_STEPS
                new_speed = start + diff * frac
                self.current_speed = new_speed
                try:
                    self.pwm.ChangeDutyCycle(new_speed)
                except Exception:
                    pass
                time.sleep(STEP_DELAY)
            if not self.stop_requested:
                self.current_speed = self.target_speed

        self.ramp_thread = threading.Thread(target=ramp, daemon=True)
        self.ramp_thread.start()

    def stop_smooth(self):
        self.ramp_to_speed(0, 1)

    def stop_immediate(self):
        self.stop_requested = True
        self.current_speed = 0.0
        self.target_speed = 0.0
        try:
            self.pwm.ChangeDutyCycle(0)
        except Exception:
            pass

    def cleanup(self):
        self.stop_immediate()
        try:
            self.pwm.stop()
        except Exception:
            pass


class ZAxisController:
    """Z-axis motor movement controller"""
    def __init__(self, motor):
        self.motor = motor
        self.moving = False
        self.move_thread = None
        self.current_position = 0.0

    def move_distance(self, distance_cm, direction, speed_percent=100):
        if self.moving:
            print("Already moving, please wait...")
            return

        distance_cm = abs(float(distance_cm))
        if distance_cm <= 0:
            print("Distance must be > 0 cm")
            return

        speed_cm_per_sec = (speed_percent / 100.0) * Z_SPEED_AT_100_PERCENT
        move_time = distance_cm / speed_cm_per_sec

        print(f"Moving {distance_cm:.2f} cm at {speed_percent}% "
              f"({'UP' if direction == 1 else 'DOWN'}) "
              f"for {move_time:.2f} seconds...")

        self.moving = True
        self.move_thread = threading.Thread(
            target=self._execute_move,
            args=(move_time, direction, speed_percent, distance_cm),
            daemon=True
        )
        self.move_thread.start()

    def _execute_move(self, duration, direction, speed_percent, distance_cm):
        try:
            self.motor.ramp_to_speed(speed_percent, direction)
            time.sleep(duration)
            self.motor.stop_immediate()
            self.current_position += distance_cm * direction
            print(f"Move complete. Current position: {self.current_position:.2f} cm")
        except Exception as ex:
            print(f"Move error: {ex}")
        finally:
            self.moving = False


# API Functions
def init_z_axis():
    """Initialize Z-axis motor and controller. Call this once at startup."""
    global _z_motor, _z_controller
    
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    _z_motor = Motor(Z_PWM_PIN, Z_DIR_PIN)
    _z_controller = ZAxisController(_z_motor)
    print("Z-axis initialized")


def z_axis(distance, direction):
    """
    Move Z-axis motor.
    
    Args:
        distance: Distance to move in cm (positive value)
        direction: 1 for UP, 0 for DOWN
    
    Example:
        z_axis(20, 1)   # Move 20 cm UP
        z_axis(10, 0)   # Move 10 cm DOWN
    """
    global _z_motor, _z_controller
    
    if _z_motor is None or _z_controller is None:
        print("Error: Z-axis not initialized. Call init_z_axis() first.")
        return
    
    # Convert direction: 0 -> -1 (down), 1 -> 1 (up)
    dir_value = 1 if direction == 1 else -1
    _z_controller.move_distance(distance, direction=dir_value, speed_percent=100)


def cleanup_z_axis():
    """Cleanup Z-axis resources. Call this at shutdown."""
    global _z_motor
    if _z_motor:
        _z_motor.cleanup()
        GPIO.cleanup()
        print("Z-axis cleanup complete")


def main():
    """Interactive menu for manual control"""
    init_z_axis()

    print("=" * 50)
    print("Z-Axis Motor Controller")
    print("=" * 50)

    try:
        while True:
            print("\nOptions:")
            print("  1 - Move UP")
            print("  2 - Move DOWN")
            print("  3 - Emergency Stop")
            print("  4 - Quit")
            
            choice = input("\nEnter choice (1-4): ").strip()

            if choice == "1":
                try:
                    distance = float(input("Enter distance (cm): "))
                    z_axis(distance, 1)
                except ValueError:
                    print("Invalid distance value")
            
            elif choice == "2":
                try:
                    distance = float(input("Enter distance (cm): "))
                    z_axis(distance, 0)
                except ValueError:
                    print("Invalid distance value")
            
            elif choice == "3":
                if _z_motor:
                    _z_motor.stop_immediate()
                print("Emergency stop activated")
            
            elif choice == "4":
                print("Quitting...")
                break
            
            else:
                print("Invalid choice")

    except KeyboardInterrupt:
        print("\nInterrupted by user")
    
    finally:
        cleanup_z_axis()


if __name__ == "__main__":
    try:
        main()
    except Exception as ex:
        print(f"Error: {ex}")
        try:
            cleanup_z_axis()
        except Exception:
            pass
