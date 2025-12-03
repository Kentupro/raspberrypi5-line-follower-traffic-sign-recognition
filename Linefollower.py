import time

from qtr_sensors import QTRArray
from pid_controller import PIDController
from motor_driver_l298n import MotorDriverL298N

# ───────── HARDWARE CONFIGURATION ──────────────────────────────────
SENSOR_PINS = [4, 17, 27, 22, 23, 24, 25, 10]
IN1, IN2 = 16, 5
IN3, IN4 = 6, 26
ENA, ENB = 12, 13
GPIOCHIP = 'gpiochip0'

# ───────── LINE FOLLOWER PARAMETERS ────────────────────────────────
BASE_SPEED = 10        # base motor speed (percentage)
MAX_SPEED = 20         # maximum motor speed (percentage)
MIN_TURN_SPEED = 0     # minimum allowed speed
MIN_START = 0          # can be used for "bump start" if needed
PWM_FREQ = 100         # PWM frequency in Hz
SAMPLE_TIME = 0.02     # control loop period (50 Hz)

# PID gains
KP = 0.28
KI = 0.0
KD = 0.0

# QTR weights (negative on the left, positive on the right)
WEIGHTS = [-35, -25, -15, -5, 5, 15, 25, 35]

# Threshold below which we consider the line "lost"
LINE_LOST_THRESHOLD = 100


class LineFollowerRobot:
    """
    Simple line-following robot:
    - reads the QTR sensor array
    - computes line position
    - uses a PID controller to adjust motor speeds
    """

    def __init__(self):
        # Initialize QTR sensor array
        self.qtr = QTRArray(SENSOR_PINS, gpiochip=GPIOCHIP)

        # Initialize L298N motor driver
        self.motors = MotorDriverL298N(
            in1=IN1, in2=IN2,
            in3=IN3, in4=IN4,
            ena=ENA, enb=ENB,
            pwm_freq=PWM_FREQ,
            gpiochip=GPIOCHIP
        )

        # PID controller
        self.pid = PIDController(kp=KP, ki=KI, kd=KD, sample_time=SAMPLE_TIME)

    def calibrate(self, duration=10.0):
        """
        Calibrate QTR sensors.
        During calibration you should move the robot over the line
        so each sensor sees both the line and the background.
        """
        print(f"Calibrating sensors for {duration} seconds...")
        self.qtr.calibrate(duration=duration, sample_time=SAMPLE_TIME)
        print("Calibration finished.\n")

    def compute_line_position(self, values):
        """
        Compute line position using weighted average of sensor readings.
        Returns:
            float position (0 = centered) or
            None if the line is considered lost.
        """
        total = sum(values)
        if total < LINE_LOST_THRESHOLD:
            # Not enough signal: line is lost
            return None

        position = sum(w * v for w, v in zip(WEIGHTS, values)) / total
        return position

    def loop(self):
        """
        Main line-following loop.
        Continuously reads sensors, computes PID correction and
        updates motor speeds.
        """
        try:
            while True:
                # Read normalized QTR values
                values = self.qtr.read_scaled()

                # Compute line position
                position = self.compute_line_position(values)

                if position is None:
                    # Line lost: slow down and go straight
                    left_speed = right_speed = BASE_SPEED * 0.5
                    self.pid.reset()
                else:
                    # Error is deviation from the center
                    error = position
                    correction = self.pid.compute(error)

                    # Convert correction into left/right motor speeds
                    left_speed = BASE_SPEED - correction
                    right_speed = BASE_SPEED + correction

                    # Clamp speeds to allowed range
                    left_speed = max(MIN_TURN_SPEED, min(MAX_SPEED, left_speed))
                    right_speed = max(MIN_TURN_SPEED, min(MAX_SPEED, right_speed))

                # Apply speeds to motors
                self.motors.forward(left_speed, right_speed)

                # Respect control loop timing
                time.sleep(SAMPLE_TIME)

        except KeyboardInterrupt:
            print("\nCTRL+C pressed, stopping robot...")
        finally:
            self.stop()

    def stop(self):
        """
        Stop the robot and release all GPIO resources.
        """
        print("Stopping motors and cleaning up GPIO...")
        self.motors.stop()
        self.motors.cleanup()
        self.qtr.close()


if __name__ == "__main__":
    robot = LineFollowerRobot()
    robot.calibrate(duration=10.0)
    robot.loop()
