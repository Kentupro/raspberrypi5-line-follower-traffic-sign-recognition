class PIDController:
    """
    Simple PID controller used for line following.
    Only P and D are used in this project (I = 0.0 by default).
    """
    def __init__(self, kp=0.28, ki=0.0, kd=0.0, sample_time=0.02):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.sample_time = sample_time

        self._prev_error = 0.0
        self._integral = 0.0

    def reset(self):
        """Reset integrator and derivative memory."""
        self._prev_error = 0.0
        self._integral = 0.0

    def compute(self, error):
        """
        Compute PID output for given error.
        Returns a correction value (positive = steer right, negative = steer left).
        """
        # Proportional
        p = self.kp * error

        # Integral
        self._integral += error * self.sample_time
        i = self.ki * self._integral

        # Derivative
        d = self.kd * (error - self._prev_error) / self.sample_time

        self._prev_error = error
        return p + i + d
