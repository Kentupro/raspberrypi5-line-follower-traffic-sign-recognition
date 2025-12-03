import time
import gpiod

TIMEOUT_US = 10000  # microseconds


class QTRArray:
    """
    Reflectance sensor array (QTR) read using gpiod.
    Handles charge, measure, calibration and scaled read.
    """

    def __init__(self, sensor_pins, gpiochip='gpiochip0'):
        self.sensor_pins = sensor_pins
        self.gpiochip = gpiochip

        self.chip = gpiod.Chip(self.gpiochip)
        self.lines = self.chip.get_lines(self.sensor_pins)

        # calibration buffers
        self.smin = [TIMEOUT_US] * len(self.sensor_pins)
        self.smax = [0] * len(self.sensor_pins)

    def _charge(self):
        try:
            self.lines.release()
        except:
            pass

        self.lines.request(consumer="qtr_charge",
                           type=gpiod.LINE_REQ_DIR_OUT)
        self.lines.set_values([1] * len(self.sensor_pins))
        time.sleep(10e-6)  # 10 µs charge

    def _measure(self):
        try:
            self.lines.release()
        except:
            pass

        self.lines.request(
            consumer="qtr_measure",
            type=gpiod.LINE_REQ_DIR_IN,
            flags=gpiod.LINE_REQ_FLAG_BIAS_DISABLE
        )

        tu = [TIMEOUT_US] * len(self.sensor_pins)
        start = time.monotonic_ns()
        limit = start + TIMEOUT_US * 1000

        rem = list(range(len(self.sensor_pins)))

        while time.monotonic_ns() < limit and rem:
            now = time.monotonic_ns()
            vals = self.lines.get_values()
            nxt = []
            for i in rem:
                if vals[i] == 0:
                    tu[i] = (now - start) // 1000
                else:
                    nxt.append(i)
            rem = nxt
            if rem:
                time.sleep(50e-6)

        return tu

    def calibrate(self, duration=10.0, sample_time=0.02):
        """
        Rolls calibration for 'duration' seconds.
        You should move the robot over the line during this time.
        """
        end = time.time() + duration
        while time.time() < end:
            self._charge()
            raw = self._measure()
            for i, v in enumerate(raw):
                self.smin[i] = min(self.smin[i], v)
                self.smax[i] = max(self.smax[i], v)
            time.sleep(sample_time)

    def read_scaled(self):
        """
        Returns a list of sensor values scaled in [0..1000].
        0 = dark, 1000 = bright (depending on surface).
        """
        self._charge()
        raw = self._measure()
        out = [0] * len(raw)

        for i, v in enumerate(raw):
            rng = self.smax[i] - self.smin[i]
            if rng > 0:
                out[i] = int(
                    max(0, min(1000, (v - self.smin[i]) * 1000 / rng))
                )

        return out

    def close(self):
        try:
            self.lines.release()
        except:
            pass
        self.chip.close()
