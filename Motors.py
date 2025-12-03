import time
import threading
import gpiod


class SoftwarePWM(threading.Thread):
    """
    Simple software PWM implemented with a background thread.
    """

    def __init__(self, line, freq):
        super().__init__(daemon=True)
        self.line = line
        self.freq = freq
        self.duty_cycle = 0.0
        self._lock = threading.Lock()
        self._run = True

    def run(self):
        while self._run:
            with self._lock:
                dc = self.duty_cycle
                period = 1.0 / self.freq
                hi = period * dc
                lo = period * (1 - dc)

            if dc >= 1.0:
                self.line.set_value(1)
                time.sleep(period)
            elif dc <= 0.0:
                self.line.set_value(0)
                time.sleep(period)
            else:
                self.line.set_value(1)
                time.sleep(hi)
                self.line.set_value(0)
                time.sleep(lo)

    def set_duty(self, d):
        with self._lock:
            self.duty_cycle = max(0.0, min(1.0, d))

    def stop(self):
        self._run = False


class MotorDriverL298N:
    """
    L298N driver controlled via gpiod + SoftwarePWM for ENA/ENB.
    """

    def __init__(self, in1, in2, in3, in4, ena, enb,
                 pwm_freq=100, gpiochip='gpiochip0'):

        self.gpiochip = gpiochip
        self.chip = gpiod.Chip(self.gpiochip)

        self.dir_lines = self.chip.get_lines([in1, in2, in3, in4])
        self.dir_lines.request(
            consumer="mot_dir",
            type=gpiod.LINE_REQ_DIR_OUT,
            default_vals=[0] * 4
        )

        self.ena_line = self.chip.get_line(ena)
        self.enb_line = self.chip.get_line(enb)

        for l in (self.ena_line, self.enb_line):
            l.request(
                consumer="mot_pwm",
                type=gpiod.LINE_REQ_DIR_OUT,
                default_vals=[0]
            )

        self.pwmL = SoftwarePWM(self.ena_line, pwm_freq)
        self.pwmR = SoftwarePWM(self.enb_line, pwm_freq)
        self.pwmL.start()
        self.pwmR.start()

    def forward(self, left_speed, right_speed):
        """
        Drive forward with `left_speed`, `right_speed` in [0..100].
        """
        self.dir_lines.set_values([1, 0, 0, 1])
        self.pwmL.set_duty(max(0.0, min(1.0, left_speed / 100.0)))
        self.pwmR.set_duty(max(0.0, min(1.0, right_speed / 100.0)))

    def stop(self):
        self.pwmL.set_duty(0.0)
        self.pwmR.set_duty(0.0)
        self.dir_lines.set_values([0, 0, 0, 0])

    def cleanup(self):
        self.pwmL.stop()
        self.pwmR.stop()
        self.pwmL.join()
        self.pwmR.join()

        self.ena_line.set_value(0)
        self.enb_line.set_value(0)

        for l in (self.ena_line, self.enb_line):
            l.release()

        self.dir_lines.set_values([0]*4)
        self.dir_lines.release()
        self.chip.close()
