import threading
from flask import Flask, Response
import time
import gpiod
from picamera2 import Picamera2
from ultralytics import YOLO
import cv2
import math

# ───────── CONFIG HARDWARE ─────────────────────────────────────────
SENSOR_PINS = [4, 17, 27, 22, 23, 24, 25, 10]
IN1, IN2 = 16, 5
IN3, IN4 = 6, 26
ENA, ENB = 12, 13
GPIOCHIP = 'gpiochip0'

# ───────── PARAMETRI LINE–FOLLOWER ────────────────────────────────
BASE_SPEED = 10
MAX_SPEED = 20
MIN_TURN_SPEED = 0
MIN_START = 0
PWM_FREQ = 100
SAMPLE_TIME = 0.02  # 50 Hz
Kp = 0.28
Kd = 0.0
WEIGHTS = [-35, -25, -15, -5, 5, 15, 25, 35]
TIMEOUT_US = 10000
LINE_LOST_TH = 100

# ───────── PARAMETRI SEMNE YOLO ───────────────────────────────────
STOP_CLASS = 71   # STOP sign class ID
SLOW_CLASS = 66   # Speed limit / slow sign class ID
STOP_DURATION = 3.0
SLOW_DURATION = 5.0

# ───────── THREAD PWM SOFT ─────────────────────────────────────────
class SoftwarePWM(threading.Thread):
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

# ───────── CLASA LINEFOLLOWER ───────────────────────────────────────
class LineFollower:
    def __init__(self):
        # Sensors
        self.chip = gpiod.Chip(GPIOCHIP)
        self.sens = self.chip.get_lines(SENSOR_PINS)

        # Motor direction pins
        self.dir = self.chip.get_lines([IN1, IN2, IN3, IN4])
        self.dir.request(consumer="dir", type=gpiod.LINE_REQ_DIR_OUT, default_vals=[0]*4)

        # Motor enables with software PWM
        self.ena = self.chip.get_line(ENA)
        self.enb = self.chip.get_line(ENB)
        for l in (self.ena, self.enb):
            l.request(consumer="pwm", type=gpiod.LINE_REQ_DIR_OUT, default_vals=[0])

        self.pwmL = SoftwarePWM(self.ena, PWM_FREQ)
        self.pwmR = SoftwarePWM(self.enb, PWM_FREQ)
        self.pwmL.start()
        self.pwmR.start()

        # Calibration arrays
        self.smin = [TIMEOUT_US] * len(SENSOR_PINS)
        self.smax = [0] * len(SENSOR_PINS)

    def _charge(self):
        try:
            self.sens.release()
        except:
            pass
        self.sens.request(consumer="chg", type=gpiod.LINE_REQ_DIR_OUT)
        self.sens.set_values([1] * len(SENSOR_PINS))
        time.sleep(10e-6)

    def _measure(self):
        try:
            self.sens.release()
        except:
            pass

        self.sens.request(
            consumer="meas",
            type=gpiod.LINE_REQ_DIR_IN,
            flags=gpiod.LINE_REQ_FLAG_BIAS_DISABLE
        )

        tu = [TIMEOUT_US] * len(SENSOR_PINS)
        start = time.monotonic_ns()
        limit = start + TIMEOUT_US * 1000
        rem = list(range(len(SENSOR_PINS)))

        while time.monotonic_ns() < limit and rem:
            now = time.monotonic_ns()
            vals = self.sens.get_values()
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

    def calibrate(self, dur=10.0):
        end = time.time() + dur
        while time.time() < end:
            self._charge()
            raw = self._measure()
            for i, v in enumerate(raw):
                self.smin[i] = min(self.smin[i], v)
                self.smax[i] = max(self.smax[i], v)
            time.sleep(SAMPLE_TIME)

    def read_scaled(self):
        self._charge()
        raw = self._measure()
        out = [0] * len(raw)
        for i, v in enumerate(raw):
            rng = self.smax[i] - self.smin[i]
            if rng > 0:
                out[i] = int(max(0, min(1000, (v - self.smin[i]) * 1000 / rng)))
        return out

    def set_motor(self, ls, rs):
        ls = max(MIN_TURN_SPEED, min(MAX_SPEED, ls))
        rs = max(MIN_TURN_SPEED, min(MAX_SPEED, rs))

        # Bump start
        if MIN_TURN_SPEED < ls < MIN_START:
            ls = MIN_START
        if MIN_TURN_SPEED < rs < MIN_START:
            rs = MIN_START

        # Forward
        self.dir.set_values([1, 0, 0, 1])
        self.pwmL.set_duty(ls / 100.0)
        self.pwmR.set_duty(rs / 100.0)

    def cleanup(self):
        self.pwmL.stop(); self.pwmR.stop()
        self.pwmL.join(); self.pwmR.join()

        self.ena.set_value(0)
        self.enb.set_value(0)

        for l in (self.ena, self.enb):
            l.release()

        self.dir.set_values([0]*4)
        self.dir.release()
        self.sens.release()
        self.chip.close()

# ───────── SETUP GLOBAL ─────────────────────────────────────────────
lf = LineFollower()
print("Calibrare senzori → 10s …")
lf.calibrate()
print("Calibrare gata ✔")

picam2 = Picamera2()
cam_cfg = picam2.create_preview_configuration(
    main={"format": "RGB888", "size": (640, 480)}
)
picam2.configure(cam_cfg)
picam2.start()

model = YOLO("model_test.pt")

current_class = None
class_ts = 0.0

app = Flask(__name__)

def gen_frames():
    global current_class, class_ts
    last_pos = 0.0
    stop1 = 0
    stop2 = 0

    while True:
        tic = time.monotonic()

        # Capture
        frame = picam2.capture_array()

        # YOLO detection
        results = model(frame, verbose=False)
        for r in results:
            for box in r.boxes:
                cls = int(box.cls[0])

                if cls in (STOP_CLASS, SLOW_CLASS) and cls != current_class:
                    current_class = cls
                    class_ts = tic

                x1, y1, x2, y2 = box.xyxy[0].int().tolist()

                if cls == STOP_CLASS:
                    label = "STOP"
                elif cls == SLOW_CLASS:
                    label = "REDUCE SPEED"
                else:
                    label = f"cls{cls}"

                cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,0), 2)
                cv2.putText(frame, label, (x1, y1-8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,0), 2)

        # Line read
        vals = lf.read_scaled()
        tot = sum(vals)
        now = tic

        # Behavior based on sign
        if current_class == STOP_CLASS and (now - class_ts) < STOP_DURATION:
            base = 0
            maxv = 0
            stop1 = 1

        elif current_class == SLOW_CLASS and (now - class_ts) < SLOW_DURATION:
            base = 7
            maxv = 14
            stop1 = 0
            stop2 = 0

        else:
            current_class = None
            base = BASE_SPEED
            maxv = MAX_SPEED

        if stop1 == 1:
            stop2 = 1

        # PID
        if tot < LINE_LOST_TH:
            ls = rs = base * 0.5
            last_pos = 0.0
        else:
            pos = sum(w*v for w,v in zip(WEIGHTS, vals)) / tot
            corr = Kp*pos + Kd*(pos - last_pos)/SAMPLE_TIME
            ls = base - corr
            rs = base + corr
            last_pos = pos

        ls = max(MIN_TURN_SPEED, min(maxv, ls))
        rs = max(MIN_TURN_SPEED, min(maxv, rs))

        lf.set_motor(ls, rs)

        # MJPEG encode
        ret, buf = cv2.imencode('.jpg', frame)
        if not ret:
            continue

        yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' +
               buf.tobytes() + b'\r\n')

        dt = time.monotonic() - tic
        if dt < SAMPLE_TIME:
            time.sleep(SAMPLE_TIME - dt)

@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def index():
    return """
    <!doctype html>
    <html>
    <head><title>Live YOLO + PID</title></head>
    <body style="margin:0; background:#000;">
    <img src="/video_feed" style="width:100vw; height:auto;">
    </body>
    </html>
    """

if __name__ == '__main__':
    try:
        app.run(host='0.0.0.0', port=5000, threaded=True)
    except KeyboardInterrupt:
        print("\nCTRL-C – opresc…")
    finally:
        lf.cleanup()
