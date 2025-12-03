from ultralytics import YOLO
import time

STOP_CLASS = 71   # stop sign
SLOW_CLASS = 66   # reduce speed / slow sign


class TrafficSignDetector:
    """
    Wrapper around YOLO model used for traffic sign detection.
    """

    def __init__(self, model_path="model_test.pt"):
        self.model = YOLO(model_path)
        self.current_class = None
        self.class_ts = 0.0

    def detect(self, frame, verbose=False):
        """
        Run YOLO on a frame.
        Returns:
          - results (Ultralytics object)
        """
        results = self.model(frame, verbose=verbose)
        return results

    def update_state(self, cls_id, ts, stop_duration=3.0, slow_duration=5.0):
        """
        Updates internal state based on detected class and timestamps.
        Can be used by main control loop for timing STOP/SLOW behavior.
        """
        if cls_id in (STOP_CLASS, SLOW_CLASS) and cls_id != self.current_class:
            self.current_class = cls_id
            self.class_ts = ts

        now = ts
        active = None

        if self.current_class == STOP_CLASS and (now - self.class_ts) < stop_duration:
            active = "STOP"
        elif self.current_class == SLOW_CLASS and (now - self.class_ts) < slow_duration:
            active = "SLOW"
        else:
            self.current_class = None

        return active
