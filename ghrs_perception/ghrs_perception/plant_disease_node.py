#!/usr/bin/env python3
"""
GHRS Plant Disease Detection Node
=====================================
Classifies plant health from camera frames using a TensorFlow Lite or
PyTorch model (PlantVillage dataset labels).  Falls back to a simple
colour-deviation heuristic when no model is available.

Subscribed topics:
  /camera/image_raw          (sensor_msgs/Image)
  /gps/fix                   (sensor_msgs/NavSatFix)  – tagged to detections

Published topics:
  /ghrs/plant_health         (ghrs_msgs/PlantHealth)
  /ghrs/plant_overlay        (sensor_msgs/Image)

Services:
  /ghrs/capture_sample       (std_srvs/Trigger)  – save timestamped sample image

Parameters:
  model_path      (str,   default '')   path to .tflite or .pt model
  input_width     (int,   default 224)
  input_height    (int,   default 224)
  confidence_min  (float, default 0.5)
  scan_rate_hz    (float, default 1.0)  how often to run inference
  save_dir        (str,   default /tmp/ghrs_samples)

Supported labels (PlantVillage subset):
  Healthy | Tomato Late Blight | Tomato Early Blight | Tomato Leaf Miner
  Powdery Mildew | Bacterial Spot | Septoria Leaf Spot | Unknown

Author : Anmar Arafat Al-Momani
Version: 1.0.0
"""

import os
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, NavSatFix
from std_srvs.srv import Trigger
from cv_bridge import CvBridge

try:
    from ghrs_msgs.msg import PlantHealth
    MSGS_AVAILABLE = True
except ImportError:
    MSGS_AVAILABLE = False

# Optional ML backends
try:
    import tflite_runtime.interpreter as tflite
    TFLITE_AVAILABLE = True
except ImportError:
    TFLITE_AVAILABLE = False

try:
    import torch
    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False


LABELS = [
    'Healthy',
    'Tomato Late Blight',
    'Tomato Early Blight',
    'Tomato Leaf Miner',
    'Powdery Mildew',
    'Bacterial Spot',
    'Septoria Leaf Spot',
    'Unknown',
]

SEVERITY_MAP = {
    'Healthy':            0.0,
    'Tomato Late Blight': 0.9,
    'Tomato Early Blight':0.7,
    'Tomato Leaf Miner':  0.5,
    'Powdery Mildew':     0.6,
    'Bacterial Spot':     0.7,
    'Septoria Leaf Spot': 0.6,
    'Unknown':            0.3,
}

RECOMMENDATIONS = {
    'Healthy':             'No action required',
    'Tomato Late Blight':  'Apply copper fungicide; remove infected foliage; improve air circulation',
    'Tomato Early Blight': 'Apply chlorothalonil fungicide; mulch base; avoid overhead watering',
    'Tomato Leaf Miner':   'Use insecticide (spinosad); remove infested leaves; introduce predators',
    'Powdery Mildew':      'Apply potassium bicarbonate or neem oil; improve ventilation',
    'Bacterial Spot':      'Apply copper bactericide; avoid wetting leaves; remove debris',
    'Septoria Leaf Spot':  'Apply fungicide at first sign; remove lower leaves; rotate crops',
    'Unknown':             'Manual inspection recommended; collect sample for laboratory analysis',
}


class PlantDiseaseNode(Node):
    """Plant disease classification node."""

    def __init__(self):
        super().__init__('plant_disease_node')

        self.declare_parameter('model_path',     '')
        self.declare_parameter('input_width',    224)
        self.declare_parameter('input_height',   224)
        self.declare_parameter('confidence_min', 0.5)
        self.declare_parameter('scan_rate_hz',   1.0)
        self.declare_parameter('save_dir',       '/tmp/ghrs_samples')

        self._model_path   = self.get_parameter('model_path').value
        self._in_w         = self.get_parameter('input_width').value
        self._in_h         = self.get_parameter('input_height').value
        self._conf_min     = self.get_parameter('confidence_min').value
        self._scan_rate    = self.get_parameter('scan_rate_hz').value
        self._save_dir     = self.get_parameter('save_dir').value

        os.makedirs(self._save_dir, exist_ok=True)

        self._bridge    = CvBridge()
        self._model     = None
        self._last_scan = 0.0
        self._latest_frame   = None
        self._latest_gps_fix = None

        self._load_model()

        # Publishers
        self._overlay_pub = self.create_publisher(
            Image, '/ghrs/plant_overlay', 5)

        if MSGS_AVAILABLE:
            self._health_pub = self.create_publisher(
                PlantHealth, '/ghrs/plant_health', 10)
        else:
            self._health_pub = None

        # Subscribers
        self.create_subscription(
            Image, '/camera/image_raw', self._image_cb, 5)
        self.create_subscription(
            NavSatFix, '/gps/fix', self._gps_cb, 10)

        # Services
        self.create_service(Trigger, '/ghrs/capture_sample', self._capture_cb)

        self.get_logger().info(
            f'PlantDiseaseNode ready. Model: '
            f'{"TFLite" if self._model else "heuristic fallback"}')

    # ────────────────────────────────────────────────────────────────────────
    def _load_model(self):
        if not self._model_path or not os.path.exists(self._model_path):
            self.get_logger().warn(
                f'Model not found at "{self._model_path}" – using colour heuristic.')
            return
        if TFLITE_AVAILABLE and self._model_path.endswith('.tflite'):
            try:
                interp = tflite.Interpreter(model_path=self._model_path)
                interp.allocate_tensors()
                self._model = ('tflite', interp)
                self.get_logger().info('TFLite model loaded.')
            except Exception as e:
                self.get_logger().error(f'TFLite load failed: {e}')
        elif TORCH_AVAILABLE and self._model_path.endswith('.pt'):
            try:
                model = torch.load(self._model_path, map_location='cpu')
                model.eval()
                self._model = ('torch', model)
                self.get_logger().info('PyTorch model loaded.')
            except Exception as e:
                self.get_logger().error(f'PyTorch load failed: {e}')

    # ────────────────────────────────────────────────────────────────────────
    def _image_cb(self, msg: Image):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            return
        self._latest_frame = frame

        now = time.monotonic()
        if (now - self._last_scan) < (1.0 / max(self._scan_rate, 0.1)):
            return
        self._last_scan = now

        label, confidence, scores = self._classify(frame)
        severity = SEVERITY_MAP.get(label, 0.3)
        recs     = RECOMMENDATIONS.get(label, 'Inspect manually')

        if self._health_pub:
            ph = PlantHealth()
            ph.header.stamp  = msg.header.stamp
            ph.disease_label = label
            ph.confidence    = float(confidence)
            ph.severity      = float(severity)
            ph.recommendations = recs
            if self._latest_gps_fix:
                ph.location = self._latest_gps_fix
            # Save image if diseased
            if label != 'Healthy' and confidence >= self._conf_min:
                fname = os.path.join(
                    self._save_dir,
                    f'{label.replace(" ", "_")}_{int(time.time())}.jpg')
                cv2.imwrite(fname, frame)
                ph.image_path = fname

            if scores and len(scores) >= 2:
                sorted_idx = sorted(range(len(scores)),
                                    key=lambda i: scores[i], reverse=True)
                ph.secondary_labels = [LABELS[i] for i in sorted_idx[1:3]
                                       if i < len(LABELS)]
                ph.secondary_conf   = [float(scores[i]) for i in sorted_idx[1:3]
                                       if i < len(scores)]
            self._health_pub.publish(ph)

        self._publish_overlay(frame, label, confidence, severity)

    def _gps_cb(self, msg: NavSatFix):
        self._latest_gps_fix = msg

    # ────────────────────────────────────────────────────────────────────────
    def _classify(self, frame):
        """Return (label, confidence, all_scores)."""
        if self._model is None:
            return self._heuristic(frame)

        inp = cv2.resize(frame, (self._in_w, self._in_h))
        inp = cv2.cvtColor(inp, cv2.COLOR_BGR2RGB)
        inp = inp.astype(np.float32) / 255.0
        inp = np.expand_dims(inp, axis=0)

        kind, model = self._model
        try:
            if kind == 'tflite':
                in_det  = model.get_input_details()
                out_det = model.get_output_details()
                model.set_tensor(in_det[0]['index'], inp)
                model.invoke()
                scores = model.get_tensor(out_det[0]['index'])[0].tolist()
            elif kind == 'torch':
                import torch
                with torch.no_grad():
                    t = torch.from_numpy(inp).permute(0, 3, 1, 2)
                    out = model(t)
                    scores = torch.softmax(out, dim=1)[0].tolist()
            else:
                return self._heuristic(frame)

            idx = int(np.argmax(scores))
            label = LABELS[idx] if idx < len(LABELS) else 'Unknown'
            return label, float(scores[idx]), scores

        except Exception as e:
            self.get_logger().warn(f'Inference error: {e}')
            return self._heuristic(frame)

    def _heuristic(self, frame):
        """Simple colour-deviation heuristic – fallback only."""
        hsv   = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # Yellow/brown regions (disease indicator)
        mask1 = cv2.inRange(hsv, np.array([15,  50,  50]),
                                 np.array([35, 255, 255]))
        # Dark spots
        mask2 = cv2.inRange(hsv, np.array([0,  0,  0]),
                                 np.array([180, 255, 60]))
        total = frame.shape[0] * frame.shape[1]
        ratio_yellow = cv2.countNonZero(mask1) / total
        ratio_dark   = cv2.countNonZero(mask2) / total

        if ratio_yellow > 0.25:
            return 'Tomato Late Blight', min(ratio_yellow * 3.0, 0.95), None
        elif ratio_dark > 0.15:
            return 'Bacterial Spot', min(ratio_dark * 4.0, 0.90), None
        else:
            conf = max(0.6, 1.0 - ratio_yellow * 5.0 - ratio_dark * 3.0)
            return 'Healthy', float(conf), None

    # ────────────────────────────────────────────────────────────────────────
    def _publish_overlay(self, frame, label, confidence, severity):
        overlay = frame.copy()
        colour  = (0, 200, 0) if label == 'Healthy' else (0, 0, 255)
        if 0.3 < severity < 0.7:
            colour = (0, 165, 255)

        cv2.putText(overlay, f'{label}  {confidence*100:.0f}%',
                    (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.75, colour, 2)
        cv2.putText(overlay, f'Severity: {severity:.1f}',
                    (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.55, colour, 1)

        bar_w = int(overlay.shape[1] * severity)
        cv2.rectangle(overlay, (0, overlay.shape[0] - 10),
                      (bar_w, overlay.shape[0]), colour, -1)
        try:
            ov_msg = self._bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
            self._overlay_pub.publish(ov_msg)
        except Exception:
            pass

    def _capture_cb(self, _request, response):
        if self._latest_frame is None:
            response.success = False
            response.message = 'No frame available'
            return response
        fname = os.path.join(self._save_dir,
                             f'sample_{int(time.time())}.jpg')
        cv2.imwrite(fname, self._latest_frame)
        response.success = True
        response.message = f'Saved {fname}'
        self.get_logger().info(f'Sample saved: {fname}')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PlantDiseaseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
