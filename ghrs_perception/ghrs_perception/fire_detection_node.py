#!/usr/bin/env python3
"""
GHRS Fire Detection Node
==========================
Uses simple HSV pixel-count thresholding (proven robust on real hardware)
to detect fire in camera frames and publish actionable detections.

Subscribed topics:
  /camera/image_raw      (sensor_msgs/Image)

Published topics:
  /ghrs/fire_detection   (ghrs_msgs/FireDetection)
  /ghrs/fire_overlay     (sensor_msgs/Image)  – annotated frame

Parameters:
  h_low, h_high         (int, default 0/35)      HSV hue range
  s_low, s_high         (int, default 100/255)   HSV saturation range
  v_low, v_high         (int, default 100/255)   HSV value range
  pixel_ratio_thresh    (float, default 0.005)   min fire fraction to trigger
  confirm_frames        (int,   default 3)       consecutive hits to confirm
  frame_id              (str,   default camera_link)

Author : Anmar Arafat Al-Momani
Version: 1.0.0
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

try:
    from ghrs_msgs.msg import FireDetection
    MSGS_AVAILABLE = True
except ImportError:
    MSGS_AVAILABLE = False


class FireDetectionNode(Node):
    """HSV-based fire detection with centroid tracking."""

    def __init__(self):
        super().__init__('fire_detection_node')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('h_low',              0)
        self.declare_parameter('h_high',             35)
        self.declare_parameter('s_low',              100)
        self.declare_parameter('s_high',             255)
        self.declare_parameter('v_low',              100)
        self.declare_parameter('v_high',             255)
        self.declare_parameter('pixel_ratio_thresh', 0.005)
        self.declare_parameter('confirm_frames',     3)
        self.declare_parameter('frame_id',           'camera_link')

        self._hl   = self.get_parameter('h_low').value
        self._hh   = self.get_parameter('h_high').value
        self._sl   = self.get_parameter('s_low').value
        self._sh   = self.get_parameter('s_high').value
        self._vl   = self.get_parameter('v_low').value
        self._vh   = self.get_parameter('v_high').value
        self._thresh = self.get_parameter('pixel_ratio_thresh').value
        self._confirm = self.get_parameter('confirm_frames').value
        self._frame_id = self.get_parameter('frame_id').value

        self._bridge  = CvBridge()
        self._hit_count = 0

        # Publishers
        self._overlay_pub = self.create_publisher(
            Image, '/ghrs/fire_overlay', 5)

        if MSGS_AVAILABLE:
            self._fire_pub = self.create_publisher(
                FireDetection, '/ghrs/fire_detection', 10)
        else:
            self._fire_pub = None
            self.get_logger().warn('ghrs_msgs not built – fire detection msg disabled.')

        # Subscriber
        self.create_subscription(
            Image, '/camera/image_raw', self._image_cb, 5)

        self.get_logger().info(
            f'FireDetectionNode ready – HSV H[{self._hl},{self._hh}] '
            f'S[{self._sl},{self._sh}] V[{self._vl},{self._vh}] '
            f'thresh={self._thresh}')

    # ────────────────────────────────────────────────────────────────────────
    def _image_cb(self, msg: Image):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge error: {e}')
            return

        h, w = frame.shape[:2]
        total_pixels = h * w

        # ── HSV mask ─────────────────────────────────────────────────────
        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(
            hsv,
            np.array([self._hl, self._sl, self._vl]),
            np.array([self._hh, self._sh, self._vh]),
        )

        fire_pixels = int(cv2.countNonZero(mask))
        ratio       = fire_pixels / total_pixels if total_pixels > 0 else 0.0
        detected    = ratio >= self._thresh

        if detected:
            self._hit_count += 1
        else:
            self._hit_count = max(0, self._hit_count - 1)

        confirmed = self._hit_count >= self._confirm

        # ── Centroid & bounding box ───────────────────────────────────────
        cx = cy = 0.0
        bx = by = bw_n = bh_n = 0.0

        if confirmed:
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                largest = max(contours, key=cv2.contourArea)
                M = cv2.moments(largest)
                if M['m00'] > 0:
                    cx = M['m10'] / M['m00']
                    cy = M['m01'] / M['m00']
                x, y, bw_, bh_ = cv2.boundingRect(largest)
                bx   = x / w
                by   = y / h
                bw_n = bw_ / w
                bh_n = bh_ / h

        # ── Publish FireDetection ─────────────────────────────────────────
        if self._fire_pub:
            fd = FireDetection()
            fd.header.stamp    = msg.header.stamp
            fd.header.frame_id = self._frame_id
            fd.fire_detected   = confirmed
            fd.confidence      = float(min(ratio / max(self._thresh, 1e-6), 1.0))
            fd.pixel_ratio     = float(ratio)
            fd.centroid_x      = int(cx)
            fd.centroid_y      = int(cy)
            fd.bbox_x          = float(bx)
            fd.bbox_y          = float(by)
            fd.bbox_w          = float(bw_n)
            fd.bbox_h          = float(bh_n)
            fd.source          = 'webcam'
            self._fire_pub.publish(fd)

        # ── Overlay ───────────────────────────────────────────────────────
        overlay = frame.copy()
        if confirmed:
            # Colour fire region
            fire_region = cv2.bitwise_and(frame, frame, mask=mask)
            overlay     = cv2.addWeighted(overlay, 0.6, fire_region, 0.4, 0)

            # Draw bounding box
            x1 = int(bx * w)
            y1 = int(by * h)
            x2 = int((bx + bw_n) * w)
            y2 = int((by + bh_n) * h)
            cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 0, 255), 2)
            cv2.circle(overlay, (int(cx), int(cy)), 8, (0, 255, 255), -1)
            cv2.putText(overlay, f'FIRE {ratio*100:.1f}%',
                        (x1, max(y1 - 10, 15)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        else:
            status = f'No fire ({ratio*100:.2f}%)'
            cv2.putText(overlay, status, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 0), 2)

        try:
            ov_msg = self._bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
            ov_msg.header = msg.header
            self._overlay_pub.publish(ov_msg)
        except Exception as e:
            self.get_logger().warn(f'Overlay publish error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = FireDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
