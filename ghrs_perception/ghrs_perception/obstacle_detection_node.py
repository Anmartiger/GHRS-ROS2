#!/usr/bin/env python3
"""
GHRS Obstacle Detection Node
==============================
Camera-based obstacle detection using edge density analysis.
Replaces LiDAR (which caused camera thread starvation on RPi).

Divides the image into a 3-column ROI (left / centre / right) and
measures edge density in the lower portion of the frame.
Publishes a simple 3-float obstacle proximity score.

Subscribed topics:
  /camera/image_raw          (sensor_msgs/Image)

Published topics:
  /ghrs/obstacle_proximity   (std_msgs/Float32MultiArray)  – [left, centre, right] 0..1
  /ghrs/obstacle_detected    (std_msgs/Bool)
  /ghrs/obstacle_overlay     (sensor_msgs/Image)

Parameters:
  roi_top_frac     (float, default 0.55)  top of ROI as fraction of height
  edge_thresh      (float, default 0.08)  edge density threshold (0–1)
  canny_low        (int,   default 50)
  canny_high       (int,   default 150)
  blur_ksize       (int,   default 5)

Author : Anmar Arafat Al-Momani
Version: 1.0.0
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32MultiArray
from cv_bridge import CvBridge


class ObstacleDetectionNode(Node):
    """Camera edge-density obstacle detector."""

    def __init__(self):
        super().__init__('obstacle_detection_node')

        self.declare_parameter('roi_top_frac', 0.55)
        self.declare_parameter('edge_thresh',  0.08)
        self.declare_parameter('canny_low',    50)
        self.declare_parameter('canny_high',   150)
        self.declare_parameter('blur_ksize',   5)

        self._roi_top   = self.get_parameter('roi_top_frac').value
        self._thresh    = self.get_parameter('edge_thresh').value
        self._canny_lo  = self.get_parameter('canny_low').value
        self._canny_hi  = self.get_parameter('canny_high').value
        self._blur_k    = self.get_parameter('blur_ksize').value

        self._bridge = CvBridge()

        # Publishers
        self._prox_pub    = self.create_publisher(
            Float32MultiArray, '/ghrs/obstacle_proximity', 10)
        self._detect_pub  = self.create_publisher(
            Bool, '/ghrs/obstacle_detected', 10)
        self._overlay_pub = self.create_publisher(
            Image, '/ghrs/obstacle_overlay', 5)

        # Subscriber
        self.create_subscription(
            Image, '/camera/image_raw', self._image_cb, 5)

        self.get_logger().info(
            f'ObstacleDetectionNode ready – thresh={self._thresh}')

    # ────────────────────────────────────────────────────────────────────────
    def _image_cb(self, msg: Image):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge: {e}')
            return

        h, w = frame.shape[:2]
        roi_y = int(self._roi_top * h)
        roi   = frame[roi_y:, :]  # lower portion only

        # Greyscale + blur + Canny
        grey  = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur  = cv2.GaussianBlur(grey, (self._blur_k, self._blur_k), 0)
        edges = cv2.Canny(blur, self._canny_lo, self._canny_hi)

        # Split into 3 columns
        rh, rw = edges.shape
        col_w  = rw // 3
        cols   = [edges[:, :col_w],
                  edges[:, col_w:2*col_w],
                  edges[:, 2*col_w:]]

        densities = []
        for col in cols:
            total = col.size
            dense = float(np.count_nonzero(col)) / total if total > 0 else 0.0
            densities.append(dense)

        obstacle = any(d > self._thresh for d in densities)

        # Publish
        prox = Float32MultiArray()
        prox.data = [float(d) for d in densities]
        self._prox_pub.publish(prox)

        ob_msg = Bool()
        ob_msg.data = obstacle
        self._detect_pub.publish(ob_msg)

        # Overlay
        overlay = frame.copy()
        colours = [(255, 0, 0), (0, 165, 255), (0, 0, 255)]
        for i, (d, c) in enumerate(zip(densities, colours)):
            x1 = i * col_w
            x2 = (i + 1) * col_w if i < 2 else rw
            y1 = roi_y
            alpha = min(d / self._thresh, 1.0)
            colour = (int(c[0] * alpha), int(c[1] * alpha), int(c[2] * alpha))
            cv2.rectangle(overlay, (x1, y1), (x2, h), colour, 2)
            cv2.putText(overlay, f'{d:.2f}',
                        (x1 + 5, y1 + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 1)

        status = 'OBSTACLE!' if obstacle else 'Clear'
        colour = (0, 0, 255) if obstacle else (0, 200, 0)
        cv2.putText(overlay, status, (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, colour, 2)

        # Draw edge overlay in ROI
        edge_bgr = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
        overlay[roi_y:] = cv2.addWeighted(
            overlay[roi_y:], 0.7, edge_bgr, 0.3, 0)

        try:
            ov_msg = self._bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
            ov_msg.header = msg.header
            self._overlay_pub.publish(ov_msg)
        except Exception as e:
            self.get_logger().warn(f'Overlay error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
