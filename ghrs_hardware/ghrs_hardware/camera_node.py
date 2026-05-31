#!/usr/bin/env python3
"""
GHRS Camera Node
=================
Captures frames from the local USB webcam and/or a Dahua IP camera
(RTSP) and publishes them as sensor_msgs/Image.

A secondary MJPEG TCP server mirrors the stream on port 5001 for the
Flask web dashboard (kept separate so it never blocks ROS spin).

Published topics:
  /camera/image_raw          (sensor_msgs/Image)  – USB webcam
  /camera/ip/image_raw       (sensor_msgs/Image)  – IP camera (if configured)

Parameters:
  webcam_device     (str,   default /dev/video0)
  webcam_width      (int,   default 640)
  webcam_height     (int,   default 480)
  webcam_fps        (int,   default 30)
  ip_cam_rtsp       (str,   default '')      – empty = disabled
  ip_cam_user       (str,   default '')
  ip_cam_pass       (str,   default '')
  mjpeg_port        (int,   default 5001)
  frame_id          (str,   default camera_link)

Author : Anmar Arafat Al-Momani
Version: 1.0.0
"""

import threading
import socketserver
import time
import os
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


# ─── MJPEG TCP streaming server ───────────────────────────────────────────────
MJPEG_BOUNDARY = b'--jpegboundary\r\nContent-Type: image/jpeg\r\n\r\n'


class _MjpegHandler(socketserver.StreamRequestHandler):
    def handle(self):
        self.wfile.write(
            b'HTTP/1.0 200 OK\r\n'
            b'Content-Type: multipart/x-mixed-replace; boundary=jpegboundary\r\n\r\n'
        )
        while True:
            frame = getattr(self.server, '_latest_frame', None)
            if frame is None:
                time.sleep(0.05)
                continue
            ret, jpg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
            if not ret:
                continue
            data = jpg.tobytes()
            try:
                self.wfile.write(MJPEG_BOUNDARY)
                self.wfile.write(data)
                self.wfile.write(b'\r\n')
            except (BrokenPipeError, ConnectionResetError):
                break


class CameraNode(Node):
    """Multi-source camera publisher with MJPEG sidecar."""

    def __init__(self):
        super().__init__('camera_node')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('webcam_device',  '/dev/video0')
        self.declare_parameter('webcam_width',   640)
        self.declare_parameter('webcam_height',  480)
        self.declare_parameter('webcam_fps',     30)
        self.declare_parameter('ip_cam_rtsp',    '')
        self.declare_parameter('ip_cam_user',    '')
        self.declare_parameter('ip_cam_pass',    '')
        self.declare_parameter('mjpeg_port',     5001)
        self.declare_parameter('frame_id',       'camera_link')

        self._dev     = self.get_parameter('webcam_device').value
        self._w       = self.get_parameter('webcam_width').value
        self._h       = self.get_parameter('webcam_height').value
        self._fps     = self.get_parameter('webcam_fps').value
        self._rtsp    = self.get_parameter('ip_cam_rtsp').value
        self._user    = self.get_parameter('ip_cam_user').value
        self._pass    = self.get_parameter('ip_cam_pass').value
        self._mjpeg_port = self.get_parameter('mjpeg_port').value
        self._frame_id = self.get_parameter('frame_id').value

        self._bridge  = CvBridge()

        # ── Publishers ──────────────────────────────────────────────────────
        self._webcam_pub = self.create_publisher(Image, '/camera/image_raw', 5)
        self._ip_pub     = self.create_publisher(Image, '/camera/ip/image_raw', 5)

        # ── Shared frames ────────────────────────────────────────────────────
        self._latest_webcam_frame = None
        self._latest_ip_frame     = None
        self._frame_lock = threading.Lock()

        # ── MJPEG server ─────────────────────────────────────────────────────
        self._mjpeg_server = None
        self._start_mjpeg_server()

        # ── Capture threads ──────────────────────────────────────────────────
        self._running = True
        self._webcam_thread = threading.Thread(
            target=self._webcam_loop, daemon=True)
        self._webcam_thread.start()

        if self._rtsp:
            self._ip_thread = threading.Thread(
                target=self._ip_loop, daemon=True)
            self._ip_thread.start()

        self.get_logger().info('CameraNode ready.')

    # ────────────────────────────────────────────────────────────────────────
    def _start_mjpeg_server(self):
        try:
            socketserver.TCPServer.allow_reuse_address = True
            srv = socketserver.ThreadingTCPServer(
                ('0.0.0.0', self._mjpeg_port), _MjpegHandler)
            srv._latest_frame = None
            self._mjpeg_server = srv
            t = threading.Thread(target=srv.serve_forever, daemon=True)
            t.start()
            self.get_logger().info(
                f'MJPEG server listening on port {self._mjpeg_port}')
        except Exception as e:
            self.get_logger().error(f'MJPEG server failed: {e}')

    def _webcam_loop(self):
        cap = cv2.VideoCapture(self._dev, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self._w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._h)
        cap.set(cv2.CAP_PROP_FPS,          self._fps)

        if not cap.isOpened():
            self.get_logger().error(f'Cannot open webcam {self._dev}')
            return

        self.get_logger().info(f'Webcam {self._dev} opened.')
        while self._running:
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.1)
                continue
            with self._frame_lock:
                self._latest_webcam_frame = frame
            if self._mjpeg_server:
                self._mjpeg_server._latest_frame = frame
            try:
                msg = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                msg.header.stamp    = self.get_clock().now().to_msg()
                msg.header.frame_id = self._frame_id
                self._webcam_pub.publish(msg)
            except Exception as e:
                self.get_logger().warn(f'Image publish error: {e}')
        cap.release()

    def _build_rtsp_url(self) -> str:
        if not self._user:
            return self._rtsp
        safe_pass = self._pass.replace('@', '%40')
        # Insert credentials into rtsp://host/path
        if '://' in self._rtsp:
            scheme, rest = self._rtsp.split('://', 1)
            return f'{scheme}://{self._user}:{safe_pass}@{rest}'
        return self._rtsp

    def _ip_loop(self):
        os.environ['OPENCV_FFMPEG_CAPTURE_OPTIONS'] = (
            'rtsp_transport;tcp|fflags;nobuffer|'
            'flags;low_delay|analyzeduration;0|probesize;32'
        )
        url = self._build_rtsp_url()
        self.get_logger().info(f'Connecting IP camera: {self._rtsp}')
        cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
        if not cap.isOpened():
            self.get_logger().error('IP camera stream failed to open.')
            return

        while self._running:
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.5)
                cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
                continue
            with self._frame_lock:
                self._latest_ip_frame = frame
            try:
                msg = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                msg.header.stamp    = self.get_clock().now().to_msg()
                msg.header.frame_id = 'ip_camera_link'
                self._ip_pub.publish(msg)
            except Exception as e:
                self.get_logger().warn(f'IP image publish error: {e}')
        cap.release()

    def destroy_node(self):
        self._running = False
        if self._mjpeg_server:
            self._mjpeg_server.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
