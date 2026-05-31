#!/usr/bin/env python3
"""
GHRS Turret Tracking Node
===========================
Receives fire detections (or generic target centroids) and drives the
turret pan/tilt servos to centre the target using a PID loop.
When a target is centred, it commands the fire-suppression sequence
(trigger + pump burst).

Subscribed topics:
  /ghrs/fire_detection   (ghrs_msgs/FireDetection)
  /ghrs/track_target     (geometry_msgs/Point)     – normalised -1..1 x,y

Published topics:
  /ghrs/turret_pan       (std_msgs/Float32)   – degrees
  /ghrs/turret_tilt      (std_msgs/Float32)   – degrees
  /ghrs/trigger          (std_msgs/Bool)
  /ghrs/pump_cmd         (std_msgs/Bool)
  /ghrs/tracking_status  (std_msgs/String)

Services:
  /ghrs/tracking_enable  (std_srvs/SetBool)

Parameters:
  kp_pan, ki_pan, kd_pan   (float, default 0.08 / 0.0 / 0.02)
  kp_tilt, ki_tilt, kd_tilt
  pan_deadband             (float, default 0.05)   normalised pixel
  tilt_deadband            (float, default 0.05)
  fire_deadband            (float, default 0.07)   when to fire
  suppress_duration        (float, default 2.0)    seconds
  max_pan_deg              (float, default 80.0)
  max_tilt_deg             (float, default 45.0)

Author : Anmar Arafat Al-Momani
Version: 1.0.0
"""

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, String
from geometry_msgs.msg import Point
from std_srvs.srv import SetBool

try:
    from ghrs_msgs.msg import FireDetection
    MSGS_AVAILABLE = True
except ImportError:
    MSGS_AVAILABLE = False


class PID:
    def __init__(self, kp, ki, kd, out_min=-90.0, out_max=90.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_min, self.out_max = out_min, out_max
        self._integral = 0.0
        self._last_err = 0.0
        self._last_t   = None

    def update(self, error: float) -> float:
        now = time.monotonic()
        dt  = (now - self._last_t) if self._last_t else 0.02
        self._last_t = now

        self._integral += error * dt
        self._integral  = max(-50.0, min(50.0, self._integral))
        deriv = (error - self._last_err) / dt if dt > 0 else 0.0
        self._last_err = error

        out = self.kp * error + self.ki * self._integral + self.kd * deriv
        return max(self.out_min, min(self.out_max, out))

    def reset(self):
        self._integral = 0.0
        self._last_err = 0.0
        self._last_t   = None


class TurretTrackingNode(Node):
    """Pan-tilt turret tracker with fire-suppression trigger."""

    def __init__(self):
        super().__init__('turret_tracking_node')

        # ── Parameters ──────────────────────────────────────────────────────
        for name, default in [
            ('kp_pan',  0.08), ('ki_pan',  0.0), ('kd_pan',  0.02),
            ('kp_tilt', 0.06), ('ki_tilt', 0.0), ('kd_tilt', 0.02),
            ('pan_deadband',  0.05),
            ('tilt_deadband', 0.05),
            ('fire_deadband', 0.07),
            ('suppress_duration', 2.0),
            ('max_pan_deg',  80.0),
            ('max_tilt_deg', 45.0),
        ]:
            self.declare_parameter(name, default)

        def gp(n): return self.get_parameter(n).value

        self._pan_pid  = PID(gp('kp_pan'),  gp('ki_pan'),  gp('kd_pan'),
                             -gp('max_pan_deg'),  gp('max_pan_deg'))
        self._tilt_pid = PID(gp('kp_tilt'), gp('ki_tilt'), gp('kd_tilt'),
                             -gp('max_tilt_deg'), gp('max_tilt_deg'))
        self._pan_db   = gp('pan_deadband')
        self._tilt_db  = gp('tilt_deadband')
        self._fire_db  = gp('fire_deadband')
        self._suppress_dur = gp('suppress_duration')

        # State
        self._enabled       = False
        self._pan_deg       = 0.0
        self._tilt_deg      = 0.0
        self._suppressing   = False
        self._suppress_end  = 0.0

        # Publishers
        self._pan_pub    = self.create_publisher(Float32, '/ghrs/turret_pan',   10)
        self._tilt_pub   = self.create_publisher(Float32, '/ghrs/turret_tilt',  10)
        self._trig_pub   = self.create_publisher(Bool,    '/ghrs/trigger',      10)
        self._pump_pub   = self.create_publisher(Bool,    '/ghrs/pump_cmd',     10)
        self._status_pub = self.create_publisher(String,  '/ghrs/tracking_status', 10)

        # Subscribers
        if MSGS_AVAILABLE:
            self.create_subscription(
                FireDetection, '/ghrs/fire_detection', self._fire_cb, 10)
        self.create_subscription(
            Point, '/ghrs/track_target', self._target_cb, 10)

        # Services
        self.create_service(SetBool, '/ghrs/tracking_enable', self._enable_svc)

        # Watchdog
        self._last_target_time = 0.0
        self.create_timer(0.05, self._watchdog_cb)

        self.get_logger().info('TurretTrackingNode ready.')

    # ────────────────────────────────────────────────────────────────────────
    def _fire_cb(self, msg):
        if not self._enabled or not msg.fire_detected:
            return
        if msg.centroid_x == 0 and msg.centroid_y == 0:
            return
        # Convert centroid to normalised error (-1..1)
        # Assumes frame width/height from bbox
        # Use direct pixel normalise via bbox centre
        norm_x = (msg.bbox_x + msg.bbox_w / 2.0) - 0.5   # -0.5..+0.5
        norm_y = (msg.bbox_y + msg.bbox_h / 2.0) - 0.5
        self._track(norm_x * 2.0, -norm_y * 2.0)          # invert Y (cam vs servo)

    def _target_cb(self, msg: Point):
        if not self._enabled:
            return
        self._track(float(msg.x), float(msg.y))

    def _track(self, err_x: float, err_y: float):
        """err_x, err_y in normalised -1..1 (0=centre)."""
        self._last_target_time = time.monotonic()

        # Deadband
        if abs(err_x) < self._pan_db:
            err_x = 0.0
        if abs(err_y) < self._tilt_db:
            err_y = 0.0

        pan_delta  = self._pan_pid.update(err_x)
        tilt_delta = self._tilt_pid.update(err_y)

        self._pan_deg  = max(-80.0, min(80.0,  self._pan_deg  + pan_delta))
        self._tilt_deg = max(-45.0, min(45.0,  self._tilt_deg + tilt_delta))

        pan_msg  = Float32(); pan_msg.data  = self._pan_deg
        tilt_msg = Float32(); tilt_msg.data = self._tilt_deg
        self._pan_pub.publish(pan_msg)
        self._tilt_pub.publish(tilt_msg)

        # Fire suppression: trigger when on-target and not already suppressing
        on_target = abs(err_x) < self._fire_db and abs(err_y) < self._fire_db
        now = time.monotonic()

        if on_target and not self._suppressing:
            self._suppressing  = True
            self._suppress_end = now + self._suppress_dur
            self._set_suppression(True)

        if self._suppressing and now > self._suppress_end:
            self._suppressing = False
            self._set_suppression(False)

        # Status
        s = String()
        s.data = (f'pan={self._pan_deg:.1f}° tilt={self._tilt_deg:.1f}° '
                  f'{"SUPPRESSING" if self._suppressing else "TRACKING"}')
        self._status_pub.publish(s)

    def _set_suppression(self, on: bool):
        t = Bool(); t.data = on
        p = Bool(); p.data = on
        self._trig_pub.publish(t)
        self._pump_pub.publish(p)
        self.get_logger().info(f'{"FIRE SUPPRESSION ON" if on else "Suppression off"}')

    def _watchdog_cb(self):
        """Stop tracking if no target received for 1 second."""
        if time.monotonic() - self._last_target_time > 1.0:
            if self._suppressing:
                self._suppressing = False
                self._set_suppression(False)

    def _enable_svc(self, request, response):
        self._enabled = request.data
        if not self._enabled:
            self._pan_pid.reset()
            self._tilt_pid.reset()
            if self._suppressing:
                self._suppressing = False
                self._set_suppression(False)
        response.success = True
        response.message = 'tracking=' + str(self._enabled)
        self.get_logger().info(
            f'Tracking {"enabled" if self._enabled else "disabled"}')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = TurretTrackingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
