#!/usr/bin/env python3
"""
GHRS Navigation Node
======================
Autonomous patrol and navigation for the GHRS rover.

Modes
-----
IDLE         – stationary, waiting
PATROL       – follow GPS waypoints in sequence
TRACK        – rotate in place toward a fire bearing
AVOID        – obstacle avoidance manoeuvre
EMERGENCY    – full stop

The node integrates GPS, IMU heading, and obstacle proximity to produce
/cmd_vel commands.  It does NOT use a full Nav2 costmap – instead it uses
the simple camera-based obstacle proximity signal.

Subscribed topics:
  /gps/fix                  (sensor_msgs/NavSatFix)
  /ghrs/heading             (std_msgs/Float32)
  /ghrs/obstacle_proximity  (std_msgs/Float32MultiArray)  – [L,C,R]
  /ghrs/fire_detection      (ghrs_msgs/FireDetection)

Published topics:
  /cmd_vel                  (geometry_msgs/Twist)
  /ghrs/nav_state           (std_msgs/String)
  /ghrs/waypoint_index      (std_msgs/Int32)

Services:
  /ghrs/nav_start           (std_srvs/Trigger)
  /ghrs/nav_stop            (std_srvs/Trigger)
  /ghrs/nav_set_mode        (std_srvs/SetBool)  – True=patrol False=idle
  /ghrs/add_waypoint        (std_srvs/Trigger)  – add current GPS pos

Parameters:
  waypoints_file   (str,   default '')    JSON file with lat/lon waypoints
  arrival_radius   (float, default 2.0)  metres
  patrol_speed     (float, default 0.3)  m/s
  turn_speed       (float, default 0.5)  rad/s
  obstacle_stop    (float, default 0.15) centre density triggers stop
  obstacle_turn    (float, default 0.10) side density triggers turn

Author : Anmar Arafat Al-Momani
Version: 1.0.0
"""

import json
import math
import os
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32, Float32MultiArray, String, Int32
from std_srvs.srv import Trigger, SetBool

try:
    from ghrs_msgs.msg import FireDetection
    MSGS_AVAILABLE = True
except ImportError:
    MSGS_AVAILABLE = False


# ─── Haversine distance ────────────────────────────────────────────────────
def haversine(lat1, lon1, lat2, lon2) -> float:
    """Return distance in metres between two GPS points."""
    R = 6_371_000.0
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi  = math.radians(lat2 - lat1)
    dlam  = math.radians(lon2 - lon1)
    a = (math.sin(dphi / 2) ** 2
         + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2) ** 2)
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def bearing_to(lat1, lon1, lat2, lon2) -> float:
    """Return bearing in degrees (0=N, 90=E) from point 1 to point 2."""
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dlam = math.radians(lon2 - lon1)
    x = math.sin(dlam) * math.cos(phi2)
    y = (math.cos(phi1) * math.sin(phi2)
         - math.sin(phi1) * math.cos(phi2) * math.cos(dlam))
    b = math.degrees(math.atan2(x, y))
    return (b + 360) % 360


def angle_diff(a: float, b: float) -> float:
    """Signed difference between two bearings (−180..+180)."""
    d = (a - b + 180) % 360 - 180
    return d


class NavigationNode(Node):
    """GPS-guided autonomous navigation node."""

    MODE_IDLE    = 'IDLE'
    MODE_PATROL  = 'PATROL'
    MODE_TRACK   = 'TRACK'
    MODE_AVOID   = 'AVOID'
    MODE_EMERG   = 'EMERGENCY'

    def __init__(self):
        super().__init__('navigation_node')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('arrival_radius', 2.0)
        self.declare_parameter('patrol_speed',   0.3)
        self.declare_parameter('turn_speed',     0.5)
        self.declare_parameter('obstacle_stop',  0.15)
        self.declare_parameter('obstacle_turn',  0.10)
        self.declare_parameter('heading_kp',     0.03)

        self._wp_file      = self.get_parameter('waypoints_file').value
        self._arrival_r    = self.get_parameter('arrival_radius').value
        self._patrol_spd   = self.get_parameter('patrol_speed').value
        self._turn_spd     = self.get_parameter('turn_speed').value
        self._ob_stop      = self.get_parameter('obstacle_stop').value
        self._ob_turn      = self.get_parameter('obstacle_turn').value
        self._heading_kp   = self.get_parameter('heading_kp').value

        # ── State ────────────────────────────────────────────────────────────
        self._mode       = self.MODE_IDLE
        self._lat = self._lon = 0.0
        self._gps_valid  = False
        self._heading    = 0.0
        self._obstacle   = [0.0, 0.0, 0.0]  # L C R
        self._fire_bearing = None
        self._waypoints  = []
        self._wp_idx     = 0
        self._avoid_end  = 0.0

        self._load_waypoints()

        # ── Publishers ───────────────────────────────────────────────────────
        self._cmd_vel_pub = self.create_publisher(Twist,  '/cmd_vel',            10)
        self._state_pub   = self.create_publisher(String, '/ghrs/nav_state',     10)
        self._wp_pub      = self.create_publisher(Int32,  '/ghrs/waypoint_index',10)

        # ── Subscribers ──────────────────────────────────────────────────────
        self.create_subscription(NavSatFix, '/gps/fix',
                                 self._gps_cb, 10)
        self.create_subscription(Float32, '/ghrs/heading',
                                 self._heading_cb, 10)
        self.create_subscription(Float32MultiArray, '/ghrs/obstacle_proximity',
                                 self._obstacle_cb, 10)
        if MSGS_AVAILABLE:
            self.create_subscription(FireDetection, '/ghrs/fire_detection',
                                     self._fire_cb, 10)

        # ── Services ─────────────────────────────────────────────────────────
        self.create_service(Trigger, '/ghrs/nav_start',   self._start_cb)
        self.create_service(Trigger, '/ghrs/nav_stop',    self._stop_cb)
        self.create_service(SetBool, '/ghrs/nav_set_mode',self._mode_cb)
        self.create_service(Trigger, '/ghrs/add_waypoint',self._add_wp_cb)

        # ── Control loop ─────────────────────────────────────────────────────
        self.create_timer(0.1, self._control_loop)

        self.get_logger().info(
            f'NavigationNode ready. {len(self._waypoints)} waypoints loaded.')

    # ────────────────────────────────────────────────────────────────────────
    def _load_waypoints(self):
        if self._wp_file and os.path.exists(self._wp_file):
            try:
                with open(self._wp_file) as f:
                    data = json.load(f)
                self._waypoints = [(wp['lat'], wp['lon'])
                                   for wp in data.get('waypoints', [])]
                self.get_logger().info(
                    f'Loaded {len(self._waypoints)} waypoints from {self._wp_file}')
            except Exception as e:
                self.get_logger().error(f'Waypoint load error: {e}')

    # ── Callbacks ────────────────────────────────────────────────────────────
    def _gps_cb(self, msg: NavSatFix):
        if not math.isnan(msg.latitude):
            self._lat       = msg.latitude
            self._lon       = msg.longitude
            self._gps_valid = True

    def _heading_cb(self, msg: Float32):
        self._heading = float(msg.data)

    def _obstacle_cb(self, msg: Float32MultiArray):
        if len(msg.data) >= 3:
            self._obstacle = list(msg.data[:3])

    def _fire_cb(self, msg):
        if msg.fire_detected:
            # Estimate bearing from centroid_x (rough horizontal angle)
            # centroid_x normalised to -0.5..0.5
            norm = (msg.centroid_x / 640.0) - 0.5  # assume 640px wide
            angle_offset = norm * 60.0              # ±30° FOV
            self._fire_bearing = (self._heading + angle_offset) % 360
        else:
            self._fire_bearing = None

    # ── Services ─────────────────────────────────────────────────────────────
    def _start_cb(self, _req, response):
        if not self._waypoints:
            response.success = False
            response.message = 'No waypoints loaded'
            return response
        self._mode  = self.MODE_PATROL
        self._wp_idx = 0
        response.success = True
        response.message = 'Patrol started'
        return response

    def _stop_cb(self, _req, response):
        self._mode = self.MODE_IDLE
        self._publish_cmd(0.0, 0.0)
        response.success = True
        response.message = 'Navigation stopped'
        return response

    def _mode_cb(self, request, response):
        self._mode = self.MODE_PATROL if request.data else self.MODE_IDLE
        response.success = True
        response.message = f'mode={self._mode}'
        return response

    def _add_wp_cb(self, _req, response):
        if not self._gps_valid:
            response.success = False
            response.message = 'GPS not valid'
            return response
        self._waypoints.append((self._lat, self._lon))
        self.get_logger().info(
            f'Added waypoint #{len(self._waypoints)}: '
            f'{self._lat:.6f}, {self._lon:.6f}')
        response.success = True
        response.message = f'Waypoint {len(self._waypoints)} added'
        return response

    # ── Control loop ─────────────────────────────────────────────────────────
    def _control_loop(self):
        state_msg = String()

        if self._mode == self.MODE_IDLE or self._mode == self.MODE_EMERG:
            self._publish_cmd(0.0, 0.0)
            state_msg.data = self._mode
            self._state_pub.publish(state_msg)
            return

        L, C, R = self._obstacle

        # ── Obstacle avoidance ───────────────────────────────────────────
        if C > self._ob_stop:
            if time.monotonic() < self._avoid_end:
                # Turn away from heavier side
                turn = self._turn_spd if L > R else -self._turn_spd
                self._publish_cmd(0.0, turn)
                state_msg.data = f'{self.MODE_AVOID} (L={L:.2f} C={C:.2f} R={R:.2f})'
                self._state_pub.publish(state_msg)
                return
            else:
                self._avoid_end = time.monotonic() + 1.5
        elif L > self._ob_turn:
            self._publish_cmd(self._patrol_spd * 0.5, -self._turn_spd * 0.4)
            state_msg.data = f'NUDGE_RIGHT'
            self._state_pub.publish(state_msg)
            return
        elif R > self._ob_turn:
            self._publish_cmd(self._patrol_spd * 0.5,  self._turn_spd * 0.4)
            state_msg.data = f'NUDGE_LEFT'
            self._state_pub.publish(state_msg)
            return

        # ── Fire tracking override ───────────────────────────────────────
        if self._fire_bearing is not None and self._mode == self.MODE_TRACK:
            err = angle_diff(self._fire_bearing, self._heading)
            w = self._heading_kp * err
            w = max(-self._turn_spd, min(self._turn_spd, w))
            self._publish_cmd(0.0, w)
            state_msg.data = f'TRACK_FIRE bearing={self._fire_bearing:.1f}'
            self._state_pub.publish(state_msg)
            return

        # ── Waypoint patrol ──────────────────────────────────────────────
        if self._mode == self.MODE_PATROL and self._waypoints and self._gps_valid:
            wp_lat, wp_lon = self._waypoints[self._wp_idx]
            dist = haversine(self._lat, self._lon, wp_lat, wp_lon)

            if dist < self._arrival_r:
                self.get_logger().info(
                    f'Arrived at waypoint {self._wp_idx + 1}')
                self._wp_idx = (self._wp_idx + 1) % len(self._waypoints)
                state_msg.data = f'ARRIVED wp={self._wp_idx}'
            else:
                target_b = bearing_to(self._lat, self._lon, wp_lat, wp_lon)
                err = angle_diff(target_b, self._heading)
                w = self._heading_kp * err
                w = max(-self._turn_spd, min(self._turn_spd, w))
                v = self._patrol_spd * max(0.0, 1.0 - abs(err) / 90.0)
                self._publish_cmd(v, w)
                state_msg.data = (
                    f'PATROL wp={self._wp_idx+1}/{len(self._waypoints)} '
                    f'dist={dist:.1f}m brg={target_b:.0f}°')

            wp_msg      = Int32()
            wp_msg.data = self._wp_idx
            self._wp_pub.publish(wp_msg)
        else:
            self._publish_cmd(0.0, 0.0)
            state_msg.data = 'PATROL_WAIT (no GPS)'

        self._state_pub.publish(state_msg)

    # ────────────────────────────────────────────────────────────────────────
    def _publish_cmd(self, linear: float, angular: float):
        t = Twist()
        t.linear.x  = float(linear)
        t.angular.z = float(angular)
        self._cmd_vel_pub.publish(t)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
