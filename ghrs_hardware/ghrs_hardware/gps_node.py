#!/usr/bin/env python3
"""
GHRS GPS Node  –  NEO-6M on /dev/ttyAMA0 @ 9600
==================================================
Parses NMEA sentences and publishes a NavSatFix.

Published topics:
  /gps/fix           (sensor_msgs/NavSatFix)
  /gps/vel           (geometry_msgs/TwistWithCovarianceStamped)
  /ghrs/gps_str      (std_msgs/String)  – raw NMEA for logging

Parameters:
  port         (str,   default /dev/ttyAMA0)
  baud         (int,   default 9600)
  frame_id     (str,   default gps_link)

Author : Anmar Arafat Al-Momani
Version: 1.0.0
"""

import serial
import threading
import pynmea2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from geometry_msgs.msg import TwistWithCovarianceStamped
from std_msgs.msg import String


class GpsNode(Node):
    """NEO-6M GPS publisher over serial NMEA."""

    def __init__(self):
        super().__init__('gps_node')

        self.declare_parameter('port',     '/dev/ttyAMA0')
        self.declare_parameter('baud',     9600)
        self.declare_parameter('frame_id', 'gps_link')
        self.declare_parameter('timeout',  2.0)

        self._port     = self.get_parameter('port').value
        self._baud     = self.get_parameter('baud').value
        self._frame_id = self.get_parameter('frame_id').value
        self._timeout  = self.get_parameter('timeout').value

        # Publishers
        self._fix_pub  = self.create_publisher(NavSatFix,  '/gps/fix',      10)
        self._vel_pub  = self.create_publisher(
            TwistWithCovarianceStamped, '/gps/vel', 10)
        self._str_pub  = self.create_publisher(String,     '/ghrs/gps_str', 10)

        # State
        self._ser    = None
        self._thread = None
        self._running = False
        self._last_rmc  = None
        self._last_gga  = None

        self._open_serial()
        if self._ser:
            self._running = True
            self._thread = threading.Thread(
                target=self._read_loop, daemon=True)
            self._thread.start()

        self.get_logger().info(f'GpsNode ready on {self._port}.')

    # ────────────────────────────────────────────────────────────────────────
    def _open_serial(self):
        try:
            self._ser = serial.Serial(
                self._port, self._baud, timeout=self._timeout)
            self.get_logger().info(f'Opened {self._port} @ {self._baud}')
        except serial.SerialException as e:
            self.get_logger().error(f'Cannot open GPS serial: {e}')
            self._ser = None

    def _read_loop(self):
        while self._running and self._ser:
            try:
                line = self._ser.readline().decode('ascii', errors='replace').strip()
            except Exception as e:
                self.get_logger().warn(f'Serial read error: {e}')
                continue

            if not line.startswith('$'):
                continue

            # Publish raw string
            raw_msg = String()
            raw_msg.data = line
            self._str_pub.publish(raw_msg)

            try:
                msg = pynmea2.parse(line)
            except pynmea2.ParseError:
                continue

            now = self.get_clock().now().to_msg()

            if isinstance(msg, pynmea2.GGA):
                self._last_gga = msg
                self._publish_fix(msg, now)

            elif isinstance(msg, pynmea2.RMC):
                self._last_rmc = msg
                if msg.status == 'A':  # Active (valid)
                    self._publish_vel(msg, now)

    def _publish_fix(self, gga, stamp):
        fix = NavSatFix()
        fix.header.stamp    = stamp
        fix.header.frame_id = self._frame_id

        status = NavSatStatus()
        if gga.gps_qual and int(gga.gps_qual) > 0:
            status.status  = NavSatStatus.STATUS_FIX
            status.service = NavSatStatus.SERVICE_GPS
        else:
            status.status  = NavSatStatus.STATUS_NO_FIX
        fix.status = status

        if gga.latitude and gga.longitude:
            fix.latitude  = float(gga.latitude)
            fix.longitude = float(gga.longitude)
            fix.altitude  = float(gga.altitude) if gga.altitude else 0.0
        else:
            fix.latitude = fix.longitude = fix.altitude = float('nan')

        hdop = float(gga.horizontal_dil) if gga.horizontal_dil else 5.0
        pos_var = (hdop * 5.0) ** 2
        fix.position_covariance = [
            pos_var, 0, 0,
            0, pos_var, 0,
            0, 0, pos_var * 4,
        ]
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        self._fix_pub.publish(fix)

    def _publish_vel(self, rmc, stamp):
        twc = TwistWithCovarianceStamped()
        twc.header.stamp    = stamp
        twc.header.frame_id = self._frame_id
        if rmc.spd_over_grnd:
            speed_ms = float(rmc.spd_over_grnd) * 0.514444  # knots → m/s
            twc.twist.twist.linear.x = speed_ms
        twc.twist.covariance[0]  = 0.25
        twc.twist.covariance[7]  = 0.25
        twc.twist.covariance[14] = 0.25
        self._vel_pub.publish(twc)

    # ────────────────────────────────────────────────────────────────────────
    def destroy_node(self):
        self._running = False
        if self._ser:
            try:
                self._ser.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GpsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
