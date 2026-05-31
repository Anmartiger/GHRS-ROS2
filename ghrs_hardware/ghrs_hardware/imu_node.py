#!/usr/bin/env python3
"""
GHRS IMU Node  –  BNO08x (I2C 0x4B, bus 1)
=============================================
Publishes orientation, angular velocity and linear acceleration.

Published topics:
  /imu/data        (sensor_msgs/Imu)
  /ghrs/heading    (std_msgs/Float32)   – fused yaw 0-360 degrees
  /ghrs/imu_raw    (sensor_msgs/Imu)    – raw (no covariance filled)

Parameters:
  i2c_bus      (int,   default 1)
  i2c_addr     (int,   default 0x4B)
  publish_rate (float, default 50.0)  Hz
  frame_id     (str,   default "imu_link")

Author : Anmar Arafat Al-Momani
Version: 1.0.0
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from builtin_interfaces.msg import Time

try:
    from adafruit_extended_bus import ExtendedI2C as I2C
    from adafruit_bno08x.i2c import BNO08X_I2C
    from adafruit_bno08x import (
        BNO_REPORT_ROTATION_VECTOR,
        BNO_REPORT_GYROSCOPE,
        BNO_REPORT_ACCELEROMETER,
    )
    HW_AVAILABLE = True
except ImportError:
    HW_AVAILABLE = False


class ImuNode(Node):
    """BNO08x IMU publisher."""

    def __init__(self):
        super().__init__('imu_node')

        self.declare_parameter('i2c_bus',      1)
        self.declare_parameter('i2c_addr',     0x4B)
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('frame_id',     'imu_link')

        self._bus_num  = self.get_parameter('i2c_bus').value
        self._addr     = self.get_parameter('i2c_addr').value
        self._rate     = self.get_parameter('publish_rate').value
        self._frame_id = self.get_parameter('frame_id').value

        self._bno = None
        self._heading = 0.0

        if HW_AVAILABLE:
            try:
                i2c = I2C(self._bus_num)
                self._bno = BNO08X_I2C(i2c, address=self._addr)
                self._bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
                self._bno.enable_feature(BNO_REPORT_GYROSCOPE)
                self._bno.enable_feature(BNO_REPORT_ACCELEROMETER)
                self.get_logger().info(
                    f'BNO08x found on I2C bus {self._bus_num} addr 0x{self._addr:02X}')
            except Exception as e:
                self.get_logger().error(f'BNO08x init failed: {e}')
                self._bno = None
        else:
            self.get_logger().warn('adafruit_bno08x not installed – simulation mode.')

        # Publishers
        self._imu_pub     = self.create_publisher(Imu,     '/imu/data',      10)
        self._heading_pub = self.create_publisher(Float32, '/ghrs/heading',  10)
        self._raw_pub     = self.create_publisher(Imu,     '/ghrs/imu_raw',  10)

        self.create_timer(1.0 / self._rate, self._publish_cb)
        self.get_logger().info(f'ImuNode ready at {self._rate} Hz.')

    # ────────────────────────────────────────────────────────────────────────
    def _publish_cb(self):
        now = self.get_clock().now().to_msg()

        if self._bno is not None:
            try:
                qi, qj, qk, qreal = self._bno.quaternion          # w last from BNO
                gx, gy, gz        = self._bno.gyro
                ax, ay, az        = self._bno.acceleration
            except Exception as e:
                self.get_logger().warn(f'IMU read error: {e}')
                return
        else:
            # Simulation: publish zeros
            qi, qj, qk, qreal = 0.0, 0.0, 0.0, 1.0
            gx = gy = gz = 0.0
            ax = ay = az = 0.0

        # ── Convert quaternion → yaw ─────────────────────────────────────
        # BNO08x reports rotation vector as (i,j,k,real) → (x,y,z,w)
        yaw_rad = math.atan2(
            2.0 * (qreal * qk + qi * qj),
            1.0 - 2.0 * (qj * qj + qk * qk)
        )
        self._heading = (math.degrees(yaw_rad) + 360.0) % 360.0

        # ── Build Imu message ────────────────────────────────────────────
        imu_msg = Imu()
        imu_msg.header.stamp    = now
        imu_msg.header.frame_id = self._frame_id

        imu_msg.orientation.x = float(qi)
        imu_msg.orientation.y = float(qj)
        imu_msg.orientation.z = float(qk)
        imu_msg.orientation.w = float(qreal)
        imu_msg.orientation_covariance = [
            0.01, 0, 0,
            0, 0.01, 0,
            0, 0, 0.01,
        ]

        imu_msg.angular_velocity.x = float(gx)
        imu_msg.angular_velocity.y = float(gy)
        imu_msg.angular_velocity.z = float(gz)
        imu_msg.angular_velocity_covariance = [
            0.005, 0, 0,
            0, 0.005, 0,
            0, 0, 0.005,
        ]

        imu_msg.linear_acceleration.x = float(ax)
        imu_msg.linear_acceleration.y = float(ay)
        imu_msg.linear_acceleration.z = float(az)
        imu_msg.linear_acceleration_covariance = [
            0.02, 0, 0,
            0, 0.02, 0,
            0, 0, 0.02,
        ]

        self._imu_pub.publish(imu_msg)
        self._raw_pub.publish(imu_msg)

        heading_msg = Float32()
        heading_msg.data = self._heading
        self._heading_pub.publish(heading_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
