#!/usr/bin/env python3
"""
GHRS Servo Controller Node  –  PCA9685 I2C 0x40
==================================================
Drives all servo channels on the PCA9685:
  ch0 – turret pan      (0°–180°, centre 90°)
  ch1 – turret tilt     (0°–180°, centre 90°)
  ch2 – trigger servo   (fire pulse)
  ch3 – hose pan        (650–2250 µs, centre 1450 µs)
  ch4 – hose tilt       (1600–2100 µs, centre 1800 µs)
  ch5 – auxiliary

Subscribed topics:
  /ghrs/servo_cmd          (ghrs_msgs/ServoCommand) – single channel command
  /ghrs/turret_pan         (std_msgs/Float32)  – degrees
  /ghrs/turret_tilt        (std_msgs/Float32)  – degrees
  /ghrs/hose_pan           (std_msgs/Float32)  – degrees (maps to µs range)
  /ghrs/hose_tilt          (std_msgs/Float32)  – degrees (maps to µs range)
  /ghrs/trigger            (std_msgs/Bool)     – fire trigger

Published topics:
  /ghrs/servo_state        (std_msgs/Float32MultiArray) – current pulse_us[0..5]

Services:
  /ghrs/servo_center_all   (std_srvs/Trigger)

Parameters:
  i2c_bus    (int, default 1)
  i2c_addr   (int, default 0x40)
  pwm_freq   (int, default 50)    Hz

Author : Anmar Arafat Al-Momani
Version: 1.0.0
"""

import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, Float32MultiArray
from std_srvs.srv import Trigger

try:
    from ghrs_msgs.msg import ServoCommand
    MSGS_AVAILABLE = True
except ImportError:
    MSGS_AVAILABLE = False

try:
    import board
    import busio
    from adafruit_pca9685 import PCA9685
    PCA_AVAILABLE = True
except ImportError:
    PCA_AVAILABLE = False


# ─── Servo calibration table ─────────────────────────────────────────────────
SERVO_CFG = {
    # channel: (min_us, centre_us, max_us)
    0: (500,  1500, 2500),   # turret pan
    1: (500,  1500, 2500),   # turret tilt
    2: (500,  1500, 2500),   # trigger
    3: (650,  1450, 2250),   # hose pan  (wrist)
    4: (1600, 1800, 2100),   # hose tilt (wrist_rot) – narrow range
    5: (500,  1500, 2500),   # auxiliary
}


def us_to_duty(pulse_us: float, freq_hz: int = 50) -> int:
    """Convert microseconds to PCA9685 16-bit duty cycle value."""
    period_us = 1_000_000.0 / freq_hz
    return int((pulse_us / period_us) * 0xFFFF)


def angle_to_us(angle_deg: float, ch: int) -> float:
    """Map -90..+90 angle to pulse width for given channel."""
    mn, ctr, mx = SERVO_CFG[ch]
    half = (mx - mn) / 2.0
    return ctr + (angle_deg / 90.0) * half


class ServoControllerNode(Node):
    """PCA9685 servo controller node."""

    def __init__(self):
        super().__init__('servo_controller')

        self.declare_parameter('i2c_bus',   1)
        self.declare_parameter('i2c_addr',  0x40)
        self.declare_parameter('pwm_freq',  50)

        self._bus_num  = self.get_parameter('i2c_bus').value
        self._addr     = self.get_parameter('i2c_addr').value
        self._freq     = self.get_parameter('pwm_freq').value

        self._lock = threading.Lock()
        self._pca  = None
        # current pulse per channel
        self._state = {ch: SERVO_CFG[ch][1] for ch in SERVO_CFG}

        if PCA_AVAILABLE:
            try:
                i2c  = busio.I2C(board.SCL, board.SDA)
                self._pca = PCA9685(i2c, address=self._addr)
                self._pca.frequency = self._freq
                self._center_all()
                self.get_logger().info(
                    f'PCA9685 on I2C 0x{self._addr:02X} ready @ {self._freq} Hz')
            except Exception as e:
                self.get_logger().error(f'PCA9685 init failed: {e}')
                self._pca = None
        else:
            self.get_logger().warn('adafruit_pca9685 not installed – simulation mode.')

        # ── Subscribers ─────────────────────────────────────────────────────
        if MSGS_AVAILABLE:
            self.create_subscription(ServoCommand, '/ghrs/servo_cmd',
                                     self._servo_cmd_cb, 10)

        self.create_subscription(Float32, '/ghrs/turret_pan',
                                 lambda m: self._ch_angle_cb(m, 0), 10)
        self.create_subscription(Float32, '/ghrs/turret_tilt',
                                 lambda m: self._ch_angle_cb(m, 1), 10)
        self.create_subscription(Float32, '/ghrs/hose_pan',
                                 lambda m: self._ch_angle_cb(m, 3), 10)
        self.create_subscription(Float32, '/ghrs/hose_tilt',
                                 lambda m: self._ch_angle_cb(m, 4), 10)
        self.create_subscription(Bool,    '/ghrs/trigger',
                                 self._trigger_cb, 10)

        # ── Publishers ──────────────────────────────────────────────────────
        self._state_pub = self.create_publisher(Float32MultiArray,
                                                '/ghrs/servo_state', 10)

        # ── Services ────────────────────────────────────────────────────────
        self.create_service(Trigger, '/ghrs/servo_center_all', self._center_cb)

        # Feedback timer
        self.create_timer(0.1, self._state_pub_cb)
        self.get_logger().info('ServoControllerNode ready.')

    # ────────────────────────────────────────────────────────────────────────
    def _servo_cmd_cb(self, msg):
        ch = int(msg.channel)
        if ch not in SERVO_CFG:
            self.get_logger().warn(f'Invalid channel {ch}')
            return
        if msg.use_angle:
            pulse = angle_to_us(float(msg.angle_deg), ch)
        else:
            pulse = float(msg.pulse_us)
        self._set_channel(ch, pulse)

    def _ch_angle_cb(self, msg: Float32, ch: int):
        pulse = angle_to_us(float(msg.data), ch)
        self._set_channel(ch, pulse)

    def _trigger_cb(self, msg: Bool):
        """Pull trigger servo forward on True, release on False."""
        mn, ctr, mx = SERVO_CFG[2]
        pulse = mx if msg.data else ctr
        self._set_channel(2, pulse)

    def _center_cb(self, _request, response):
        self._center_all()
        response.success = True
        response.message = 'All servos centred'
        return response

    def _state_pub_cb(self):
        s = Float32MultiArray()
        s.data = [float(self._state.get(ch, 0)) for ch in range(6)]
        self._state_pub.publish(s)

    # ────────────────────────────────────────────────────────────────────────
    def _set_channel(self, ch: int, pulse_us: float):
        mn, ctr, mx = SERVO_CFG[ch]
        pulse_us = max(mn, min(mx, pulse_us))
        with self._lock:
            self._state[ch] = pulse_us
            if self._pca:
                try:
                    self._pca.channels[ch].duty_cycle = us_to_duty(
                        pulse_us, self._freq)
                except Exception as e:
                    self.get_logger().warn(f'Servo ch{ch} write error: {e}')
            else:
                self.get_logger().debug(f'[SIM] ch{ch} = {pulse_us:.0f} µs')

    def _center_all(self):
        for ch in SERVO_CFG:
            self._set_channel(ch, SERVO_CFG[ch][1])

    def destroy_node(self):
        self._center_all()
        if self._pca:
            self._pca.deinit()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ServoControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
