#!/usr/bin/env python3
"""
GHRS Motor Controller Node
============================
Drives the two BTS7960 H-bridge motor drivers via RPi GPIO.

Subscribed topics:
  /cmd_vel          (geometry_msgs/Twist)   – velocity commands from Nav2 or teleop
  /ghrs/drive_raw   (std_msgs/Float32MultiArray) – raw [left, right] -1..1

Published topics:
  /ghrs/motor_feedback (std_msgs/Float32MultiArray) – current [left, right] PWM

Services:
  /ghrs/estop        (std_srvs/SetBool)  – emergency stop

Parameters:
  l_rpwm  (int, default 24)
  l_lpwm  (int, default 23)
  r_rpwm  (int, default 13)
  r_lpwm  (int, default 12)
  invert_lr (bool, default True)
  max_speed (float, default 1.0)
  wheel_separation (float, default 0.35) metres

Author : Anmar Arafat Al-Momani
Version: 1.0.0
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import SetBool
import math

try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False


# ─── PWM frequency ───────────────────────────────────────────────────────────
PWM_FREQ = 1000  # Hz


class MotorControllerNode(Node):
    """BTS7960 dual H-bridge motor controller."""

    def __init__(self):
        super().__init__('motor_controller')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('l_rpwm', 24)
        self.declare_parameter('l_lpwm', 23)
        self.declare_parameter('r_rpwm', 13)
        self.declare_parameter('r_lpwm', 12)
        self.declare_parameter('invert_lr', True)
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('wheel_separation', 0.35)
        self.declare_parameter('cmd_vel_timeout', 0.5)

        self.l_rpwm_pin = self.get_parameter('l_rpwm').value
        self.l_lpwm_pin = self.get_parameter('l_lpwm').value
        self.r_rpwm_pin = self.get_parameter('r_rpwm').value
        self.r_lpwm_pin = self.get_parameter('r_lpwm').value
        self.invert_lr  = self.get_parameter('invert_lr').value
        self.max_speed  = self.get_parameter('max_speed').value
        self.wheel_sep  = self.get_parameter('wheel_separation').value
        self.cmd_timeout = self.get_parameter('cmd_vel_timeout').value

        # ── GPIO setup ───────────────────────────────────────────────────────
        self._pwm_l_fwd = None
        self._pwm_l_rev = None
        self._pwm_r_fwd = None
        self._pwm_r_rev = None
        self._estop = False

        if GPIO_AVAILABLE:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            for pin in [self.l_rpwm_pin, self.l_lpwm_pin,
                        self.r_rpwm_pin, self.r_lpwm_pin]:
                GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)

            self._pwm_l_fwd = GPIO.PWM(self.l_rpwm_pin, PWM_FREQ)
            self._pwm_l_rev = GPIO.PWM(self.l_lpwm_pin, PWM_FREQ)
            self._pwm_r_fwd = GPIO.PWM(self.r_rpwm_pin, PWM_FREQ)
            self._pwm_r_rev = GPIO.PWM(self.r_lpwm_pin, PWM_FREQ)
            for pwm in [self._pwm_l_fwd, self._pwm_l_rev,
                        self._pwm_r_fwd, self._pwm_r_rev]:
                pwm.start(0)
            self.get_logger().info('GPIO PWM initialised.')
        else:
            self.get_logger().warn('RPi.GPIO not available – running in simulation mode.')

        # ── State ────────────────────────────────────────────────────────────
        self._cur_left  = 0.0
        self._cur_right = 0.0
        self._last_cmd_time = self.get_clock().now()

        # ── Subscribers ─────────────────────────────────────────────────────
        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_cb, 10)
        self.create_subscription(Float32MultiArray, '/ghrs/drive_raw',
                                 self._raw_drive_cb, 10)

        # ── Publishers ──────────────────────────────────────────────────────
        self._fb_pub = self.create_publisher(Float32MultiArray,
                                             '/ghrs/motor_feedback', 10)

        # ── Services ────────────────────────────────────────────────────────
        self.create_service(SetBool, '/ghrs/estop', self._estop_cb)

        # ── Watchdog timer (cmd_vel timeout) ────────────────────────────────
        self.create_timer(0.1, self._watchdog_cb)

        self.get_logger().info('MotorControllerNode ready.')

    # ────────────────────────────────────────────────────────────────────────
    def _cmd_vel_cb(self, msg: Twist):
        """Convert Twist → differential drive wheel speeds."""
        if self._estop:
            return
        v = msg.linear.x
        w = msg.angular.z
        # unicycle → differential
        left  = v - (w * self.wheel_sep / 2.0)
        right = v + (w * self.wheel_sep / 2.0)
        # normalise to -1..1
        max_val = max(abs(left), abs(right), 1.0)
        left  /= max_val
        right /= max_val
        self._set_speeds(left * self.max_speed, right * self.max_speed)
        self._last_cmd_time = self.get_clock().now()

    def _raw_drive_cb(self, msg: Float32MultiArray):
        """Direct raw [-1,1] wheel speed command."""
        if self._estop or len(msg.data) < 2:
            return
        self._set_speeds(
            float(msg.data[0]) * self.max_speed,
            float(msg.data[1]) * self.max_speed,
        )
        self._last_cmd_time = self.get_clock().now()

    def _estop_cb(self, request, response):
        self._estop = request.data
        if self._estop:
            self._stop_motors()
            self.get_logger().warn('E-STOP ACTIVATED')
        else:
            self.get_logger().info('E-STOP cleared')
        response.success = True
        response.message = 'estop=' + str(self._estop)
        return response

    def _watchdog_cb(self):
        """Stop motors if no command received within timeout."""
        elapsed = (self.get_clock().now() - self._last_cmd_time).nanoseconds * 1e-9
        if elapsed > self.cmd_timeout and (self._cur_left != 0 or self._cur_right != 0):
            self._stop_motors()
        # publish feedback
        fb = Float32MultiArray()
        fb.data = [self._cur_left, self._cur_right]
        self._fb_pub.publish(fb)

    # ────────────────────────────────────────────────────────────────────────
    def _set_speeds(self, left: float, right: float):
        left  = max(-1.0, min(1.0, left))
        right = max(-1.0, min(1.0, right))
        if self.invert_lr:
            left, right = -left, -right
        self._apply_pwm(left, right)
        self._cur_left  = left
        self._cur_right = right

    def _apply_pwm(self, left: float, right: float):
        if not GPIO_AVAILABLE:
            self.get_logger().debug(f'[SIM] L={left:.2f} R={right:.2f}')
            return
        # Left motor
        if left >= 0:
            self._pwm_l_fwd.ChangeDutyCycle(left * 100)
            self._pwm_l_rev.ChangeDutyCycle(0)
        else:
            self._pwm_l_fwd.ChangeDutyCycle(0)
            self._pwm_l_rev.ChangeDutyCycle(abs(left) * 100)
        # Right motor
        if right >= 0:
            self._pwm_r_fwd.ChangeDutyCycle(right * 100)
            self._pwm_r_rev.ChangeDutyCycle(0)
        else:
            self._pwm_r_fwd.ChangeDutyCycle(0)
            self._pwm_r_rev.ChangeDutyCycle(abs(right) * 100)

    def _stop_motors(self):
        self._apply_pwm(0.0, 0.0)
        self._cur_left  = 0.0
        self._cur_right = 0.0

    def destroy_node(self):
        self._stop_motors()
        if GPIO_AVAILABLE:
            for pwm in [self._pwm_l_fwd, self._pwm_l_rev,
                        self._pwm_r_fwd, self._pwm_r_rev]:
                if pwm:
                    pwm.stop()
            GPIO.cleanup()
        super().destroy_node()


# ─── Entry point ─────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = MotorControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
