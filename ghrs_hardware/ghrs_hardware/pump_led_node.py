#!/usr/bin/env python3
"""
GHRS Pump & LED Node
=====================
Controls the water pump (GPIO 17, active-low) and
status LED (GPIO 27, active-low) via ROS2 topics and services.

Subscribed topics:
  /ghrs/pump_cmd   (std_msgs/Bool)  – True = on
  /ghrs/led_cmd    (std_msgs/Bool)  – True = on

Services:
  /ghrs/pump_set   (std_srvs/SetBool)
  /ghrs/led_set    (std_srvs/SetBool)
  /ghrs/pump_burst (std_srvs/Trigger)  – 2-second burst then off

Published topics:
  /ghrs/actuator_state (std_msgs/Bool[]) – [pump, led]

Parameters:
  pump_pin    (int, default 17)
  led_pin     (int, default 27)
  active_low  (bool, default True)
  burst_sec   (float, default 2.0)

Author : Anmar Arafat Al-Momani
Version: 1.0.0
"""

import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray
from std_srvs.srv import SetBool, Trigger

try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False


class PumpLedNode(Node):
    """GPIO pump and LED controller."""

    def __init__(self):
        super().__init__('pump_led_node')

        self.declare_parameter('pump_pin',   17)
        self.declare_parameter('led_pin',    27)
        self.declare_parameter('active_low', True)
        self.declare_parameter('burst_sec',  2.0)

        self._pump_pin   = self.get_parameter('pump_pin').value
        self._led_pin    = self.get_parameter('led_pin').value
        self._active_low = self.get_parameter('active_low').value
        self._burst_sec  = self.get_parameter('burst_sec').value

        self._pump_on = False
        self._led_on  = False
        self._burst_timer = None

        if GPIO_AVAILABLE:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            GPIO.setup(self._pump_pin, GPIO.OUT,
                       initial=GPIO.HIGH if self._active_low else GPIO.LOW)
            GPIO.setup(self._led_pin,  GPIO.OUT,
                       initial=GPIO.HIGH if self._active_low else GPIO.LOW)
            self.get_logger().info('GPIO pump/LED pins initialised.')
        else:
            self.get_logger().warn('RPi.GPIO unavailable – simulation mode.')

        # Subscribers
        self.create_subscription(Bool, '/ghrs/pump_cmd', self._pump_cb, 10)
        self.create_subscription(Bool, '/ghrs/led_cmd',  self._led_cb,  10)

        # Services
        self.create_service(SetBool, '/ghrs/pump_set',   self._pump_svc)
        self.create_service(SetBool, '/ghrs/led_set',    self._led_svc)
        self.create_service(Trigger, '/ghrs/pump_burst', self._burst_svc)

        # Publisher
        self._state_pub = self.create_publisher(
            Float32MultiArray, '/ghrs/actuator_state', 10)
        self.create_timer(0.5, self._state_pub_cb)

        self.get_logger().info('PumpLedNode ready.')

    # ────────────────────────────────────────────────────────────────────────
    def _set_gpio(self, pin: int, state: bool):
        """Write a GPIO pin respecting active-low polarity."""
        if not GPIO_AVAILABLE:
            self.get_logger().debug(f'[SIM] GPIO pin {pin} = {state}')
            return
        logical = not state if self._active_low else state
        GPIO.output(pin, GPIO.LOW if logical else GPIO.HIGH)

    def _set_pump(self, on: bool):
        self._pump_on = on
        self._set_gpio(self._pump_pin, on)
        self.get_logger().info(f'Pump {"ON" if on else "OFF"}')

    def _set_led(self, on: bool):
        self._led_on = on
        self._set_gpio(self._led_pin, on)
        self.get_logger().info(f'LED {"ON" if on else "OFF"}')

    # ── Subscribers ──────────────────────────────────────────────────────────
    def _pump_cb(self, msg: Bool):
        self._set_pump(msg.data)

    def _led_cb(self, msg: Bool):
        self._set_led(msg.data)

    # ── Services ─────────────────────────────────────────────────────────────
    def _pump_svc(self, request, response):
        self._set_pump(request.data)
        response.success = True
        response.message = 'pump=' + str(self._pump_on)
        return response

    def _led_svc(self, request, response):
        self._set_led(request.data)
        response.success = True
        response.message = 'led=' + str(self._led_on)
        return response

    def _burst_svc(self, _request, response):
        """Activate pump for burst_sec seconds then turn off."""
        if self._burst_timer:
            self._burst_timer.cancel()
        self._set_pump(True)
        self._burst_timer = threading.Timer(
            self._burst_sec, lambda: self._set_pump(False))
        self._burst_timer.start()
        response.success = True
        response.message = f'Pump burst {self._burst_sec}s started'
        return response

    def _state_pub_cb(self):
        msg = Float32MultiArray()
        msg.data = [float(self._pump_on), float(self._led_on)]
        self._state_pub.publish(msg)

    # ────────────────────────────────────────────────────────────────────────
    def destroy_node(self):
        self._set_pump(False)
        self._set_led(False)
        if self._burst_timer:
            self._burst_timer.cancel()
        if GPIO_AVAILABLE:
            GPIO.cleanup([self._pump_pin, self._led_pin])
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PumpLedNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
