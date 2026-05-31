#!/usr/bin/env python3
"""
GHRS Pesticide Spraying Coordinator Node
==========================================
Orchestrates the pesticide spraying sequence when plant disease is detected
or when manually triggered. Controls the pan-tilt hose turret, pump, and LED.

Subscribed topics:
  /ghrs/plant_health       (ghrs_msgs/PlantHealth)
  /ghrs/tracking_status    (std_msgs/String)

Published topics:
  /cmd_vel                 (geometry_msgs/Twist)   – zero during spraying
  /ghrs/pump_cmd           (std_msgs/Bool)
  /ghrs/led_cmd            (std_msgs/Bool)
  /ghrs/spray_state        (std_msgs/String)

Services called (client):
  /ghrs/tracking_enable    (std_srvs/SetBool)
  /ghrs/estop              (std_srvs/SetBool)

Services offered:
  /ghrs/manual_spray       (std_srvs/Trigger)   – manually trigger spray
  /ghrs/spray_cancel       (std_srvs/Trigger)   – abort spray

Parameters:
  auto_spray           (bool,  default True)
  spray_duration       (float, default 5.0)    seconds to spray per target
  spray_timeout        (float, default 30.0)   max seconds before abort
  pump_pre_delay       (float, default 0.5)    seconds before pump on

Author : Anmar Arafat Al-Momani
Version: 1.0.0
"""

import time
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, SetBool

try:
    from ghrs_msgs.msg import PlantHealth
    MSGS_AVAILABLE = True
except ImportError:
    MSGS_AVAILABLE = False


class PesticideSprayNode(Node):
    """Pesticide spraying coordinator for plant disease treatment."""

    STATE_IDLE      = 'IDLE'
    STATE_DETECTED  = 'DISEASE_DETECTED'
    STATE_SPRAYING  = 'SPRAYING'
    STATE_DONE      = 'SPRAY_COMPLETE'
    STATE_TIMEOUT   = 'SPRAY_TIMEOUT'

    def __init__(self):
        super().__init__('pesticide_spray_node')

        self.declare_parameter('auto_spray',     True)
        self.declare_parameter('spray_duration', 5.0)
        self.declare_parameter('spray_timeout',  30.0)
        self.declare_parameter('pump_pre_delay', 0.5)

        self._auto          = self.get_parameter('auto_spray').value
        self._duration      = self.get_parameter('spray_duration').value
        self._timeout       = self.get_parameter('spray_timeout').value
        self._pre_delay     = self.get_parameter('pump_pre_delay').value

        self._state         = self.STATE_IDLE
        self._spray_start   = 0.0
        self._running       = True

        # Publishers
        self._cmd_pub    = self.create_publisher(Twist,  '/cmd_vel',            10)
        self._pump_pub   = self.create_publisher(Bool,   '/ghrs/pump_cmd',      10)
        self._led_pub    = self.create_publisher(Bool,   '/ghrs/led_cmd',       10)
        self._state_pub  = self.create_publisher(String, '/ghrs/spray_state',   10)

        # Subscribers
        if MSGS_AVAILABLE:
            self.create_subscription(
                PlantHealth, '/ghrs/plant_health', self._plant_cb, 10)

        # Service clients
        self._tracking_client = self.create_client(SetBool, '/ghrs/tracking_enable')
        self._estop_client    = self.create_client(SetBool, '/ghrs/estop')

        # Services offered
        self.create_service(Trigger, '/ghrs/manual_spray',   self._manual_cb)
        self.create_service(Trigger, '/ghrs/spray_cancel',   self._cancel_cb)

        # Watchdog
        self.create_timer(0.5, self._watchdog_cb)

        self.get_logger().info('PesticideSprayNode ready.')

    # ─────────────────────────────────────────────────────────────────────
    def _plant_cb(self, msg):
        """Trigger spray when a diseased plant is detected."""
        if msg.label.lower() in ('healthy', 'unknown'):
            return
        if self._state == self.STATE_IDLE:
            self._state = self.STATE_DETECTED
            self.get_logger().warn(
                f'DISEASED PLANT DETECTED: {msg.label}  '
                f'conf={msg.confidence:.2f}  lat={msg.latitude:.6f}  lon={msg.longitude:.6f}')
            if self._auto:
                self._begin_spray()

    def _begin_spray(self):
        if self._state == self.STATE_SPRAYING:
            return
        self._state       = self.STATE_SPRAYING
        self._spray_start = time.monotonic()
        self.get_logger().warn('>>> PESTICIDE SPRAY SEQUENCE STARTED <<<')

        # Pause drive
        self._publish_cmd(0.0, 0.0)

        # Enable turret tracking to aim at target
        self._call_set_bool(self._tracking_client, True)

        def _delayed_pump():
            time.sleep(self._pre_delay)
            self._set_pump(True)
            self._set_led(True)

        threading.Thread(target=_delayed_pump, daemon=True).start()
        self._publish_state()

    def _end_spray(self, reason: str):
        self.get_logger().info(f'Spray ended: {reason}')
        self._set_pump(False)
        self._set_led(False)
        self._call_set_bool(self._tracking_client, False)
        self._state = self.STATE_DONE
        self._publish_state()

    # ── Helpers ──────────────────────────────────────────────────────────
    def _set_pump(self, on: bool):
        m = Bool(); m.data = on
        self._pump_pub.publish(m)
        self.get_logger().info(f'Pump {"ON" if on else "OFF"}')

    def _set_led(self, on: bool):
        m = Bool(); m.data = on
        self._led_pub.publish(m)

    def _publish_cmd(self, v, w):
        t = Twist()
        t.linear.x  = float(v)
        t.angular.z = float(w)
        self._cmd_pub.publish(t)

    def _publish_state(self):
        s = String(); s.data = self._state
        self._state_pub.publish(s)

    def _call_set_bool(self, client, value: bool):
        if client.service_is_ready():
            req = SetBool.Request(); req.data = value
            client.call_async(req)

    # ── Services ─────────────────────────────────────────────────────────
    def _manual_cb(self, _req, response):
        self._state = self.STATE_DETECTED
        self._begin_spray()
        response.success = True
        response.message = 'Manual spray started'
        return response

    def _cancel_cb(self, _req, response):
        self._end_spray('manual cancel')
        response.success = True
        response.message = 'Spray cancelled'
        return response

    def _watchdog_cb(self):
        if self._state == self.STATE_SPRAYING:
            elapsed = time.monotonic() - self._spray_start
            if elapsed >= self._duration:
                self._end_spray(f'duration complete ({elapsed:.1f}s)')
            elif elapsed > self._timeout:
                self._end_spray(f'timeout after {elapsed:.0f}s')
                self._state = self.STATE_TIMEOUT
        # Reset to IDLE after DONE so next plant triggers again
        elif self._state == self.STATE_DONE:
            self._state = self.STATE_IDLE
        self._publish_state()

    # ─────────────────────────────────────────────────────────────────────
    def destroy_node(self):
        self._running = False
        self._set_pump(False)
        self._set_led(False)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PesticideSprayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
