#!/usr/bin/env python3
"""
GHRS Report Generation Node
==============================
Aggregates mission data from all nodes and generates structured HTML
and plain-text reports at the end of each patrol mission.

Reports include:
  - Mission summary (start/end time, distance, waypoints visited)
  - Fire events (timestamp, GPS location, suppression outcome)
  - Plant disease detections (label, severity, GPS, image thumbnail path)
  - Obstacle events
  - System health log

Subscribed topics:
  /ghrs/fire_detection        (ghrs_msgs/FireDetection)
  /ghrs/plant_health          (ghrs_msgs/PlantHealth)
  /ghrs/suppression_state     (std_msgs/String)
  /ghrs/nav_state             (std_msgs/String)
  /gps/fix                    (sensor_msgs/NavSatFix)
  /ghrs/motor_feedback        (std_msgs/Float32MultiArray)

Services:
  /ghrs/generate_report       (std_srvs/Trigger)
  /ghrs/start_mission         (std_srvs/Trigger)
  /ghrs/end_mission           (std_srvs/Trigger)

Published topics:
  /ghrs/report_ready          (std_msgs/String)  – path to generated report

Parameters:
  report_dir    (str, default /tmp/ghrs_reports)
  rover_id      (str, default GHRS)
  operator      (str, default Anmar Arafat Al-Momani)

Author : Anmar Arafat Al-Momani
Version: 1.0.0
"""

import os
import json
import math
import time
from datetime import datetime
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from sensor_msgs.msg import NavSatFix
from std_srvs.srv import Trigger

try:
    from ghrs_msgs.msg import FireDetection, PlantHealth
    MSGS_AVAILABLE = True
except ImportError:
    MSGS_AVAILABLE = False


def haversine(lat1, lon1, lat2, lon2) -> float:
    R = 6_371_000.0
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi  = math.radians(lat2 - lat1)
    dlam  = math.radians(lon2 - lon1)
    a = (math.sin(dphi / 2) ** 2
         + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2) ** 2)
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


class ReportNode(Node):
    """Mission data aggregator and report generator."""

    def __init__(self):
        super().__init__('report_node')

        self.declare_parameter('report_dir', '/tmp/ghrs_reports')
        self.declare_parameter('rover_id',   'GHRS')
        self.declare_parameter('operator',   'Anmar Arafat Al-Momani')

        self._report_dir = self.get_parameter('report_dir').value
        self._rover_id   = self.get_parameter('rover_id').value
        self._operator   = self.get_parameter('operator').value

        os.makedirs(self._report_dir, exist_ok=True)

        # Mission state
        self._mission_active  = False
        self._mission_start   = None
        self._mission_end     = None
        self._fire_events     = []
        self._plant_events    = []
        self._obstacle_count  = 0
        self._gps_track       = []     # (lat, lon, time)
        self._last_gps        = None
        self._total_distance  = 0.0
        self._nav_states      = []

        # Subscribers
        if MSGS_AVAILABLE:
            self.create_subscription(FireDetection, '/ghrs/fire_detection',
                                     self._fire_cb, 10)
            self.create_subscription(PlantHealth, '/ghrs/plant_health',
                                     self._plant_cb, 10)

        self.create_subscription(String,   '/ghrs/suppression_state',
                                 self._suppress_cb, 10)
        self.create_subscription(String,   '/ghrs/nav_state',
                                 self._nav_state_cb, 10)
        self.create_subscription(NavSatFix,'/gps/fix',
                                 self._gps_cb, 10)
        self.create_subscription(Float32MultiArray, '/ghrs/obstacle_proximity',
                                 self._obstacle_cb, 10)

        # Services
        self.create_service(Trigger, '/ghrs/generate_report', self._gen_cb)
        self.create_service(Trigger, '/ghrs/start_mission',   self._start_cb)
        self.create_service(Trigger, '/ghrs/end_mission',     self._end_cb)

        # Publisher
        self._ready_pub = self.create_publisher(String, '/ghrs/report_ready', 10)

        self.get_logger().info(
            f'ReportNode ready. Reports → {self._report_dir}')

    # ────────────────────────────────────────────────────────────────────────
    def _fire_cb(self, msg):
        if not self._mission_active or not msg.fire_detected:
            return
        entry = {
            'time': datetime.now().isoformat(),
            'confidence':   round(float(msg.confidence), 3),
            'pixel_ratio':  round(float(msg.pixel_ratio), 4),
            'centroid':     (msg.centroid_x, msg.centroid_y),
            'gps':          (self._last_gps[0], self._last_gps[1])
                             if self._last_gps else None,
        }
        # Avoid duplicating rapid detections (< 5 s)
        if self._fire_events:
            last_t = datetime.fromisoformat(self._fire_events[-1]['time'])
            if (datetime.now() - last_t).total_seconds() < 5:
                return
        self._fire_events.append(entry)
        self.get_logger().info(f'Fire event logged ({len(self._fire_events)} total)')

    def _plant_cb(self, msg):
        if not self._mission_active:
            return
        entry = {
            'time':        datetime.now().isoformat(),
            'label':       msg.disease_label,
            'confidence':  round(float(msg.confidence), 3),
            'severity':    round(float(msg.severity), 2),
            'recs':        msg.recommendations,
            'image':       getattr(msg, 'image_path', ''),
            'gps':         (self._last_gps[0], self._last_gps[1])
                            if self._last_gps else None,
        }
        if msg.disease_label != 'Healthy':
            self._plant_events.append(entry)

    def _suppress_cb(self, msg: String):
        pass  # Could annotate fire events with outcome

    def _nav_state_cb(self, msg: String):
        if self._mission_active:
            self._nav_states.append({
                'time': datetime.now().isoformat(),
                'state': msg.data,
            })

    def _gps_cb(self, msg: NavSatFix):
        if math.isnan(msg.latitude):
            return
        if self._last_gps and self._mission_active:
            self._total_distance += haversine(
                self._last_gps[0], self._last_gps[1],
                msg.latitude, msg.longitude)
        self._last_gps = (msg.latitude, msg.longitude)
        if self._mission_active:
            self._gps_track.append(
                (msg.latitude, msg.longitude, time.time()))

    def _obstacle_cb(self, msg: Float32MultiArray):
        if not self._mission_active or len(msg.data) < 3:
            return
        if msg.data[1] > 0.15:   # centre > threshold
            self._obstacle_count += 1

    # ── Services ─────────────────────────────────────────────────────────────
    def _start_cb(self, _req, response):
        self._mission_active = True
        self._mission_start  = datetime.now()
        self._fire_events    = []
        self._plant_events   = []
        self._obstacle_count = 0
        self._gps_track      = []
        self._total_distance = 0.0
        self._nav_states     = []
        response.success = True
        response.message = f'Mission started {self._mission_start.isoformat()}'
        self.get_logger().info('Mission logging started.')
        return response

    def _end_cb(self, _req, response):
        self._mission_active = False
        self._mission_end    = datetime.now()
        path = self._generate_report()
        response.success = True
        response.message = f'Mission ended. Report: {path}'
        return response

    def _gen_cb(self, _req, response):
        path = self._generate_report()
        response.success = True
        response.message = path
        return response

    # ── Report generation ────────────────────────────────────────────────────
    def _generate_report(self) -> str:
        ts    = datetime.now().strftime('%Y%m%d_%H%M%S')
        fname = f'GHRS_Mission_Report_{ts}'
        html_path = os.path.join(self._report_dir, fname + '.html')
        json_path = os.path.join(self._report_dir, fname + '.json')

        start_s = self._mission_start.isoformat() if self._mission_start else 'N/A'
        end_s   = (self._mission_end or datetime.now()).isoformat()
        if self._mission_start:
            dur_s = str((self._mission_end or datetime.now()) - self._mission_start).split('.')[0]
        else:
            dur_s = 'N/A'

        # ── JSON data ────────────────────────────────────────────────────
        data = {
            'rover_id':        self._rover_id,
            'operator':        self._operator,
            'mission_start':   start_s,
            'mission_end':     end_s,
            'duration':        dur_s,
            'total_distance_m': round(self._total_distance, 1),
            'fire_events':     self._fire_events,
            'plant_events':    self._plant_events,
            'obstacle_count':  self._obstacle_count,
            'gps_track_count': len(self._gps_track),
        }
        with open(json_path, 'w') as f:
            json.dump(data, f, indent=2)

        # ── HTML report ──────────────────────────────────────────────────
        fire_rows = ''
        for i, ev in enumerate(self._fire_events, 1):
            gps_str = f"{ev['gps'][0]:.6f}, {ev['gps'][1]:.6f}" if ev['gps'] else 'N/A'
            fire_rows += f"""
            <tr>
              <td>{i}</td>
              <td>{ev['time']}</td>
              <td>{ev['confidence']*100:.0f}%</td>
              <td>{ev['pixel_ratio']*100:.2f}%</td>
              <td>{gps_str}</td>
            </tr>"""

        plant_rows = ''
        for i, ev in enumerate(self._plant_events, 1):
            gps_str = f"{ev['gps'][0]:.6f}, {ev['gps'][1]:.6f}" if ev['gps'] else 'N/A'
            sev_bar = int(ev['severity'] * 100)
            plant_rows += f"""
            <tr>
              <td>{i}</td>
              <td>{ev['time']}</td>
              <td><b>{ev['label']}</b></td>
              <td>{ev['confidence']*100:.0f}%</td>
              <td>
                <div class="sev-bar">
                  <div class="sev-fill" style="width:{sev_bar}%;background:{'#e74c3c' if sev_bar>60 else '#f39c12'}"></div>
                </div>
                {sev_bar}%
              </td>
              <td style="font-size:0.8em">{ev['recs']}</td>
              <td>{gps_str}</td>
            </tr>"""

        html = f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<title>GHRS Mission Report – {ts}</title>
<style>
  * {{ box-sizing: border-box; margin: 0; padding: 0; }}
  body {{ font-family: 'Segoe UI', Arial, sans-serif; background: #0d1117; color: #c9d1d9; padding: 2rem; }}
  h1 {{ color: #58a6ff; font-size: 2rem; margin-bottom: 0.25rem; }}
  h2 {{ color: #3fb950; font-size: 1.2rem; margin: 1.5rem 0 0.5rem; border-bottom: 1px solid #30363d; padding-bottom: 0.3rem; }}
  .badge {{ display:inline-block; padding:.2rem .6rem; border-radius:4px; font-size:.8rem; font-weight:bold; margin-left:.5rem; }}
  .header-info {{ display:flex; gap:2rem; flex-wrap:wrap; margin:1rem 0; background:#161b22; padding:1rem; border-radius:8px; border:1px solid #30363d; }}
  .stat {{ text-align:center; }}
  .stat .val {{ font-size:1.8rem; font-weight:bold; color:#58a6ff; }}
  .stat .lbl {{ font-size:.75rem; color:#8b949e; text-transform:uppercase; }}
  table {{ width:100%; border-collapse:collapse; margin-top:.5rem; font-size:.85rem; }}
  th {{ background:#161b22; color:#8b949e; text-align:left; padding:.5rem .75rem; font-weight:600; }}
  td {{ padding:.45rem .75rem; border-bottom:1px solid #21262d; }}
  tr:hover td {{ background:#161b22; }}
  .sev-bar {{ display:inline-block; width:80px; height:8px; background:#21262d; border-radius:4px; vertical-align:middle; margin-right:.4rem; }}
  .sev-fill {{ height:100%; border-radius:4px; }}
  .fire-tag {{ background:#e74c3c22; color:#e74c3c; border:1px solid #e74c3c44; }}
  .ok-tag   {{ background:#3fb95022; color:#3fb950; border:1px solid #3fb95044; }}
  footer {{ margin-top:2rem; color:#8b949e; font-size:.8rem; border-top:1px solid #21262d; padding-top:1rem; }}
</style>
</head>
<body>
<h1>🤖 GHRS Green House Rover System
  <span class="badge {'fire-tag' if self._fire_events else 'ok-tag'}">
    {'⚠ FIRE EVENTS' if self._fire_events else '✓ ALL CLEAR'}
  </span>
</h1>
<p style="color:#8b949e">Mission Report · Generated {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}</p>

<div class="header-info">
  <div class="stat"><div class="val">{self._rover_id}</div><div class="lbl">Rover ID</div></div>
  <div class="stat"><div class="val">{start_s[:10]}</div><div class="lbl">Date</div></div>
  <div class="stat"><div class="val">{dur_s}</div><div class="lbl">Duration</div></div>
  <div class="stat"><div class="val">{self._total_distance:.0f} m</div><div class="lbl">Distance</div></div>
  <div class="stat"><div class="val">{len(self._fire_events)}</div><div class="lbl">Fire Events</div></div>
  <div class="stat"><div class="val">{len(self._plant_events)}</div><div class="lbl">Disease Detections</div></div>
  <div class="stat"><div class="val">{self._obstacle_count}</div><div class="lbl">Obstacles</div></div>
  <div class="stat"><div class="val">{self._operator}</div><div class="lbl">Operator</div></div>
</div>

<h2>🔥 Fire Events</h2>
{'<p style="color:#8b949e;font-style:italic">No fire events recorded during this mission.</p>' if not self._fire_events else f'''
<table>
  <tr><th>#</th><th>Timestamp</th><th>Confidence</th><th>Coverage</th><th>GPS Location</th></tr>
  {fire_rows}
</table>'''}

<h2>🌿 Plant Health Detections</h2>
{'<p style="color:#8b949e;font-style:italic">No disease detections recorded.</p>' if not self._plant_events else f'''
<table>
  <tr><th>#</th><th>Timestamp</th><th>Disease</th><th>Confidence</th><th>Severity</th><th>Recommendations</th><th>GPS Location</th></tr>
  {plant_rows}
</table>'''}

<h2>📊 Mission Summary</h2>
<table>
  <tr><th>Parameter</th><th>Value</th></tr>
  <tr><td>Rover ID</td><td>{self._rover_id}</td></tr>
  <tr><td>Operator</td><td>{self._operator}</td></tr>
  <tr><td>Mission Start</td><td>{start_s}</td></tr>
  <tr><td>Mission End</td><td>{end_s}</td></tr>
  <tr><td>Total Duration</td><td>{dur_s}</td></tr>
  <tr><td>Distance Covered</td><td>{self._total_distance:.1f} m</td></tr>
  <tr><td>GPS Track Points</td><td>{len(self._gps_track)}</td></tr>
  <tr><td>Fire Events</td><td>{len(self._fire_events)}</td></tr>
  <tr><td>Plant Disease Events</td><td>{len(self._plant_events)}</td></tr>
  <tr><td>Obstacle Encounters</td><td>{self._obstacle_count}</td></tr>
</table>

<footer>
  GHRS Green House Rover System · Amman, Jordan<br>
  Developer: Anmar Arafat Al-Momani<br>
  Report file: {html_path}
</footer>
</body>
</html>
"""
        with open(html_path, 'w') as f:
            f.write(html)

        self.get_logger().info(f'Report generated: {html_path}')

        # Notify
        s = String(); s.data = html_path
        self._ready_pub.publish(s)
        return html_path


def main(args=None):
    rclpy.init(args=args)
    node = ReportNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
