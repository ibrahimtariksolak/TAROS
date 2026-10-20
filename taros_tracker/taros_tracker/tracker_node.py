"""
Coklu sensor fuzyonu ve coklu hedef takip dugumu.

Uc sensorun (radar, EO/IR, akustik) tespitlerini alir, EKF ile fuse eder
ve birlesmis iz listesini yayinlar. Her sensor olcumu geldiginde bagimsiz
olarak iliskilendirme ve guncelleme yapar (asenkron fuzyon).
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Vector3
from rclpy.node import Node

from taros_msgs.msg import (
    AcousticDetectionArray,
    EoirDetectionArray,
    RadarReturnArray,
    Track,
    TrackArray,
)

from taros_tracker.data_association import associate
from taros_tracker.track_manager import TrackManager


class TrackerNode(Node):
    """Coklu sensor fuzyonu ve coklu hedef takip dugumu."""

    def __init__(self):
        """Tracker dugumunu baslatir."""
        super().__init__('tracker')

        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('gate_threshold', 16.0)

        self._rate = self.get_parameter('publish_rate').value
        self._gate = self.get_parameter('gate_threshold').value
        self._manager = TrackManager()
        self._last_predict_time = None

        self._radar_sub = self.create_subscription(
            RadarReturnArray, 'taros/sensor/radar/returns',
            self._on_radar, 10)

        self._eoir_sub = self.create_subscription(
            EoirDetectionArray, 'taros/sensor/eoir/detections',
            self._on_eoir, 10)

        self._acoustic_sub = self.create_subscription(
            AcousticDetectionArray, 'taros/sensor/acoustic/detections',
            self._on_acoustic, 10)

        self._pub = self.create_publisher(
            TrackArray, 'taros/tracker/tracks', 10)

        self._timer = self.create_timer(1.0 / self._rate, self._publish_tracks)

        self.get_logger().info(
            f'Tracker baslatildi: yayim={self._rate}Hz, '
            f'kapi_esigi={self._gate}')

    def _current_time(self):
        """ROS saatinden saniye dondurur."""
        return self.get_clock().now().nanoseconds / 1e9

    def _predict_step(self):
        """Zaman farki kadar tum izleri ileri tasir."""
        now = self._current_time()
        if self._last_predict_time is not None:
            dt = now - self._last_predict_time
            if 0 < dt < 2.0:
                self._manager.predict_all(dt)
        self._last_predict_time = now

    def _on_radar(self, msg):
        """Radar tespitlerini isle."""
        self._predict_step()
        now = self._current_time()

        measurements = []
        classifications = []
        positions = []

        for ret in msg.returns:
            meas = np.array([ret.range, ret.azimuth, ret.elevation, ret.doppler_velocity])
            measurements.append(meas)
            classifications.append('')
            positions.append((
                ret.position_enu.x,
                ret.position_enu.y,
                ret.position_enu.z,
            ))

        self._process_detections(
            measurements, classifications, positions,
            'radar', now)

    def _on_eoir(self, msg):
        """EO/IR tespitlerini isle."""
        self._predict_step()
        now = self._current_time()

        measurements = []
        classifications = []
        positions = []

        for det in msg.detections:
            meas = np.array([det.azimuth, det.elevation])
            measurements.append(meas)
            classifications.append(det.classification)

            if det.estimated_range > 0:
                cos_el = math.cos(det.elevation)
                x = det.estimated_range * cos_el * math.cos(det.azimuth)
                y = det.estimated_range * cos_el * math.sin(det.azimuth)
                z = det.estimated_range * math.sin(det.elevation)
                positions.append((x, y, z))
            else:
                est_range = 2000.0
                cos_el = math.cos(det.elevation)
                x = est_range * cos_el * math.cos(det.azimuth)
                y = est_range * cos_el * math.sin(det.azimuth)
                z = est_range * math.sin(det.elevation)
                positions.append((x, y, z))

        self._process_detections(
            measurements, classifications, positions,
            'eoir', now)

    def _on_acoustic(self, msg):
        """Akustik tespitlerini isle."""
        self._predict_step()
        now = self._current_time()

        measurements = []
        classifications = []
        positions = []

        for det in msg.detections:
            meas = np.array([det.doa_azimuth, det.doa_elevation])
            measurements.append(meas)

            cls = ''
            if det.signature_match and det.signature_match != 'UNKNOWN':
                sig_to_cls = {
                    'DJI_MAVIC_3': 'COMMERCIAL_QUAD',
                    'FIXED_WING_GENERIC': 'COMMERCIAL_FIXEDWING',
                    'MILITARY_HEX': 'MILITARY',
                }
                cls = sig_to_cls.get(det.signature_match, '')
            classifications.append(cls)

            est_range = 400.0
            cos_el = math.cos(det.doa_elevation)
            x = est_range * cos_el * math.cos(det.doa_azimuth)
            y = est_range * cos_el * math.sin(det.doa_azimuth)
            z = est_range * math.sin(det.doa_elevation)
            positions.append((x, y, z))

        self._process_detections(
            measurements, classifications, positions,
            'acoustic', now)

    def _process_detections(self, measurements, classifications, positions,
                            sensor_type, timestamp):
        """Iliskilendirme, guncelleme ve yeni iz olusturma."""
        if not measurements:
            return

        ekfs, tracks = self._manager.get_ekf_list()
        matches, unmatched_t, unmatched_m = associate(
            ekfs, measurements, sensor_type, self._gate)

        for t_idx, m_idx in matches:
            self._manager.update_matched(
                tracks[t_idx], measurements[m_idx],
                sensor_type, classifications[m_idx], timestamp)

        self._manager.mark_unmatched([tracks[i] for i in unmatched_t])

        for m_idx in unmatched_m:
            pos = positions[m_idx]
            initial_state = [pos[0], pos[1], pos[2], 0.0, 0.0, 0.0]
            track = self._manager.initiate_track(initial_state, timestamp)
            if classifications[m_idx]:
                track.classification = classifications[m_idx]

    def _publish_tracks(self):
        """Aktif izleri periyodik olarak yayinlar."""
        now = self._current_time()
        self._manager.maintain(now)

        active = self._manager.active_tracks
        if not active:
            return

        msg = TrackArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'

        for track in active:
            t = Track()
            t.track_id = track.track_id
            t.status = track.status

            pos = track.ekf.position
            vel = track.ekf.velocity

            t.position = Point(x=float(pos[0]), y=float(pos[1]), z=float(pos[2]))
            t.velocity = Vector3(x=float(vel[0]), y=float(vel[1]), z=float(vel[2]))
            t.speed = float(np.linalg.norm(vel))
            t.heading = float(math.atan2(vel[1], vel[0])) if t.speed > 0.1 else 0.0
            t.altitude = float(pos[2])
            t.range_to_base = float(math.hypot(pos[0], pos[1]))
            t.time_to_arrival = (
                t.range_to_base / t.speed if t.speed > 0.1 else 0.0)

            t.classification = track.classification
            t.confidence = track.confidence
            t.sensor_contributions = track.sensor_contributions
            t.update_count = track.total_updates

            ros_time = self.get_clock().now().to_msg()
            t.last_update = ros_time

            msg.tracks.append(t)

        self._pub.publish(msg)


def main(args=None):
    """Tracker dugumunu calistirir."""
    rclpy.init(args=args)
    node = TrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
