"""
Akustik sensor dizisi simulatoru.

4 elemanli mikrofon dizisi ile gelis yonu (DOA) kestirimi, frekans spektrumu
analizi ve akustik imza eslestirmesi yapar. Kisa menzilli, pasif sensor.
"""

import math
import random

import rclpy
from rclpy.node import Node

from taros_msgs.msg import (
    AcousticDetection,
    AcousticDetectionArray,
    TrackArray,
    WeatherCondition,
)

from taros_sensor.sensor_models import (
    DOMINANT_FREQ_HZ,
    SOURCE_SPL_DB,
    acoustic_doa_noise,
    acoustic_snr,
    azimuth_elevation,
    db_to_linear,
    distance_3d,
    generate_frequency_spectrum,
)

CLASSIFICATION_TO_SIGNATURE = {
    'COMMERCIAL_QUAD': 'DJI_MAVIC_3',
    'COMMERCIAL_FIXEDWING': 'FIXED_WING_GENERIC',
    'MILITARY': 'MILITARY_HEX',
    'UNKNOWN': 'UNKNOWN',
}

ACOUSTIC_DETECTION_THRESHOLD_DB = 6.0


class AcousticSimNode(Node):
    """Akustik sensor dizisi simulatoru."""

    def __init__(self):
        """Akustik dizi parametrelerini yukler ve tarama zamanlayicisini baslatir."""
        super().__init__('acoustic_sim')

        self.declare_parameter('update_rate', 5.0)
        self.declare_parameter('max_range', 800.0)
        self.declare_parameter('baseline', 1.0)
        self.declare_parameter('background_noise_db', 40.0)
        self.declare_parameter('sensor_x', 5.0)
        self.declare_parameter('sensor_y', 0.0)
        self.declare_parameter('sensor_z', 2.0)

        self._rate = self.get_parameter('update_rate').value
        self._max_range = self.get_parameter('max_range').value
        self._baseline = self.get_parameter('baseline').value
        self._bg_noise = self.get_parameter('background_noise_db').value
        self._sensor_pos = (
            self.get_parameter('sensor_x').value,
            self.get_parameter('sensor_y').value,
            self.get_parameter('sensor_z').value,
        )

        self._detection_id = 0
        self._wind_speed = 0.0
        self._rain_rate = 0.0

        self._latest_tracks = None

        self._track_sub = self.create_subscription(
            TrackArray, 'taros/ground_truth/tracks',
            self._on_tracks, 10)

        self._weather_sub = self.create_subscription(
            WeatherCondition, 'taros/environment/weather',
            self._on_weather, 10)

        self._pub = self.create_publisher(
            AcousticDetectionArray, 'taros/sensor/acoustic/detections', 10)

        self._timer = self.create_timer(1.0 / self._rate, self._update)

        self.get_logger().info(
            f'Akustik sensor baslatildi: menzil={self._max_range}m, '
            f'frekans={self._rate}Hz')

    def _on_tracks(self, msg):
        self._latest_tracks = msg

    def _on_weather(self, msg):
        self._wind_speed = math.sqrt(
            msg.wind.x ** 2 + msg.wind.y ** 2 + msg.wind.z ** 2)
        self._rain_rate = msg.rain_rate

    def _update(self):
        if self._latest_tracks is None:
            return

        rain_noise = 5.0 * math.log10(1.0 + self._rain_rate) if self._rain_rate > 0 else 0.0
        effective_bg = self._bg_noise + rain_noise

        detections = []
        for track in self._latest_tracks.tracks:
            det = self._evaluate_target(track, effective_bg)
            if det is not None:
                detections.append(det)

        if detections:
            msg = AcousticDetectionArray()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'acoustic_array'
            msg.detections = detections
            self._pub.publish(msg)

    def _evaluate_target(self, track, background_db):
        """Tek hedef icin akustik tespit."""
        target_pos = (track.position.x, track.position.y, track.position.z)
        rng = distance_3d(self._sensor_pos, target_pos)

        if rng > self._max_range or rng < 1.0:
            return None

        source_spl = SOURCE_SPL_DB.get(track.classification, 72.0)
        snr_db = acoustic_snr(rng, source_spl, background_db, self._wind_speed)

        if snr_db < ACOUSTIC_DETECTION_THRESHOLD_DB:
            return None

        pd = min(0.98, 1.0 / (1.0 + math.exp(-(snr_db - ACOUSTIC_DETECTION_THRESHOLD_DB))))
        if random.random() > pd:
            return None

        az, el = azimuth_elevation(self._sensor_pos, target_pos)
        snr_lin = db_to_linear(snr_db)
        dominant_freq = DOMINANT_FREQ_HZ.get(track.classification, 200.0)
        doa_sigma = acoustic_doa_noise(snr_lin, self._baseline, dominant_freq)

        spectrum = generate_frequency_spectrum(track.classification, snr_db)
        signature = CLASSIFICATION_TO_SIGNATURE.get(track.classification, 'UNKNOWN')
        match_conf = min(0.95, snr_db / 30.0)
        if random.random() > match_conf:
            signature = 'UNKNOWN'

        self._detection_id += 1
        det = AcousticDetection()
        det.header.stamp = self.get_clock().now().to_msg()
        det.header.frame_id = 'acoustic_array'
        det.doa_azimuth = az + random.gauss(0, doa_sigma)
        det.doa_elevation = el + random.gauss(0, doa_sigma * 1.5)
        det.frequency_spectrum = [float(v) for v in spectrum]
        det.dominant_frequency = dominant_freq + random.gauss(0, 5.0)
        det.confidence = min(1.0, max(0.0, snr_db / 25.0 + random.gauss(0, 0.05)))
        det.signature_match = signature
        return det


def main(args=None):
    """Akustik sensor simulasyon dugumunu calistirir."""
    rclpy.init(args=args)
    node = AcousticSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
