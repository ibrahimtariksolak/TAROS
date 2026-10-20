"""
Elektro-optik / kizilotesi kamera simulatoru.

Pan-tilt gimbal uzerinde dar goruslu kamera. Hedefleri FOV icindeyse ve
menzil kosullarini sagliyorsa tespit eder. Bearing-only olcum uretir;
menzil tahmini yalnizca yakin hedefler icin verilir.
"""

import math
import random

import rclpy
from rclpy.node import Node

from taros_msgs.msg import (
    EoirDetection,
    EoirDetectionArray,
    TrackArray,
    WeatherCondition,
)

from taros_sensor.sensor_models import (
    azimuth_elevation,
    distance_3d,
    eoir_max_range,
    eoir_pixel_from_bearing,
)


class EoirCameraNode(Node):
    """EO/IR kamera simulatoru, en yakin hedefe otomatik kilit."""

    def __init__(self):
        """Kamera parametrelerini yukler ve gimbal zamanlayicisini baslatir."""
        super().__init__('eoir_camera')

        self.declare_parameter('update_rate', 10.0)
        self.declare_parameter('h_fov_deg', 30.0)
        self.declare_parameter('v_fov_deg', 22.5)
        self.declare_parameter('max_range', 5000.0)
        self.declare_parameter('min_range', 10.0)
        self.declare_parameter('pan_rate_dps', 45.0)
        self.declare_parameter('camera_x', 0.0)
        self.declare_parameter('camera_y', 0.0)
        self.declare_parameter('camera_z', 8.0)
        self.declare_parameter('az_noise_deg', 0.05)
        self.declare_parameter('el_noise_deg', 0.05)

        self._rate = self.get_parameter('update_rate').value
        self._h_fov = math.radians(self.get_parameter('h_fov_deg').value)
        self._v_fov = math.radians(self.get_parameter('v_fov_deg').value)
        self._base_max_range = self.get_parameter('max_range').value
        self._min_range = self.get_parameter('min_range').value
        self._pan_rate = math.radians(self.get_parameter('pan_rate_dps').value)
        self._cam_pos = (
            self.get_parameter('camera_x').value,
            self.get_parameter('camera_y').value,
            self.get_parameter('camera_z').value,
        )
        self._az_noise = math.radians(self.get_parameter('az_noise_deg').value)
        self._el_noise = math.radians(self.get_parameter('el_noise_deg').value)

        self._cam_az = 0.0
        self._cam_el = math.radians(15.0)
        self._target_az = 0.0
        self._target_el = math.radians(15.0)
        self._detection_id = 0

        self._latest_tracks = None
        self._visibility = 10000.0
        self._rain_rate = 0.0
        self._is_night = False

        self._track_sub = self.create_subscription(
            TrackArray, 'taros/ground_truth/tracks',
            self._on_tracks, 10)

        self._weather_sub = self.create_subscription(
            WeatherCondition, 'taros/environment/weather',
            self._on_weather, 10)

        self._pub = self.create_publisher(
            EoirDetectionArray, 'taros/sensor/eoir/detections', 10)

        self._timer = self.create_timer(1.0 / self._rate, self._update)

        self.get_logger().info(
            f'EO/IR kamera baslatildi: FOV={math.degrees(self._h_fov):.0f}x'
            f'{math.degrees(self._v_fov):.0f} derece, '
            f'menzil={self._base_max_range}m')

    def _on_tracks(self, msg):
        self._latest_tracks = msg

    def _on_weather(self, msg):
        self._visibility = msg.visibility
        self._rain_rate = msg.rain_rate
        self._is_night = msg.is_night

    def _update(self):
        if self._latest_tracks is None:
            return

        self._update_gimbal()

        max_range = eoir_max_range(
            self._base_max_range, self._visibility,
            self._rain_rate, self._is_night)

        detections = []
        for track in self._latest_tracks.tracks:
            det = self._evaluate_target(track, max_range)
            if det is not None:
                detections.append(det)

        if detections:
            msg = EoirDetectionArray()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera'
            msg.detections = detections
            self._pub.publish(msg)

    def _update_gimbal(self):
        """Gimbal'i en yakin hedefe dogru cevir."""
        if not self._latest_tracks or not self._latest_tracks.tracks:
            return

        closest_range = float('inf')
        closest_az = self._cam_az
        closest_el = self._cam_el

        for track in self._latest_tracks.tracks:
            target_pos = (track.position.x, track.position.y, track.position.z)
            rng = distance_3d(self._cam_pos, target_pos)
            if rng < closest_range:
                closest_range = rng
                az, el = azimuth_elevation(self._cam_pos, target_pos)
                closest_az = az
                closest_el = el

        self._target_az = closest_az
        self._target_el = closest_el

        dt = 1.0 / self._rate
        max_step = self._pan_rate * dt

        az_diff = self._wrap(self._target_az - self._cam_az)
        if abs(az_diff) > max_step:
            az_diff = math.copysign(max_step, az_diff)
        self._cam_az = self._wrap(self._cam_az + az_diff)

        el_diff = self._target_el - self._cam_el
        if abs(el_diff) > max_step:
            el_diff = math.copysign(max_step, el_diff)
        self._cam_el = max(math.radians(-30.0),
                           min(math.radians(90.0), self._cam_el + el_diff))

    def _evaluate_target(self, track, max_range):
        """Tek hedef icin EO/IR tespiti."""
        target_pos = (track.position.x, track.position.y, track.position.z)
        rng = distance_3d(self._cam_pos, target_pos)

        if rng < self._min_range or rng > max_range:
            return None

        az, el = azimuth_elevation(self._cam_pos, target_pos)
        pixel = eoir_pixel_from_bearing(
            az, el, self._cam_az, self._cam_el, self._h_fov, self._v_fov)

        if pixel is None:
            return None

        px, py = pixel

        pd = min(0.95, max_range / rng * 0.5) if rng > 0 else 0.95
        if random.random() > pd:
            return None

        self._detection_id += 1
        det = EoirDetection()
        det.header.stamp = self.get_clock().now().to_msg()
        det.header.frame_id = 'camera'
        det.detection_id = self._detection_id
        det.azimuth = az + random.gauss(0, self._az_noise)
        det.elevation = el + random.gauss(0, self._el_noise)
        det.pixel_x = max(0.0, min(1.0, px + random.gauss(0, 0.005)))
        det.pixel_y = max(0.0, min(1.0, py + random.gauss(0, 0.005)))

        apparent_size = 2.0 / max(rng, 1.0)
        det.bbox_width = min(0.3, max(0.01, apparent_size + random.gauss(0, 0.002)))
        det.bbox_height = min(0.3, max(0.01, apparent_size * 0.7 + random.gauss(0, 0.002)))

        if rng < 1000.0:
            det.estimated_range = rng + random.gauss(0, rng * 0.1)
        else:
            det.estimated_range = 0.0

        det.classification = track.classification
        det.confidence = min(0.95, 500.0 / max(rng, 1.0))
        return det

    @staticmethod
    def _wrap(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    """EO/IR kamera simulasyon dugumunu calistirir."""
    rclpy.init(args=args)
    node = EoirCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
