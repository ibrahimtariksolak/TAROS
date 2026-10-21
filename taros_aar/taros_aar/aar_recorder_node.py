"""Muharebe sonrasi analiz kayit dugumu."""

import json
import os
import time

import rclpy
from rclpy.node import Node

from taros_msgs.msg import (
    EngagementRecord,
    TrackArray,
    WeatherCondition,
)


class AARRecorderNode(Node):
    """Tum ana konulari kaydeden AAR dugumu."""

    def __init__(self):
        """AAR kayit dugumunu baslatir."""
        super().__init__('aar_recorder')

        self.declare_parameter('output_dir', '/tmp/taros_aar')
        self.declare_parameter('flush_interval', 5.0)

        output_dir = self.get_parameter('output_dir').value
        os.makedirs(output_dir, exist_ok=True)

        timestamp = time.strftime('%Y%m%d_%H%M%S')
        self._filepath = os.path.join(
            output_dir, f'aar_{timestamp}.jsonl')
        self._buffer = []

        self._track_sub = self.create_subscription(
            TrackArray, '/taros/tracker/tracks',
            self._on_tracks, 10)

        self._engagement_sub = self.create_subscription(
            EngagementRecord, '/taros/engagement/result',
            self._on_engagement, 10)

        self._weather_sub = self.create_subscription(
            WeatherCondition, '/taros/environment/weather',
            self._on_weather, 10)

        flush_interval = self.get_parameter('flush_interval').value
        self._timer = self.create_timer(flush_interval, self._flush)

        self.get_logger().info(
            f'AAR kayit baslatildi: {self._filepath}')

    def _stamp_to_float(self, stamp):
        """ROS zaman damgasini float saniyeye cevirir."""
        return stamp.sec + stamp.nanosec * 1e-9

    def _record(self, topic, data):
        """Kayit tamponuna veri ekler."""
        entry = {
            'timestamp': self._stamp_to_float(
                self.get_clock().now().to_msg()),
            'topic': topic,
            'data': data,
        }
        self._buffer.append(entry)

    def _on_tracks(self, msg):
        """Iz listesini kaydeder."""
        tracks = []
        for t in msg.tracks:
            tracks.append({
                'track_id': t.track_id,
                'status': t.status,
                'classification': t.classification,
                'position': [t.position.x, t.position.y, t.position.z],
                'speed': t.speed,
                'threat_score': t.threat_score,
                'confidence': t.confidence,
            })
        self._record('tracks', {'count': len(tracks), 'tracks': tracks})

    def _on_engagement(self, msg):
        """Angazman sonucunu kaydeder."""
        self._record('engagement', {
            'engagement_id': msg.engagement_id,
            'track_id': msg.track_id,
            'unit_id': msg.unit_id,
            'effector_type': msg.effector_type,
            'result': msg.result,
            'sensor_to_shoot_time': msg.sensor_to_shoot_time,
            'miss_distance': msg.miss_distance,
        })

    def _on_weather(self, msg):
        """Hava durumunu kaydeder."""
        self._record('weather', {
            'visibility': msg.visibility,
            'rain_rate': msg.rain_rate,
            'fog_density': msg.fog_density,
            'wind': [msg.wind.x, msg.wind.y, msg.wind.z],
            'is_night': msg.is_night,
            'temperature': msg.temperature,
        })

    def _flush(self):
        """Tamponu dosyaya yazar."""
        if not self._buffer:
            return
        try:
            with open(self._filepath, 'a') as f:
                for entry in self._buffer:
                    f.write(json.dumps(entry) + '\n')
            count = len(self._buffer)
            self._buffer.clear()
            self.get_logger().debug(f'{count} kayit yazildi')
        except OSError as e:
            self.get_logger().error(f'Dosya yazma hatasi: {e}')


def main(args=None):
    """AAR kayit dugumunu calistirir."""
    rclpy.init(args=args)
    node = AARRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
