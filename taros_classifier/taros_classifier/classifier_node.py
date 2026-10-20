"""
Hedef siniflandirma ve tehdit degerlendirme dugumu.

Tracker'dan gelen izleri alir, niyet analizi ve tehdit skorlamasi yapar,
yorunge kestirimiyle en yakin yaklasma hesaplar.
"""

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node

from taros_msgs.msg import ThreatAssessment, TrackArray

from taros_classifier.intent_analyzer import classify_intent, compute_features
from taros_classifier.threat_scorer import compute_threat_score, threat_level
from taros_classifier.trajectory_predictor import closest_approach, predicted_impact


class ClassifierNode(Node):
    """Hedef siniflandirma ve tehdit degerlendirme dugumu."""

    def __init__(self):
        """Siniflandirma dugumunu baslatir."""
        super().__init__('classifier')

        self.declare_parameter('publish_rate', 5.0)
        self._rate = self.get_parameter('publish_rate').value

        self._prev_headings = {}
        self._prev_altitudes = {}
        self._latest_tracks = None

        self._track_sub = self.create_subscription(
            TrackArray, 'taros/tracker/tracks',
            self._on_tracks, 10)

        self._pub = self.create_publisher(
            ThreatAssessment, 'taros/classifier/threats', 10)

        self._timer = self.create_timer(1.0 / self._rate, self._evaluate)

        self.get_logger().info(
            f'Siniflandirma baslatildi: frekans={self._rate}Hz')

    def _on_tracks(self, msg):
        """Tracker izlerini alir."""
        self._latest_tracks = msg

    def _evaluate(self):
        """Her iz icin tehdit degerlendirmesi yapar."""
        if self._latest_tracks is None:
            return

        dt = 1.0 / self._rate

        for track in self._latest_tracks.tracks:
            tid = track.track_id

            features = compute_features(
                track,
                prev_heading=self._prev_headings.get(tid),
                prev_altitude=self._prev_altitudes.get(tid),
                dt=dt,
            )

            intent = classify_intent(features)

            score = compute_threat_score(
                features, intent, track.classification)

            pos = (track.position.x, track.position.y, track.position.z)
            vel = (track.velocity.x, track.velocity.y, track.velocity.z)
            ca_dist, ca_time = closest_approach(pos, vel)
            impact = predicted_impact(pos, vel)

            msg = ThreatAssessment()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'world'
            msg.track_id = tid
            msg.threat_score = score
            msg.threat_level = threat_level(score)
            msg.intent = intent
            msg.drone_type = track.classification
            msg.time_to_arrival = track.time_to_arrival
            msg.closest_approach = ca_dist
            msg.predicted_impact = Point(
                x=impact[0], y=impact[1], z=impact[2])

            self._pub.publish(msg)

            self._prev_headings[tid] = track.heading
            self._prev_altitudes[tid] = track.altitude

        active_ids = {t.track_id for t in self._latest_tracks.tracks}
        stale = [k for k in self._prev_headings if k not in active_ids]
        for k in stale:
            self._prev_headings.pop(k, None)
            self._prev_altitudes.pop(k, None)


def main(args=None):
    """Siniflandirma dugumunu calistirir."""
    rclpy.init(args=args)
    node = ClassifierNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
