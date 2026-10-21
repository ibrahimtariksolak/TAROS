"""Komuta kontrol arayuz koprusu dugumu."""

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node

from taros_msgs.msg import (
    EngagementRecord,
    SystemStatus,
    TrackArray,
    WeaponTargetAssignment,
)


class C2BridgeNode(Node):
    """Sistem durumunu toplayip tek mesajda yayinlayan kopru dugumu."""

    def __init__(self):
        """C2 kopru dugumunu baslatir."""
        super().__init__('c2_bridge')

        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('unit_id', 'taros_main')

        self._unit_id = self.get_parameter('unit_id').value

        self._active_tracks = 0
        self._confirmed_tracks = 0
        self._active_engagements = 0
        self._interceptor_count = 0
        self._sensor_health = [1.0, 1.0, 1.0]

        self._track_sub = self.create_subscription(
            TrackArray, '/taros/tracker/tracks',
            self._on_tracks, 10)

        self._wta_sub = self.create_subscription(
            WeaponTargetAssignment, '/taros/iads/assignments',
            self._on_assignments, 10)

        self._eng_sub = self.create_subscription(
            EngagementRecord, '/taros/engagement/result',
            self._on_engagement, 10)

        self._status_pub = self.create_publisher(
            SystemStatus, '/taros/c2/system_status', 10)

        rate = self.get_parameter('publish_rate').value
        self._timer = self.create_timer(1.0 / rate, self._publish_status)

        self.get_logger().info(
            f'C2 kopru baslatildi: birlik={self._unit_id}')

    def _on_tracks(self, msg):
        """Iz istatistiklerini gunceller."""
        self._active_tracks = len(msg.tracks)
        self._confirmed_tracks = sum(
            1 for t in msg.tracks if t.status == 1)

    def _on_assignments(self, msg):
        """Atama sayisini gunceller."""
        self._active_engagements = len(msg.assignments)

    def _on_engagement(self, msg):
        """Angazman sonucunu isler."""
        if msg.result in (1, 2, 3):
            self._active_engagements = max(
                0, self._active_engagements - 1)

    def _compute_readiness(self):
        """Hazirlik seviyesini hesaplar."""
        if self._active_engagements > 0:
            return 3
        if self._active_tracks > 0:
            return 2
        return 1

    def _publish_status(self):
        """Sistem durumunu periyodik olarak yayinlar."""
        msg = SystemStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.unit_id = self._unit_id
        msg.operational = True
        msg.readiness_level = self._compute_readiness()
        msg.interceptor_count = self._interceptor_count
        msg.sensor_health = self._sensor_health
        msg.position = Point(x=0.0, y=0.0, z=0.0)
        msg.effective_range = 5000.0

        self._status_pub.publish(msg)


def main(args=None):
    """C2 kopru dugumunu calistirir."""
    rclpy.init(args=args)
    node = C2BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
