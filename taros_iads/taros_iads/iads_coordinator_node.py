"""Entegre hava savunma sistemi koordinasyon dugumu."""

import rclpy
from rclpy.node import Node

from taros_msgs.msg import (
    EngagementCommand,
    TrackArray,
    WeaponTargetAssignment,
)

from taros_iads.wta_optimizer import DefenseUnit, WTAOptimizer


class IADSCoordinatorNode(Node):
    """Coklu savunma birligi koordinasyon dugumu."""

    def __init__(self):
        """IADS koordinator dugumunu baslatir."""
        super().__init__('iads_coordinator')

        self.declare_parameter('assignment_rate', 2.0)
        self.declare_parameter('threat_score_threshold', 30.0)
        self.declare_parameter('unit_configs', [
            'alpha', 0.0, 0.0, 3000.0,
            'bravo', 2000.0, 0.0, 3000.0,
            'charlie', 1000.0, 1500.0, 3000.0,
        ])

        self._threshold = self.get_parameter(
            'threat_score_threshold').value

        self._units = self._parse_units()
        self._optimizer = WTAOptimizer(self._units)
        self._latest_tracks = None

        self._track_sub = self.create_subscription(
            TrackArray, '/taros/tracker/tracks',
            self._on_tracks, 10)

        self._wta_pub = self.create_publisher(
            WeaponTargetAssignment, '/taros/iads/assignments', 10)

        rate = self.get_parameter('assignment_rate').value
        self._timer = self.create_timer(1.0 / rate, self._assign)

        self.get_logger().info(
            f'IADS koordinator baslatildi: {len(self._units)} birlik')

    def _parse_units(self):
        """Parametre degerlerinden birlik listesini olusturur."""
        params = self.get_parameter('unit_configs').value
        units = []
        for i in range(0, len(params) - 3, 4):
            units.append(DefenseUnit(
                unit_id=str(params[i]),
                x=float(params[i + 1]),
                y=float(params[i + 2]),
                effective_range=float(params[i + 3]),
            ))
        return units

    def _on_tracks(self, msg):
        """Iz listesini kaydeder."""
        self._latest_tracks = msg

    def _assign(self):
        """Periyodik olarak silah-hedef atamasini yapar."""
        if self._latest_tracks is None:
            return

        threats = []
        for track in self._latest_tracks.tracks:
            if track.threat_score >= self._threshold:
                priority = self._score_to_priority(track.threat_score)
                threats.append((
                    track.track_id,
                    track.position.x,
                    track.position.y,
                    priority,
                ))

        if not threats:
            return

        results = self._optimizer.assign(threats)

        msg = WeaponTargetAssignment()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'

        for track_id, unit_id, eff_type, priority, prob in results:
            cmd = EngagementCommand()
            cmd.header.stamp = msg.header.stamp
            cmd.track_id = track_id
            cmd.unit_id = unit_id
            cmd.effector_type = eff_type
            cmd.priority = priority
            cmd.authorization = 0
            msg.assignments.append(cmd)

        msg.total_coverage = self._optimizer.compute_coverage(
            results, len(threats))
        msg.unassigned_threats = len(threats) - len(results)

        self._wta_pub.publish(msg)

        self.get_logger().debug(
            f'Atama: {len(results)}/{len(threats)} tehdit, '
            f'kapsam={msg.total_coverage:.1%}')

    @staticmethod
    def _score_to_priority(score):
        """Tehdit puanini 1-10 oncelik degerine cevirir."""
        if score >= 90:
            return 1
        if score >= 70:
            return 3
        if score >= 50:
            return 5
        return 8


def main(args=None):
    """IADS koordinator dugumunu calistirir."""
    rclpy.init(args=args)
    node = IADSCoordinatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
