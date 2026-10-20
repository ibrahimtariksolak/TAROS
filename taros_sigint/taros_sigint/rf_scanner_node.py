"""
RF sinyal tarayici simulatoru.

Her drone'un kontrol baglanti sinyalini simule eder. Drone'a yakin
noktadan yayimlanan RF sinyali algilayip RFSignal mesaji olusturur.
"""

import math
import random

import rclpy
from rclpy.node import Node

from taros_msgs.msg import RFSignal, TrackArray

PROTOCOL_MAP = {
    'COMMERCIAL_QUAD': ('DJI_OCCUSYNC', 2400.0, 40.0, 'OFDM'),
    'COMMERCIAL_FIXEDWING': ('ELRS', 915.0, 1.0, 'FHSS'),
    'MILITARY': ('UNKNOWN', 5800.0, 80.0, 'DSSS'),
    'UNKNOWN': ('UNKNOWN', 2400.0, 20.0, 'UNKNOWN'),
}


class RFScannerNode(Node):
    """RF sinyal tarayici."""

    def __init__(self):
        """Tarayici parametrelerini yukler."""
        super().__init__('rf_scanner')

        self.declare_parameter('update_rate', 2.0)
        self.declare_parameter('sensitivity_dbm', -90.0)
        self.declare_parameter('scanner_x', 0.0)
        self.declare_parameter('scanner_y', 0.0)
        self.declare_parameter('scanner_z', 5.0)

        self._rate = self.get_parameter('update_rate').value
        self._sensitivity = self.get_parameter('sensitivity_dbm').value
        self._scanner_pos = (
            self.get_parameter('scanner_x').value,
            self.get_parameter('scanner_y').value,
            self.get_parameter('scanner_z').value,
        )

        self._latest_tracks = None

        self._track_sub = self.create_subscription(
            TrackArray, 'taros/ground_truth/tracks',
            self._on_tracks, 10)

        self._pub = self.create_publisher(
            RFSignal, 'taros/sigint/rf_signals', 10)

        self._timer = self.create_timer(1.0 / self._rate, self._scan)

        self.get_logger().info(
            f'RF tarayici baslatildi: hassasiyet={self._sensitivity}dBm')

    def _on_tracks(self, msg):
        """Ground truth izlerini alir."""
        self._latest_tracks = msg

    def _scan(self):
        """Her hedef icin RF sinyali uretir."""
        if self._latest_tracks is None:
            return

        for track in self._latest_tracks.tracks:
            signal = self._generate_signal(track)
            if signal is not None:
                self._pub.publish(signal)

    def _generate_signal(self, track):
        """Tek hedef icin RF sinyal simulasyonu."""
        target_pos = (
            track.position.x, track.position.y, track.position.z)
        dx = target_pos[0] - self._scanner_pos[0]
        dy = target_pos[1] - self._scanner_pos[1]
        dz = target_pos[2] - self._scanner_pos[2]
        rng = math.sqrt(dx * dx + dy * dy + dz * dz)

        if rng < 1.0:
            rng = 1.0

        proto, freq, bw, mod = PROTOCOL_MAP.get(
            track.classification, PROTOCOL_MAP['UNKNOWN'])

        tx_power_dbm = 20.0
        path_loss = 20.0 * math.log10(rng) + 20.0 * math.log10(freq) - 27.55
        received_dbm = tx_power_dbm - path_loss + random.gauss(0, 3.0)

        if received_dbm < self._sensitivity:
            return None

        bearing = math.atan2(dy, dx) + random.gauss(0, math.radians(2.0))

        msg = RFSignal()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'rf_scanner'
        msg.frequency = freq + random.gauss(0, 0.5)
        msg.bandwidth = bw + random.gauss(0, 0.5)
        msg.modulation_type = mod
        msg.signal_strength = received_dbm
        msg.bearing = bearing
        msg.matched_protocol = proto
        return msg


def main(args=None):
    """RF tarayici dugumunu calistirir."""
    rclpy.init(args=args)
    node = RFScannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
