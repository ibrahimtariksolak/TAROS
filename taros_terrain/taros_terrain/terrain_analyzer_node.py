"""Arazi analizi ve radar gorus hatti degerlendirme dugumu."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from taros_msgs.msg import TrackArray

from taros_terrain.terrain_models import (
    ElevationGrid,
    check_line_of_sight,
    compute_viewshed,
)


class TerrainAnalyzerNode(Node):
    """Arazi analizi ve radar gorus hatti degerlendirme dugumu."""

    def __init__(self):
        """Arazi analiz dugumunu baslatir."""
        super().__init__('terrain_analyzer')

        self.declare_parameter('radar_x', 500.0)
        self.declare_parameter('radar_y', 500.0)
        self.declare_parameter('radar_z', 15.0)
        self.declare_parameter('max_range', 5000.0)
        self.declare_parameter(
            'obstacle_positions', [250.0, 250.0, 50.0, 30.0])
        self.declare_parameter('viewshed_rate', 1.0)

        rx = self.get_parameter('radar_x').value
        ry = self.get_parameter('radar_y').value
        rz = self.get_parameter('radar_z').value
        self._radar_pos = (rx, ry, rz)
        self._max_range = self.get_parameter('max_range').value

        self._grid = ElevationGrid(
            width=self._max_range * 2,
            height=self._max_range * 2,
            resolution=10.0,
        )

        obs_params = self.get_parameter('obstacle_positions').value
        for i in range(0, len(obs_params) - 3, 4):
            self._grid.add_obstacle(
                obs_params[i], obs_params[i + 1],
                obs_params[i + 2], obs_params[i + 3],
            )

        self._track_sub = self.create_subscription(
            TrackArray, '/taros/tracker/tracks',
            self._on_tracks, 10)

        self._los_pub = self.create_publisher(
            Float64MultiArray, '/taros/terrain/los_status', 10)

        self._blind_pub = self.create_publisher(
            Float64MultiArray, '/taros/terrain/blind_zones', 10)

        rate = self.get_parameter('viewshed_rate').value
        self._timer = self.create_timer(1.0 / rate, self._publish_viewshed)

        self.get_logger().info(
            f'Arazi analizci baslatildi: radar=({rx},{ry},{rz})')

    def _on_tracks(self, msg):
        """Iz listesini alir ve her iz icin LOS durumunu yayinlar."""
        los_msg = Float64MultiArray()
        for track in msg.tracks:
            target = (
                track.position.x,
                track.position.y,
                track.position.z,
            )
            los = check_line_of_sight(self._grid, self._radar_pos, target)
            los_msg.data.append(float(track.track_id))
            los_msg.data.append(1.0 if los else 0.0)
        self._los_pub.publish(los_msg)

    def _publish_viewshed(self):
        """Periyodik olarak kor alan haritasini yayinlar."""
        blind = compute_viewshed(
            self._grid, self._radar_pos, self._max_range)
        msg = Float64MultiArray()
        msg.data = [float(a) for a in blind]
        self._blind_pub.publish(msg)

        if blind:
            coverage = 1.0 - len(blind) / 36.0
            self.get_logger().debug(
                f'Gorunurluk kapsami: {coverage:.1%}, '
                f'kor alan sayisi: {len(blind)}')


def main(args=None):
    """Arazi analiz dugumunu calistirir."""
    rclpy.init(args=args)
    node = TerrainAnalyzerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
