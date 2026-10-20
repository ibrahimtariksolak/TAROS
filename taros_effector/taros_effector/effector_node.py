"""
Efektör dugumu — RF karistirma, GPS aldatma ve kinetik onleme.

Man-in-the-loop: yetki verilmeden hicbir efektor devreye girmez.
Engage servisi cagirildiginda AuthorizeEngagement servisiyle onay
istenir; onay gelmedikce mudahale baslamaz.
"""

import rclpy
from rclpy.node import Node

from taros_msgs.msg import EngagementCommand, TrackArray
from taros_msgs.srv import AuthorizeEngagement, Engage

RF_JAMMER = 0
GPS_SPOOFER = 1
KINETIC_INTERCEPTOR = 2

PENDING = 0
AUTHORIZED = 1
DENIED = 2

EFFECTOR_NAMES = {
    RF_JAMMER: 'RF_JAMMER',
    GPS_SPOOFER: 'GPS_SPOOFER',
    KINETIC_INTERCEPTOR: 'KINETIC',
}


class EffectorNode(Node):
    """Man-in-the-loop efektör dugumu."""

    def __init__(self):
        """Efektör servislerini baslatir."""
        super().__init__('effector')

        self._engagement_id = 0
        self._active_engagements = {}

        self._engage_srv = self.create_service(
            Engage, 'taros/engage', self._handle_engage)

        self._auth_srv = self.create_service(
            AuthorizeEngagement, 'taros/authorize_engagement',
            self._handle_authorize)

        self._cmd_pub = self.create_publisher(
            EngagementCommand, 'taros/effector/commands', 10)

        self._track_sub = self.create_subscription(
            TrackArray, 'taros/tracker/tracks',
            self._on_tracks, 10)

        self._latest_tracks = {}

        self.get_logger().info('Efektör dugumu baslatildi (man-in-the-loop)')

    def _on_tracks(self, msg):
        """Aktif izleri gunceller."""
        self._latest_tracks = {t.track_id: t for t in msg.tracks}

    def _handle_engage(self, request, response):
        """Mudahale istegini isler — onay bekler."""
        track_id = request.track_id
        effector_type = request.effector_type

        if track_id not in self._latest_tracks:
            response.success = False
            response.message = f'Hedef {track_id} aktif degil'
            response.engagement_id = 0
            return response

        if effector_type not in EFFECTOR_NAMES:
            response.success = False
            response.message = f'Bilinmeyen efektor tipi: {effector_type}'
            response.engagement_id = 0
            return response

        self._engagement_id += 1
        eid = self._engagement_id

        self._active_engagements[eid] = {
            'track_id': track_id,
            'effector_type': effector_type,
            'authorization': PENDING,
        }

        cmd = EngagementCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.track_id = track_id
        cmd.effector_type = effector_type
        cmd.authorization = PENDING
        cmd.priority = 5
        cmd.unit_id = 'TAROS_BASE'
        self._cmd_pub.publish(cmd)

        self.get_logger().info(
            f'Mudahale istegi #{eid}: hedef={track_id}, '
            f'efektor={EFFECTOR_NAMES[effector_type]}, '
            f'durum=ONAY_BEKLIYOR')

        response.success = True
        response.message = 'Yetkilendirme bekleniyor'
        response.engagement_id = eid
        return response

    def _handle_authorize(self, request, response):
        """Operatör onay/red karari."""
        eid = None
        for eng_id, eng in self._active_engagements.items():
            if (eng['track_id'] == request.track_id
                    and eng['effector_type'] == request.effector_type
                    and eng['authorization'] == PENDING):
                eid = eng_id
                break

        if eid is None:
            response.authorized = False
            response.reason = 'Bekleyen mudahale istegi bulunamadi'
            return response

        self._active_engagements[eid]['authorization'] = AUTHORIZED

        cmd = EngagementCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.track_id = request.track_id
        cmd.effector_type = request.effector_type
        cmd.authorization = AUTHORIZED
        cmd.priority = 3
        cmd.unit_id = 'TAROS_BASE'
        self._cmd_pub.publish(cmd)

        self.get_logger().info(
            f'Mudahale #{eid} YETKILENDIRILDI: '
            f'hedef={request.track_id}, '
            f'efektor={EFFECTOR_NAMES.get(request.effector_type, "?")}')

        response.authorized = True
        response.reason = 'Operator onayladi'
        return response


def main(args=None):
    """Efektör dugumunu calistirir."""
    rclpy.init(args=args)
    node = EffectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
