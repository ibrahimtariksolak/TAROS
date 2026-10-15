"""
Yerlestirilen drone'lari ucus planina gore ucurur ve ground truth yayinlar.

Hedef drone'lar tam aerodinamik modelle degil kinematik olarak simule edilir:
amac hedeflerin kendisini dogrulamak degil, algilama/takip zincirini beslemek.
Ground truth cikisi tracker dogrulugunu olcmek icin referans olarak kullanilir.
"""

import math

import rclpy
from gazebo_msgs.msg import EntityState, ModelStates
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Pose, Vector3
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from taros_msgs.msg import DroneFlightPlan, Track, TrackArray

# Bir sonraki rota noktasina gecis yaricapi [m]
WAYPOINT_ARRIVAL_RADIUS = 5.0

# LOITER davranisinda daire yaricapi [m]
LOITER_RADIUS = 30.0

# Track durumu: ground truth her zaman CONFIRMED kabul edilir
TRACK_STATUS_CONFIRMED = 1

TYPE_TO_CLASS = {
    'quadrotor': 'COMMERCIAL_QUAD',
    'fixedwing': 'COMMERCIAL_FIXEDWING',
    'hexarotor': 'MILITARY',
}


class DronePilotNode(Node):
    """Ucus plani alir, drone'u hareket ettirir, ground truth track yayinlar."""

    def __init__(self):
        super().__init__('drone_pilot')

        self.declare_parameter('update_rate', 30.0)
        self._update_rate = float(self.get_parameter('update_rate').value)
        self._dt = 1.0 / self._update_rate

        self._cb_group = ReentrantCallbackGroup()

        self._drones = {}
        self._live_models = set()
        self._seen_model_states = False
        self._track_ids = {}
        self._next_track_id = 0
        self._pending_state_calls = set()

        plan_qos = QoSProfile(
            depth=64,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._plan_sub = self.create_subscription(
            DroneFlightPlan, 'taros/drone_flight_plan',
            self._on_flight_plan, plan_qos, callback_group=self._cb_group)

        self._state_sub = self.create_subscription(
            ModelStates, '/gazebo/model_states',
            self._on_model_states, 10, callback_group=self._cb_group)

        self._set_state_client = self.create_client(
            SetEntityState, '/gazebo/set_entity_state',
            callback_group=self._cb_group)

        self._gt_pub = self.create_publisher(
            TrackArray, 'taros/ground_truth/tracks', 10)

        self._timer = self.create_timer(
            self._dt, self._update, callback_group=self._cb_group)

        self.get_logger().info(f'Drone pilot {self._update_rate:.0f} Hz calisiyor')

    def _on_flight_plan(self, msg):
        """Spawner'dan gelen ucus planini kaydeder."""
        if msg.drone_id in self._drones:
            return

        self._drones[msg.drone_id] = {
            'type': msg.drone_type,
            'waypoints': [(p.x, p.y, p.z) for p in msg.waypoints],
            'wp_index': 0,
            'speed': msg.speed,
            'behavior': msg.behavior,
            'loiter_center': None,
            'loiter_angle': 0.0,
            # Otoritatif kinematik durum. Gazebo'dan poz geri okumuyoruz:
            # hedefler teleport ile surulduğu icin fizik motorunun yercekimi
            # sapmasi geri besleme dongusune girip irtifa kaybina yol aciyordu.
            'position': (
                msg.spawn_position.x, msg.spawn_position.y, msg.spawn_position.z),
            'yaw': 0.0,
            # Gazebo'da en az bir kez gorulene kadar imha kontrolune girmez.
            'confirmed': False,
        }
        self._track_ids[msg.drone_id] = self._next_track_id
        self._next_track_id += 1

        self.get_logger().info(
            f'Ucus plani alindi: {msg.drone_id} ({msg.behavior}, '
            f'{len(msg.waypoints)} rota noktasi)')

    def _on_model_states(self, msg):
        """
        Gazebo'da hangi hedeflerin hayatta oldugunu izler.

        Poz geri okumak icin degil: mudahale sonrasi silinen hedefleri
        takipten dusurmek icin kullanilir.
        """
        self._live_models = {name for name in msg.name if name.startswith('drone_')}
        self._seen_model_states = True

    def _retire_destroyed(self):
        """
        Gazebo'dan silinmis hedefleri ucus listesinden cikarir.

        Bir hedef ancak model_states'te en az bir kez goruldukten SONRA
        kaybolursa dusurulur. Aksi halde spawn ile model_states guncellemesi
        arasindaki birkac milisaniyelik pencere, yeni dogmus hedefi aninda
        imha edilmis sayar.
        """
        if not self._seen_model_states:
            return

        for name, info in self._drones.items():
            if not info['confirmed'] and name in self._live_models:
                info['confirmed'] = True

        gone = [
            name for name, info in self._drones.items()
            if info['confirmed'] and name not in self._live_models
        ]
        for name in gone:
            del self._drones[name]
            self._track_ids.pop(name, None)
            self._pending_state_calls.discard(name)
            self.get_logger().info(f'Hedef takipten dusuruldu: {name}')

    def _update(self):
        self._retire_destroyed()
        tracks = []

        # Snapshot: reentrant callback grubu altinda _on_flight_plan ayni anda
        # yeni hedef ekleyebilir, canli sozluk uzerinde dongu patlar.
        for drone_id, info in list(self._drones.items()):
            position = info['position']
            velocity = self._compute_velocity(info, position)

            new_pose = self._advance(info, velocity)
            self._apply_state(drone_id, new_pose, velocity)
            tracks.append(self._build_track(drone_id, info, new_pose, velocity))

        if tracks:
            message = TrackArray()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = 'world'
            message.tracks = tracks
            self._gt_pub.publish(message)

    def _compute_velocity(self, info, position):
        """Davranisa gore istenen hiz vektorunu uretir."""
        behavior = info['behavior']
        if behavior == 'LOITER':
            return self._loiter_velocity(info, position)
        return self._waypoint_velocity(info, position)

    def _loiter_velocity(self, info, position):
        if info['loiter_center'] is None:
            info['loiter_center'] = position

        speed = info['speed']
        center_x, center_y, center_z = info['loiter_center']
        info['loiter_angle'] += (speed / LOITER_RADIUS) * self._dt

        target = (
            center_x + LOITER_RADIUS * math.cos(info['loiter_angle']),
            center_y + LOITER_RADIUS * math.sin(info['loiter_angle']),
            center_z,
        )
        return self._velocity_towards(position, target, speed)

    def _waypoint_velocity(self, info, position):
        waypoints = info['waypoints']
        if not waypoints:
            return (0.0, 0.0, 0.0)

        index = info['wp_index']
        if index >= len(waypoints):
            # RECON gorevleri rotayi bastan tekrarlar, digerleri son
            # noktada bekler.
            if info['behavior'] != 'RECON':
                return (0.0, 0.0, 0.0)
            info['wp_index'] = 0
            index = 0

        target = waypoints[index]
        if self._distance(position, target) < WAYPOINT_ARRIVAL_RADIUS:
            info['wp_index'] = index + 1

        return self._velocity_towards(position, target, info['speed'])

    @staticmethod
    def _distance(a, b):
        return math.dist(a, b)

    @staticmethod
    def _velocity_towards(origin, target, speed):
        dx = target[0] - origin[0]
        dy = target[1] - origin[1]
        dz = target[2] - origin[2]
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)
        if distance < 1e-6:
            return (0.0, 0.0, 0.0)
        scale = speed / distance
        return (dx * scale, dy * scale, dz * scale)

    def _advance(self, info, velocity):
        """Kinematik durumu bir adim ileri entegre edip yeni pozu dondurur."""
        position = info['position']
        advanced = (
            position[0] + velocity[0] * self._dt,
            position[1] + velocity[1] * self._dt,
            position[2] + velocity[2] * self._dt,
        )
        info['position'] = advanced

        # Hiz sifira dustugunde (hedefe varildiginda) son yonu koruyoruz.
        if abs(velocity[0]) > 1e-3 or abs(velocity[1]) > 1e-3:
            info['yaw'] = math.atan2(velocity[1], velocity[0])

        pose = Pose()
        pose.position.x = advanced[0]
        pose.position.y = advanced[1]
        pose.position.z = advanced[2]
        pose.orientation.z = math.sin(info['yaw'] * 0.5)
        pose.orientation.w = math.cos(info['yaw'] * 0.5)
        return pose

    def _apply_state(self, drone_id, pose, velocity):
        """Gazebo'da model durumunu guncellerken ucus halindeki istekleri sinirlar."""
        if not self._set_state_client.service_is_ready():
            return
        if drone_id in self._pending_state_calls:
            # Onceki istek hala donmediyse yenisini kuyruga atmiyoruz;
            # aksi halde future'lar birikip bellek sizdirir.
            return

        request = SetEntityState.Request()
        request.state = EntityState()
        request.state.name = drone_id
        request.state.pose = pose
        # Twist'i de vermek Gazebo'nun guncellemeler arasinda hareketi
        # puruzsuz interpole etmesini saglar.
        request.state.twist.linear = Vector3(
            x=velocity[0], y=velocity[1], z=velocity[2])
        request.state.reference_frame = 'world'

        self._pending_state_calls.add(drone_id)
        future = self._set_state_client.call_async(request)
        future.add_done_callback(
            lambda _, name=drone_id: self._pending_state_calls.discard(name))

    def _build_track(self, drone_id, info, pose, velocity):
        track = Track()
        track.track_id = self._track_ids[drone_id]
        track.status = TRACK_STATUS_CONFIRMED
        track.classification = TYPE_TO_CLASS.get(info['type'], 'UNKNOWN')
        track.position = pose.position
        track.velocity = Vector3(x=velocity[0], y=velocity[1], z=velocity[2])
        track.speed = math.sqrt(sum(component * component for component in velocity))
        # Hedef durdugunda atan2(0, 0) = 0 olacagi icin son bilinen yonu
        # kullaniyoruz; aksi halde ground truth'ta sahte yon atlamasi olusur.
        track.heading = info['yaw']
        track.altitude = pose.position.z
        track.range_to_base = math.hypot(pose.position.x, pose.position.y)
        if track.speed > 0.1:
            track.time_to_arrival = track.range_to_base / track.speed
        track.confidence = 1.0
        track.sensor_contributions = [1.0, 1.0, 1.0]
        track.last_update = self.get_clock().now().to_msg()
        return track


def main(args=None):
    rclpy.init(args=args)
    node = DronePilotNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
