"""Hedef drone'lari Gazebo'ya yerlestirir ve ucus planlarini pilota devreder."""

import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from taros_gazebo.validation import DRONE_TYPES, validate_spawn_request
from taros_msgs.msg import DroneFlightPlan
from taros_msgs.srv import SpawnDrone

# En dusuk guvenli spawn irtifasi. Drone modelleri ucus halinde simule edildigi
# icin yer seviyesinde dogarlarsa zemine carpip devrilirler.
MIN_SPAWN_ALTITUDE = 20.0


class DroneSpawnerNode(Node):
    """``taros/spawn_drone`` servisini sunar, Gazebo'ya model yerlestirir."""

    def __init__(self):
        super().__init__('drone_spawner')

        # Gazebo servis cagrisi bir servis callback'i icinden yapildigi icin
        # reentrant grup + cok is parcacikli executor gerekiyor; aksi halde
        # tek is parcacikli executor kendini bekleyip kilitlenir.
        self._cb_group = ReentrantCallbackGroup()

        self._spawn_client = self.create_client(
            SpawnEntity, '/spawn_entity', callback_group=self._cb_group)

        # Pilot bu node'dan sonra baslasa bile planlari alabilsin diye
        # transient_local (latched) kullaniyoruz.
        plan_qos = QoSProfile(
            depth=64,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._plan_pub = self.create_publisher(
            DroneFlightPlan, 'taros/drone_flight_plan', plan_qos)

        self._srv = self.create_service(
            SpawnDrone, 'taros/spawn_drone', self._handle_spawn,
            callback_group=self._cb_group)

        self._model_cache = {}
        self._drone_counter = 0

        self.get_logger().info('Drone spawner ready')

    def _load_model_sdf(self, drone_type):
        """
        Model SDF'ini diskten okur ve onbellege alir.

        drone_type disaridan (servis istegi) geldigi icin dosya yoluna
        girmeden once whitelist'e karsi dogrulanir. Cagiran tarafta da ayni
        kontrol var; burada tekrarlamak fonksiyonu tek basina guvenli kilar
        ve '../' tarzi yol kacisini imkansiz birakir.
        """
        if drone_type not in DRONE_TYPES:
            self.get_logger().error(f'Whitelist disi model tipi reddedildi: {drone_type!r}')
            return None

        if drone_type in self._model_cache:
            return self._model_cache[drone_type]

        sdf_path = os.path.join(
            get_package_share_directory('taros_gazebo'),
            'models', drone_type, 'model.sdf')

        if not os.path.exists(sdf_path):
            return None

        with open(sdf_path, 'r') as handle:
            sdf_xml = handle.read()

        self._model_cache[drone_type] = sdf_xml
        return sdf_xml

    def _handle_spawn(self, request, response):
        error = validate_spawn_request(
            request.drone_type, request.speed,
            request.spawn_position, request.waypoints)
        if error is not None:
            response.success = False
            response.message = error
            self.get_logger().error(error)
            return response

        sdf_xml = self._load_model_sdf(request.drone_type)
        if sdf_xml is None:
            response.success = False
            response.message = f'Model SDF bulunamadi: {request.drone_type}'
            self.get_logger().error(response.message)
            return response

        if not self._spawn_client.wait_for_service(timeout_sec=5.0):
            response.success = False
            response.message = 'Gazebo /spawn_entity servisi hazir degil'
            self.get_logger().error(response.message)
            return response

        self._drone_counter += 1
        drone_id = f'drone_{request.drone_type}_{self._drone_counter}'

        pose = Pose()
        pose.position.x = request.spawn_position.x
        pose.position.y = request.spawn_position.y
        pose.position.z = max(request.spawn_position.z, MIN_SPAWN_ALTITUDE)

        spawn_req = SpawnEntity.Request()
        spawn_req.name = drone_id
        spawn_req.xml = sdf_xml
        spawn_req.initial_pose = pose
        spawn_req.reference_frame = 'world'

        # ReentrantCallbackGroup + MultiThreadedExecutor altinda senkron
        # call() guvenli; spin_until_future_complete ile kendi executor'umuzu
        # yeniden dondurmek yerine bunu kullaniyoruz.
        gazebo_result = self._spawn_client.call(spawn_req)

        if gazebo_result is None or not gazebo_result.success:
            reason = gazebo_result.status_message if gazebo_result else 'cevap yok'
            self._drone_counter -= 1
            response.success = False
            response.message = f'Gazebo spawn basarisiz: {reason}'
            self.get_logger().error(response.message)
            return response

        self._publish_flight_plan(drone_id, request, pose)

        response.success = True
        response.drone_id = drone_id
        response.message = (
            f'{request.behavior} gorevi, {request.speed:.0f} m/s, '
            f'z={pose.position.z:.0f} m')
        self.get_logger().info(f'{drone_id} yerlestirildi -> {response.message}')
        return response

    def _publish_flight_plan(self, drone_id, request, spawn_pose):
        """Pilotun drone'u ucurabilmesi icin ucus planini yayinlar."""
        plan = DroneFlightPlan()
        plan.header.stamp = self.get_clock().now().to_msg()
        plan.header.frame_id = 'world'
        plan.drone_id = drone_id
        plan.drone_type = request.drone_type
        # Pilot kinematik durumu buradan baslatir; Gazebo'dan poz okuyup
        # uzerine entegre etmek yercekimi sapmasini geri besleme ile takip
        # ettirir ve hedefler yavasca batar.
        plan.spawn_position = spawn_pose.position
        plan.waypoints = list(request.waypoints)
        plan.speed = request.speed
        plan.behavior = request.behavior
        self._plan_pub.publish(plan)


def main(args=None):
    rclpy.init(args=args)
    node = DroneSpawnerNode()
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
