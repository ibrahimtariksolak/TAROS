"""
Onceden tanimli hava tehdidi senaryolarini calistirir.

Her senaryo bir dalga hedef drone uretir. Senaryolar algilama/takip/mudahale
zincirini tekrarlanabilir kosullarda test etmek icin kullanilir; AAR
karsilastirmalari ayni senaryo adiyla yapilir.
"""

import math

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node

from taros_msgs.srv import SpawnDrone


def _ring_scenario(count, radius):
    """Hedefleri korunan alanin etrafina esit acilarla dagitir."""
    drones = []
    types = ('quadrotor', 'hexarotor', 'quadrotor', 'fixedwing')
    for index in range(count):
        angle = 2.0 * math.pi * index / count
        drones.append({
            'type': types[index % len(types)],
            'pos': [
                radius * math.cos(angle),
                radius * math.sin(angle),
                30.0 + index * 2.0,
            ],
            'waypoints': [[0.0, 0.0, 25.0 + index]],
            'speed': 10.0 + (index % 5) * 3.0,
            'behavior': 'ATTACK',
        })
    return drones


SCENARIOS = {
    'single_recon': {
        'description': 'Tek quadrotor kesif ucusu',
        'drones': [
            {'type': 'quadrotor', 'pos': [300, 200, 50],
             'waypoints': [[100, 50, 50], [0, 0, 40], [-50, -30, 45]],
             'speed': 15.0, 'behavior': 'RECON'},
        ],
    },
    'dual_probe': {
        'description': 'Iki drone farkli yonden sizma',
        'drones': [
            {'type': 'quadrotor', 'pos': [400, 0, 60],
             'waypoints': [[200, 0, 55], [50, 0, 40], [0, 0, 30]],
             'speed': 18.0, 'behavior': 'ATTACK'},
            {'type': 'fixedwing', 'pos': [-300, 300, 80],
             'waypoints': [[-100, 100, 70], [0, 0, 60]],
             'speed': 25.0, 'behavior': 'TRANSIT'},
        ],
    },
    'swarm_wave': {
        'description': '10 drone dalga saldirisi',
        'drones': [
            {'type': 'quadrotor',
             'pos': [500 + i * 20, -100 + i * 25, 40 + i * 3],
             'waypoints': [[0, 0, 35]],
             'speed': 12.0 + i * 0.5, 'behavior': 'ATTACK'}
            for i in range(10)
        ],
    },
    'swarm_saturate': {
        'description': '20 drone doyurma saldirisi, tum yonlerden',
        'drones': _ring_scenario(20, 400.0),
    },
    'mixed_loiter': {
        'description': 'Kesif + loiter + saldiri karisik',
        'drones': [
            {'type': 'fixedwing', 'pos': [600, 0, 100],
             'waypoints': [[300, 0, 90], [100, 0, 80], [0, 0, 70]],
             'speed': 30.0, 'behavior': 'RECON'},
            {'type': 'quadrotor', 'pos': [0, 500, 60],
             'waypoints': [[0, 200, 55]],
             'speed': 8.0, 'behavior': 'LOITER'},
            {'type': 'hexarotor', 'pos': [-400, -200, 50],
             'waypoints': [[-100, -50, 40], [0, 0, 30]],
             'speed': 14.0, 'behavior': 'ATTACK'},
        ],
    },
}


class ScenarioManagerNode(Node):
    """Secilen senaryodaki hedefleri sirayla spawn eder."""

    def __init__(self):
        super().__init__('scenario_manager')

        self.declare_parameter('scenario', 'single_recon')
        self.declare_parameter('delay_between_spawns', 2.0)

        self.scenario_valid = False
        scenario_name = self.get_parameter('scenario').value

        if scenario_name not in SCENARIOS:
            self.get_logger().error(
                f'Bilinmeyen senaryo: {scenario_name}. '
                f'Gecerli secenekler: {list(SCENARIOS)}')
            return

        scenario = SCENARIOS[scenario_name]
        self._drones = list(scenario['drones'])
        self._spawn_index = 0
        self._spawned = 0
        self._failed = 0
        self._reported = False

        self._client = self.create_client(SpawnDrone, 'taros/spawn_drone')
        delay = float(self.get_parameter('delay_between_spawns').value)
        self._timer = self.create_timer(delay, self._spawn_next)

        self.scenario_valid = True
        self.get_logger().info(
            f'Senaryo: {scenario_name} - {scenario["description"]} '
            f'({len(self._drones)} hedef)')

    def _spawn_next(self):
        if self._spawn_index >= len(self._drones):
            # Tum istekler gonderildi; sonuclar hala yolda olabilir, bu yuzden
            # tamamlandi raporunu son cevap gelene kadar bekletiyoruz.
            self._timer.cancel()
            self._report_if_done()
            return

        if not self._client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('taros/spawn_drone hazir degil, bekleniyor...')
            return

        drone = self._drones[self._spawn_index]
        self._spawn_index += 1

        request = SpawnDrone.Request()
        request.drone_type = drone['type']
        request.spawn_position = Point(
            x=float(drone['pos'][0]),
            y=float(drone['pos'][1]),
            z=float(drone['pos'][2]))
        request.waypoints = [
            Point(x=float(wp[0]), y=float(wp[1]), z=float(wp[2]))
            for wp in drone['waypoints']
        ]
        request.speed = float(drone['speed'])
        request.behavior = drone['behavior']

        future = self._client.call_async(request)
        future.add_done_callback(self._on_spawn_done)

    def _on_spawn_done(self, future):
        result = future.result()
        if result is not None and result.success:
            self._spawned += 1
            self.get_logger().info(f'  + {result.drone_id}: {result.message}')
        else:
            self._failed += 1
            reason = result.message if result is not None else 'cagri basarisiz'
            self.get_logger().error(f'  - Spawn basarisiz: {reason}')
        self._report_if_done()

    def _report_if_done(self):
        """Tum istekler cevaplandiginda tek seferlik ozet basar."""
        total = len(self._drones)
        if self._spawn_index < total:
            return
        if self._spawned + self._failed < total:
            return
        if self._reported:
            return
        self._reported = True

        if self._failed:
            self.get_logger().error(
                f'Senaryo tamam: {self._spawned}/{total} hedef uretildi, '
                f'{self._failed} basarisiz')
        else:
            self.get_logger().info(
                f'Senaryo tamam: {self._spawned}/{total} hedef uretildi')


def main(args=None):
    rclpy.init(args=args)
    node = ScenarioManagerNode()
    try:
        if node.scenario_valid:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
