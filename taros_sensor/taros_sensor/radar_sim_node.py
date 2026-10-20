"""
3D arama radar simulatoru.

X-band darbeli Doppler radar modeli. Ground truth'u alir, radar denklemine
gore SNR hesaplar, Swerling-I tespit olasiligi uygular ve tespit edilen
hedefler icin gurultulu olcumler uretir.
"""

import math
import random

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node

from taros_msgs.msg import (
    RadarReturn,
    RadarReturnArray,
    TrackArray,
    WeatherCondition,
)

from taros_sensor.sensor_models import (
    RCS_DBSM,
    azimuth_elevation,
    db_to_linear,
    distance_3d,
    doppler_velocity,
    radar_angle_noise,
    radar_range_noise,
    radar_snr,
    radar_system_constant,
    swerling_i_pd,
)


class RadarSimNode(Node):
    """Donen 3D arama radar simulatoru."""

    def __init__(self):
        """Radar parametrelerini yukler ve tarama zamanlayicisini baslatir."""
        super().__init__('radar_sim')

        self.declare_parameter('update_rate', 2.0)
        self.declare_parameter('max_range', 10000.0)
        self.declare_parameter('min_range', 50.0)
        self.declare_parameter('azimuth_beamwidth_deg', 2.0)
        self.declare_parameter('elevation_beamwidth_deg', 5.0)
        self.declare_parameter('scan_rate_rpm', 15.0)
        self.declare_parameter('range_resolution', 15.0)
        self.declare_parameter('detection_threshold_db', 13.0)
        self.declare_parameter('false_alarm_rate', 1e-6)
        self.declare_parameter('radar_x', 0.0)
        self.declare_parameter('radar_y', 0.0)
        self.declare_parameter('radar_z', 10.0)

        self._rate = self.get_parameter('update_rate').value
        self._max_range = self.get_parameter('max_range').value
        self._min_range = self.get_parameter('min_range').value
        self._az_bw = math.radians(self.get_parameter('azimuth_beamwidth_deg').value)
        self._el_bw = math.radians(self.get_parameter('elevation_beamwidth_deg').value)
        self._scan_rpm = self.get_parameter('scan_rate_rpm').value
        self._range_res = self.get_parameter('range_resolution').value
        self._threshold_db = self.get_parameter('detection_threshold_db').value
        self._pfa = self.get_parameter('false_alarm_rate').value
        self._radar_pos = (
            self.get_parameter('radar_x').value,
            self.get_parameter('radar_y').value,
            self.get_parameter('radar_z').value,
        )

        self._k_sys = radar_system_constant()
        self._scan_angle = 0.0
        self._detection_id = 0
        self._rain_rate = 0.0

        self._latest_tracks = None

        self._track_sub = self.create_subscription(
            TrackArray, 'taros/ground_truth/tracks',
            self._on_tracks, 10)

        self._weather_sub = self.create_subscription(
            WeatherCondition, 'taros/environment/weather',
            self._on_weather, 10)

        self._pub = self.create_publisher(
            RadarReturnArray, 'taros/sensor/radar/returns', 10)

        self._timer = self.create_timer(1.0 / self._rate, self._scan_cycle)

        self.get_logger().info(
            f'Radar sim baslatildi: menzil={self._max_range}m, '
            f'tarama={self._scan_rpm}rpm, frekans={self._rate}Hz')

    def _on_tracks(self, msg):
        self._latest_tracks = msg

    def _on_weather(self, msg):
        self._rain_rate = msg.rain_rate

    def _scan_cycle(self):
        """Bir tarama dongusu: antenin baktigi yondeki hedefleri degerlendir."""
        scan_advance = (self._scan_rpm * 360.0 / 60.0) * (1.0 / self._rate)
        self._scan_angle += math.radians(scan_advance)
        if self._scan_angle > math.pi:
            self._scan_angle -= 2.0 * math.pi

        if self._latest_tracks is None:
            return

        returns = []
        for track in self._latest_tracks.tracks:
            ret = self._evaluate_target(track)
            if ret is not None:
                returns.append(ret)

        returns.extend(self._generate_false_alarms())

        if returns:
            msg = RadarReturnArray()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'radar'
            msg.returns = returns
            self._pub.publish(msg)

    def _evaluate_target(self, track):
        """Tek hedef icin radar tespiti degerlendir."""
        target_pos = (track.position.x, track.position.y, track.position.z)
        target_vel = (track.velocity.x, track.velocity.y, track.velocity.z)

        rng = distance_3d(self._radar_pos, target_pos)
        if rng < self._min_range or rng > self._max_range:
            return None

        az, el = azimuth_elevation(self._radar_pos, target_pos)

        az_diff = abs(self._wrap(az - self._scan_angle))
        if az_diff > self._az_bw:
            return None

        if el < math.radians(-5.0) or el > math.radians(45.0):
            return None

        rcs = RCS_DBSM.get(track.classification, -8.0)
        snr_db = radar_snr(rng, rcs, self._k_sys, self._rain_rate)
        pd = swerling_i_pd(snr_db, self._threshold_db)

        if random.random() > pd:
            return None

        snr_lin = db_to_linear(snr_db)
        range_noise = random.gauss(0, radar_range_noise(self._range_res, snr_lin))
        az_noise = random.gauss(0, radar_angle_noise(self._az_bw, snr_lin))
        el_noise = random.gauss(0, radar_angle_noise(self._el_bw, snr_lin))

        meas_range = rng + range_noise
        meas_az = az + az_noise
        meas_el = el + el_noise

        radial_vel = doppler_velocity(target_vel, self._radar_pos, target_pos)
        vel_noise = random.gauss(0, 0.5)

        cos_el = math.cos(meas_el)
        enu_x = meas_range * cos_el * math.cos(meas_az) + self._radar_pos[0]
        enu_y = meas_range * cos_el * math.sin(meas_az) + self._radar_pos[1]
        enu_z = meas_range * math.sin(meas_el) + self._radar_pos[2]

        self._detection_id += 1
        ret = RadarReturn()
        ret.header.stamp = self.get_clock().now().to_msg()
        ret.header.frame_id = 'radar'
        ret.detection_id = self._detection_id
        ret.range = meas_range
        ret.azimuth = meas_az
        ret.elevation = meas_el
        ret.doppler_velocity = radial_vel + vel_noise
        ret.rcs = rcs + random.gauss(0, 2.0)
        ret.snr = snr_db
        ret.position_enu = Point(x=enu_x, y=enu_y, z=enu_z)
        return ret

    def _generate_false_alarms(self):
        """Poisson dagilimli yanlis alarm uretir."""
        num_range_cells = int(self._max_range / self._range_res)
        expected = self._pfa * num_range_cells
        n_fa = 0
        if expected > 0:
            n_fa = random.randint(0, max(1, int(expected * 10)))
            if random.random() > expected * 10:
                n_fa = 0

        alarms = []
        for _ in range(n_fa):
            fa_range = random.uniform(self._min_range, self._max_range)
            fa_az = self._scan_angle + random.gauss(0, self._az_bw)
            fa_el = random.uniform(math.radians(-5.0), math.radians(45.0))

            cos_el = math.cos(fa_el)
            enu_x = fa_range * cos_el * math.cos(fa_az) + self._radar_pos[0]
            enu_y = fa_range * cos_el * math.sin(fa_az) + self._radar_pos[1]
            enu_z = fa_range * math.sin(fa_el) + self._radar_pos[2]

            self._detection_id += 1
            ret = RadarReturn()
            ret.header.stamp = self.get_clock().now().to_msg()
            ret.header.frame_id = 'radar'
            ret.detection_id = self._detection_id
            ret.range = fa_range
            ret.azimuth = fa_az
            ret.elevation = fa_el
            ret.doppler_velocity = random.gauss(0, 5.0)
            ret.rcs = random.uniform(-20.0, 0.0)
            ret.snr = self._threshold_db + random.uniform(0, 3.0)
            ret.position_enu = Point(x=enu_x, y=enu_y, z=enu_z)
            alarms.append(ret)
        return alarms

    @staticmethod
    def _wrap(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    """Radar simulasyon dugumunu calistirir."""
    rclpy.init(args=args)
    node = RadarSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
