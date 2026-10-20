"""
Hava kosullari yayincisi.

Sensor basirimini etkileyen meteorolojik kosullari yayinlar. SetWeather
servisi ile senaryo icerisinde dinamik olarak degistirilebilir.
"""

import rclpy
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from taros_msgs.msg import WeatherCondition
from taros_msgs.srv import SetWeather


class WeatherNode(Node):
    """Hava durumu yayincisi ve SetWeather servis sunucusu."""

    def __init__(self):
        """Hava durumu parametrelerini yukler ve yayinci zamanlayicisini baslatir."""
        super().__init__('weather_node')

        self.declare_parameter('visibility', 10000.0)
        self.declare_parameter('rain_rate', 0.0)
        self.declare_parameter('fog_density', 0.0)
        self.declare_parameter('wind_x', 0.0)
        self.declare_parameter('wind_y', 0.0)
        self.declare_parameter('wind_z', 0.0)
        self.declare_parameter('is_night', False)
        self.declare_parameter('temperature', 20.0)
        self.declare_parameter('publish_rate', 1.0)

        self._visibility = self.get_parameter('visibility').value
        self._rain_rate = self.get_parameter('rain_rate').value
        self._fog_density = self.get_parameter('fog_density').value
        self._wind = Vector3(
            x=self.get_parameter('wind_x').value,
            y=self.get_parameter('wind_y').value,
            z=self.get_parameter('wind_z').value,
        )
        self._is_night = self.get_parameter('is_night').value
        self._temperature = self.get_parameter('temperature').value
        rate = self.get_parameter('publish_rate').value

        latched_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._pub = self.create_publisher(
            WeatherCondition, 'taros/environment/weather', latched_qos)

        self._srv = self.create_service(
            SetWeather, 'taros/set_weather', self._handle_set_weather)

        self._timer = self.create_timer(1.0 / rate, self._publish)

        self._publish()

        self.get_logger().info(
            f'Hava durumu baslatildi: gorunurluk={self._visibility}m, '
            f'yagmur={self._rain_rate}mm/h, gece={self._is_night}')

    def _handle_set_weather(self, request, response):
        """Hava kosullarini gunceller."""
        self._visibility = request.visibility
        self._rain_rate = request.rain_rate
        self._fog_density = request.fog_density
        self._wind = request.wind
        self._is_night = request.is_night

        self._publish()

        self.get_logger().info(
            f'Hava guncellendi: gorunurluk={self._visibility}m, '
            f'yagmur={self._rain_rate}mm/h, sis={self._fog_density}, '
            f'gece={self._is_night}')

        response.success = True
        return response

    def _publish(self):
        msg = WeatherCondition()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.visibility = self._visibility
        msg.rain_rate = self._rain_rate
        msg.fog_density = self._fog_density
        msg.wind = self._wind
        msg.is_night = self._is_night
        msg.temperature = self._temperature
        self._pub.publish(msg)


def main(args=None):
    """Hava durumu dugumunu calistirir."""
    rclpy.init(args=args)
    node = WeatherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
