"""UGV otonom intikal ve kinematik simulasyon dugumu."""

import math

import rclpy
from geometry_msgs.msg import Point, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node

from taros_navigation.path_planner import direct_path


class NavigationNode(Node):
    """UGV navigasyon simulasyon dugumu."""

    def __init__(self):
        """Navigasyon dugumunu baslatir."""
        super().__init__('navigation')

        self.declare_parameter('max_speed', 5.0)
        self.declare_parameter('max_turn_rate', 1.0)
        self.declare_parameter('update_rate', 10.0)
        self.declare_parameter('waypoint_tolerance', 1.0)

        self._max_speed = self.get_parameter('max_speed').value
        self._max_turn = self.get_parameter('max_turn_rate').value
        self._tolerance = self.get_parameter('waypoint_tolerance').value

        self._x = 0.0
        self._y = 0.0
        self._heading = 0.0
        self._speed = 0.0
        self._path = []
        self._path_idx = 0

        self._goal_sub = self.create_subscription(
            Point, '/taros/navigation/goal',
            self._on_goal, 10)

        self._odom_pub = self.create_publisher(
            Odometry, '/taros/navigation/odom', 10)

        rate = self.get_parameter('update_rate').value
        self._dt = 1.0 / rate
        self._timer = self.create_timer(self._dt, self._update)

        self.get_logger().info(
            f'Navigasyon baslatildi: max_hiz={self._max_speed} m/s')

    def _on_goal(self, msg):
        """Hedef konumu alir ve yol planlar."""
        start = (self._x, self._y)
        goal = (msg.x, msg.y)
        self._path = direct_path(start, goal, step_size=2.0)
        self._path_idx = 0
        self.get_logger().info(
            f'Yeni hedef: ({msg.x:.1f}, {msg.y:.1f}), '
            f'{len(self._path)} adim')

    def _update(self):
        """Kinematik modeli bir adim ilerletir ve odometri yayinlar."""
        if self._path and self._path_idx < len(self._path):
            tx, ty = self._path[self._path_idx]
            dx = tx - self._x
            dy = ty - self._y
            dist = math.hypot(dx, dy)

            if dist < self._tolerance:
                self._path_idx += 1
                if self._path_idx >= len(self._path):
                    self._speed = 0.0
                    self.get_logger().info('Hedefe ulasildi')
            else:
                target_heading = math.atan2(dy, dx)
                heading_err = self._normalize_angle(
                    target_heading - self._heading)

                turn = max(-self._max_turn * self._dt,
                           min(self._max_turn * self._dt, heading_err))
                self._heading += turn

                if abs(heading_err) < 0.3:
                    self._speed = min(self._max_speed, dist / self._dt)
                else:
                    self._speed = self._max_speed * 0.3

                self._x += self._speed * math.cos(self._heading) * self._dt
                self._y += self._speed * math.sin(self._heading) * self._dt

        self._publish_odom()

    @staticmethod
    def _normalize_angle(angle):
        """Aciyi -pi ile pi arasina normalize eder."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def _publish_odom(self):
        """Odometri mesajini yayinlar."""
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'

        msg.pose.pose.position = Point(
            x=self._x, y=self._y, z=0.0)

        half = self._heading / 2.0
        msg.pose.pose.orientation = Quaternion(
            x=0.0, y=0.0,
            z=math.sin(half), w=math.cos(half))

        msg.twist.twist = Twist(
            linear=Vector3(x=self._speed, y=0.0, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=0.0))

        self._odom_pub.publish(msg)


def main(args=None):
    """Navigasyon dugumunu calistirir."""
    rclpy.init(args=args)
    node = NavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
