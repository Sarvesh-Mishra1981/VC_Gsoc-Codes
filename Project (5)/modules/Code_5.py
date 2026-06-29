import math
import numpy as np
import rclpy

from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    DurabilityPolicy,
    qos_profile_sensor_data,
)
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image


WIDTH = 1500
HEIGHT = 970


def quaternion_to_yaw(q):
    numerator = 2.0 * (q.w * q.z + q.x * q.y)
    denominator = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(numerator, denominator)


def pose_to_map(world_x, world_y):
    pixel_y = -23.58 * (-20.36 - world_x)
    pixel_x = -23.53 * (-31.95 - world_y)

    return round(pixel_x), round(pixel_y)


def bresenham(x0, y0, x1, y1):
    points = []

    dx = abs(x1 - x0)
    dy = abs(y1 - y0)

    step_x = 1 if x0 < x1 else -1
    step_y = 1 if y0 < y1 else -1

    error = dx - dy

    while True:
        points.append((x0, y0))

        if x0 == x1 and y0 == y1:
            break

        double_error = 2 * error

        if double_error > -dy:
            error -= dy
            x0 += step_x

        if double_error < dx:
            error += dx
            y0 += step_y

    return points


def main(inputs, outputs, parameters, synchronise):
    if not rclpy.ok():
        rclpy.init(args=None)

    class MapPublisher(Node):
        def __init__(self):
            super().__init__("vc_ra_map_publisher")

            self.world_x = 0.0
            self.world_y = 0.0
            self.yaw = 0.0

            self.have_odom = False
            self.scan = None

            self.last_x = None
            self.last_y = None
            self.last_yaw = None

            self.log_map = np.zeros(
                (HEIGHT, WIDTH),
                dtype=np.float32,
            )
            self.image_map = np.full(
                (HEIGHT, WIDTH),
                128,
                dtype=np.uint8,
            )

            map_qos = QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )

            self.map_publisher = self.create_publisher(
                Image,
                "/webgui/user_map",
                map_qos,
            )

            self.create_subscription(
                Odometry,
                "/turtlebot3/odom",
                self.odom_callback,
                qos_profile_sensor_data,
            )

            self.create_subscription(
                LaserScan,
                "/turtlebot3/laser/scan",
                self.scan_callback,
                qos_profile_sensor_data,
            )

        def odom_callback(self, msg):
            self.world_x = msg.pose.pose.position.x
            self.world_y = msg.pose.pose.position.y
            self.yaw = quaternion_to_yaw(
                msg.pose.pose.orientation
            )
            self.have_odom = True

        def scan_callback(self, msg):
            self.scan = msg

        def robot_moved(self):
            if self.last_x is None:
                return True

            distance = math.hypot(
                self.world_x - self.last_x,
                self.world_y - self.last_y,
            )

            angle_difference = abs(
                self.yaw - self.last_yaw
            )
            angle_difference = min(
                angle_difference,
                2.0 * math.pi - angle_difference,
            )

            return (
                distance > 0.05
                or angle_difference > 0.05
            )

        def update_map(self):
            if not self.have_odom or self.scan is None:
                return

            if not self.robot_moved():
                return

            self.last_x = self.world_x
            self.last_y = self.world_y
            self.last_yaw = self.yaw

            robot_x, robot_y = pose_to_map(
                self.world_x,
                self.world_y,
            )

            for index in range(
                0,
                len(self.scan.ranges),
                3,
            ):
                distance = self.scan.ranges[index]

                if not math.isfinite(distance):
                    continue

                if distance < 0.10 or distance > 10.0:
                    continue

                laser_angle = (
                    self.scan.angle_min
                    + index * self.scan.angle_increment
                )
                world_angle = self.yaw + laser_angle

                hit_world_x = (
                    self.world_x
                    + distance * math.cos(world_angle)
                )
                hit_world_y = (
                    self.world_y
                    + distance * math.sin(world_angle)
                )

                hit_x, hit_y = pose_to_map(
                    hit_world_x,
                    hit_world_y,
                )

                line = bresenham(
                    robot_x,
                    robot_y,
                    hit_x,
                    hit_y,
                )

                for cell_x, cell_y in line[:-1]:
                    if (
                        0 <= cell_x < WIDTH
                        and 0 <= cell_y < HEIGHT
                    ):
                        self.log_map[
                            cell_y,
                            cell_x,
                        ] -= 0.4

                if (
                    0 <= hit_x < WIDTH
                    and 0 <= hit_y < HEIGHT
                ):
                    self.log_map[hit_y, hit_x] += 0.9

            np.clip(
                self.log_map,
                -5.0,
                5.0,
                out=self.log_map,
            )

            self.image_map.fill(128)
            self.image_map[self.log_map < -2.0] = 255
            self.image_map[self.log_map > 2.0] = 0

            self.publish_map()

        def publish_map(self):
            message = Image()

            message.header.stamp = (
                self.get_clock().now().to_msg()
            )
            message.header.frame_id = "odom"

            message.height = HEIGHT
            message.width = WIDTH
            message.encoding = "mono8"
            message.is_bigendian = False
            message.step = WIDTH
            message.data = self.image_map.tobytes()

            self.map_publisher.publish(message)

    node = MapPublisher()

    while True:
        rclpy.spin_once(node, timeout_sec=0.0)

        ready_wire = inputs.read_number("laser_ready")

        if (
            ready_wire is not None
            and float(ready_wire[0]) > 0.5
        ):
            node.update_map()

        synchronise()