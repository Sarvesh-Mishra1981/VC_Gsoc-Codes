import math
import rclpy

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def main(inputs, outputs, parameters, synchronise):
    if not rclpy.ok():
        rclpy.init(args=None)

    class LaserReader(Node):
        def __init__(self):
            super().__init__("vc_laser_reader")

            self.front_distance = 10.0
            self.laser_ready = False

            self.create_subscription(
                LaserScan,
                "/turtlebot3/laser/scan",
                self.scan_callback,
                qos_profile_sensor_data,
            )

        def scan_callback(self, msg):
            values = []

            for index, distance in enumerate(msg.ranges):
                angle = msg.angle_min + index * msg.angle_increment

                if -0.35 <= angle <= 0.35:
                    if math.isfinite(distance) and distance > 0.05:
                        values.append(distance)

            self.front_distance = min(values) if values else 10.0
            self.laser_ready = True

    node = LaserReader()

    while True:
        rclpy.spin_once(node, timeout_sec=0.0)

        outputs.share_number(
            "front_distance",
            node.front_distance,
        )
        outputs.share_number(
            "laser_ready",
            1.0 if node.laser_ready else 0.0,
        )

        synchronise()