import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def main(inputs, outputs, parameters, synchronise):
    if not rclpy.ok():
        rclpy.init(args=None)

    class LaserDistanceBlock(Node):
        def __init__(self):
            super().__init__("vc_laser_distance_block")

            self.have_scan = False
            self.ranges = []
            self.angle_min = 0.0
            self.angle_increment = 0.0
            self.front = 10.0

            self.create_subscription(
                LaserScan,
                "/turtlebot3/laser/scan",
                self.scan_callback,
                qos_profile_sensor_data,
            )

        def scan_callback(self, msg):
            self.have_scan = True
            self.ranges = list(msg.ranges)
            self.angle_min = msg.angle_min
            self.angle_increment = msg.angle_increment

        def update_front_distance(self):
            if not self.have_scan:
                self.front = 10.0
                return

            values = []

            for i, dist in enumerate(self.ranges):
                angle = self.angle_min + i * self.angle_increment

                if -0.35 <= angle <= 0.35:
                    if math.isfinite(dist) and dist > 0.05:
                        values.append(dist)

            if values:
                self.front = min(values)
            else:
                self.front = 10.0

    node = LaserDistanceBlock()

    while True:
        rclpy.spin_once(node, timeout_sec=0.0)
        node.update_front_distance()

        outputs.share_number("front_distance", node.front)

        synchronise()