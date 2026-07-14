#!/usr/bin/env python3

import math
import rclpy

from rclpy.node import Node
from std_msgs.msg import Bool


class TargetReachedPublisher(Node):
    def __init__(self):
        super().__init__("target_force_block")

        self.publisher = self.create_publisher(
            Bool,
            "/webgui/target_reached",
            1,
        )

    def mark_reached(self):
        msg = Bool()
        msg.data = True
        self.publisher.publish(msg)


def absolute2relative(x_abs, y_abs, robotx, roboty, robott):
    dx = x_abs - robotx
    dy = y_abs - roboty

    x_rel = dx * math.cos(-robott) - dy * math.sin(-robott)
    y_rel = dx * math.sin(-robott) + dy * math.cos(-robott)

    return x_rel, y_rel


def main(inputs, outputs, parameters, synchronise):
    if not rclpy.ok():
        rclpy.init(args=None)

    node = TargetReachedPublisher()

    last_car_force = [0.0, 0.0]
    last_distance = 999.0

    while True:
        try:
            odom_data = inputs.read_array("OdomIn")
            target_data = inputs.read_array("TargetIn")

            if odom_data is not None and target_data is not None:
                robotx = float(odom_data[0])
                roboty = float(odom_data[1])
                robott = float(odom_data[2])

                target_abs_x = float(target_data[0])
                target_abs_y = float(target_data[1])

                target_rel_x, target_rel_y = absolute2relative(
                    target_abs_x,
                    target_abs_y,
                    robotx,
                    roboty,
                    robott,
                )

                distance = math.hypot(
                    target_rel_x,
                    target_rel_y,
                )

                target_weight = 4.0

                if distance > 0.001:
                    car_force_x = (
                        target_rel_x / distance
                    ) * target_weight

                    car_force_y = (
                        target_rel_y / distance
                    ) * target_weight
                else:
                    car_force_x = 0.0
                    car_force_y = 0.0

                if distance < 2.5:
                    node.mark_reached()

                last_car_force = [
                    car_force_x,
                    car_force_y,
                ]

                last_distance = distance

            outputs.share_array("CarForce", last_car_force)
            outputs.share_number("TargetDistance", last_distance)

            rclpy.spin_once(node, timeout_sec=0.0)

        except Exception as error:
            print("Target Force error:", error)
            outputs.share_array("CarForce", [0.0, 0.0])
            outputs.share_number("TargetDistance", 999.0)

        synchronise()