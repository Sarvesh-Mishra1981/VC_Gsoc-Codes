#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile, DurabilityPolicy


current_target = None


class TargetSubscriber(Node):
    def __init__(self, topic):
        super().__init__("target_subscriber")

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.subscription = self.create_subscription(
            Point,
            topic,
            self.callback,
            qos,
        )

    def callback(self, msg):
        global current_target

        current_target = [
            float(msg.x),
            float(msg.y),
        ]


def main(inputs, outputs, parameters, synchronise):
    global current_target

    auto_enable = False

    try:
        inputs.read_number("Enable")
    except Exception:
        auto_enable = True

    rclpy.init(args=None)

    target_subscriber = TargetSubscriber(
        parameters.read_string("ROSTopic")
    )

    try:
        while auto_enable or inputs.read_number("Enable"):
            current_target = None

            rclpy.spin_once(
                target_subscriber,
                timeout_sec=0.0,
            )

            if current_target is not None:
                outputs.share_array("Out", current_target)

            synchronise()

    except Exception as error:
        print("Target block error:", error)

    finally:
        target_subscriber.destroy_node()
        rclpy.shutdown()