#!/usr/bin/env python3

import math
import rclpy

from rclpy.node import Node
from nav_msgs.msg import Odometry


robot_pose = None


class OdomSubscriber(Node):
    def __init__(self, topic):
        super().__init__("odom_subscriber")

        self.subscription = self.create_subscription(
            Odometry,
            topic,
            self.callback,
            10,
        )

    def callback(self, msg):
        global robot_pose

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        robot_pose = [float(x), float(y), float(yaw)]


def main(inputs, outputs, parameters, synchronise):
    global robot_pose

    auto_enable = False

    try:
        inputs.read_number("Enable")
    except Exception:
        auto_enable = True

    rclpy.init(args=None)

    odom_subscriber = OdomSubscriber(
        parameters.read_string("ROSTopic")
    )

    try:
        while auto_enable or inputs.read_number("Enable"):
            robot_pose = None

            rclpy.spin_once(
                odom_subscriber,
                timeout_sec=0.0,
            )

            if robot_pose is not None:
                outputs.share_array("Out", robot_pose)

            synchronise()

    except Exception as error:
        print("Odom block error:", error)

    finally:
        odom_subscriber.destroy_node()
        rclpy.shutdown()

        /odom

// block3 /webgui/current_target

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


        // block4 OdomIn,TargetIn   CarForce

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


        // block5. LaserIn  ObsForce

#!/usr/bin/env python3

import math


def safe_float(value):
    try:
        number = float(value)
        if math.isfinite(number):
            return number
    except Exception:
        pass

    return None


def clamp(value, minimum, maximum):
    return max(minimum, min(value, maximum))


def main(inputs, outputs, parameters, synchronise):
    filtered_x = 0.0
    filtered_y = 0.0

    obstacle_weight = 4.0
    max_force = 6.0
    alpha = 0.25

    while True:
        try:
            laser_data = inputs.read_array("LaserIn")

            obs_x = 0.0
            obs_y = 0.0

            if laser_data is not None:
                laser_count = len(laser_data)

                if laser_count > 1:
                    for index in range(laser_count):
                        dist = safe_float(laser_data[index])

                        if dist is None:
                            continue

                        if dist < 0.20 or dist > 3.0:
                            continue

                        angle = (
                            -math.pi / 2.0
                            + index * (math.pi / float(laser_count - 1))
                        )

                        strength = (3.0 - dist) / 3.0
                        repulsive_magnitude = strength * strength

                        obs_x -= repulsive_magnitude * math.cos(angle)
                        obs_y -= repulsive_magnitude * math.sin(angle)

                obs_x *= obstacle_weight
                obs_y *= obstacle_weight

                obs_x = clamp(obs_x, -max_force, max_force)
                obs_y = clamp(obs_y, -max_force, max_force)

                filtered_x = (1.0 - alpha) * filtered_x + alpha * obs_x
                filtered_y = (1.0 - alpha) * filtered_y + alpha * obs_y

            outputs.share_array("ObsForce", [filtered_x, filtered_y])

        except Exception as error:
            print("Obstacle Force error:", error)
            outputs.share_array("ObsForce", [0.0, 0.0])

        synchronise()


        // block 6 CarForce,ObsForce,TargetDistance   VelsOut
#!/usr/bin/env python3


def clamp(value, minimum, maximum):
    return max(minimum, min(value, maximum))


def main(inputs, outputs, parameters, synchronise):
    filtered_v = 0.0
    filtered_w = 0.0

    max_speed = 2.0
    min_speed = 0.6

    steering_gain = 0.35
    max_turn = 0.95

    steering_deadband = 0.35

    alpha_v = 0.20
    alpha_w = 0.35

    while True:
        try:
            car_force = inputs.read_array("CarForce")
            obs_force = inputs.read_array("ObsForce")
            distance_data = inputs.read_number("TargetDistance")

            target_v = 0.0
            target_w = 0.0

            if (
                car_force is not None
                and obs_force is not None
                and distance_data is not None
            ):
                avg_force_y = float(car_force[1]) + float(obs_force[1])
                distance_to_target = float(distance_data[0])

                if abs(avg_force_y) < steering_deadband:
                    avg_force_y = 0.0

                target_w = avg_force_y * steering_gain
                target_w = clamp(target_w, -max_turn, max_turn)

                turn_strength = abs(target_w)

                target_v = max_speed - 0.6 * turn_strength
                target_v = clamp(target_v, min_speed, max_speed)

                if distance_to_target < 1.0:
                    target_v = 0.0
                    target_w = 0.0

            filtered_v = (1.0 - alpha_v) * filtered_v + alpha_v * target_v
            filtered_w = (1.0 - alpha_w) * filtered_w + alpha_w * target_w

            outputs.share_array(
                "VelsOut",
                [
                    filtered_v,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    filtered_w,
                ],
            )

        except Exception as error:
            print("Velocity Mixer error:", error)
            outputs.share_array(
                "VelsOut",
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            )

        synchronise()


        