import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def write_output(outputs, name, value):
    if hasattr(outputs, "write"):
        outputs.write(name, value)
        return
    if hasattr(outputs, "set"):
        outputs.set(name, value)
        return
    try:
        outputs[name] = value
        return
    except Exception:
        pass
    setattr(outputs, name, value)


def main(inputs, outputs, parameters, synchronise):
    if not rclpy.ok():
        rclpy.init(args=None)

    class LaserBlock(Node):
        def __init__(self):
            super().__init__("vc_laser_scan_reader")

            self.laser_ready = False
            self.ranges = []
            self.angle_min = 0.0
            self.angle_increment = 0.0

            self.create_subscription(
                LaserScan,
                "/turtlebot3/laser/scan",
                self.scan_callback,
                qos_profile_sensor_data,
            )

        def scan_callback(self, msg):
            self.laser_ready = True
            self.ranges = list(msg.ranges)
            self.angle_min = msg.angle_min
            self.angle_increment = msg.angle_increment

    node = LaserBlock()

    while True:
        rclpy.spin_once(node, timeout_sec=0.0)

        write_output(outputs, "laser_ready", node.laser_ready)
        write_output(outputs, "ranges", node.ranges)
        write_output(outputs, "angle_min", node.angle_min)
        write_output(outputs, "angle_increment", node.angle_increment)

        synchronise()