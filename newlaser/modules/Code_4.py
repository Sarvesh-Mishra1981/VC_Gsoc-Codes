import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


def read_input(inputs, name, default=None):
    if hasattr(inputs, "read"):
        return inputs.read(name)
    if hasattr(inputs, "get"):
        return inputs.get(name, default)
    try:
        return inputs[name]
    except Exception:
        return getattr(inputs, name, default)


def main(inputs, outputs, parameters, synchronise):
    if not rclpy.ok():
        rclpy.init(args=None)

    class CmdVelBlock(Node):
        def __init__(self):
            super().__init__("vc_cmd_vel_publisher")
            self.pub = self.create_publisher(Twist, "/turtlebot3/cmd_vel", 10)

        def publish_cmd(self, linear_x, angular_z):
            msg = Twist()
            msg.linear.x = float(linear_x)
            msg.angular.z = float(angular_z)
            self.pub.publish(msg)

    node = CmdVelBlock()

    while True:
        linear_x = read_input(inputs, "linear_x", 0.0)
        angular_z = read_input(inputs, "angular_z", 0.0)

        if linear_x is None:
            linear_x = 0.0

        if angular_z is None:
            angular_z = 0.0

        node.publish_cmd(linear_x, angular_z)

        synchronise()