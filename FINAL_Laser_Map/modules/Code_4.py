import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


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
        linear_wire = inputs.read_number("linear_x")
        angular_wire = inputs.read_number("angular_z")

        if linear_wire is None:
            linear_x = 0.0
        else:
            linear_x = float(linear_wire[0])

        if angular_wire is None:
            angular_z = 0.0
        else:
            angular_z = float(angular_wire[0])

        node.publish_cmd(linear_x, angular_z)

        synchronise()