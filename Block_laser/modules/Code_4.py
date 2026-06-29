import rclpy

from rclpy.node import Node
from geometry_msgs.msg import Twist


def main(inputs, outputs, parameters, synchronise):
    if not rclpy.ok():
        rclpy.init(args=None)

    class VelocityPublisher(Node):
        def __init__(self):
            super().__init__("vc_velocity_publisher")

            self.publisher = self.create_publisher(
                Twist,
                "/turtlebot3/cmd_vel",
                10,
            )

        def send_velocity(self, linear_x, angular_z):
            command = Twist()
            command.linear.x = float(linear_x)
            command.angular.z = float(angular_z)
            self.publisher.publish(command)

    node = VelocityPublisher()

    while True:
        linear_wire = inputs.read_number("linear_x")
        angular_wire = inputs.read_number("angular_z")

        linear_x = (
            float(linear_wire[0])
            if linear_wire is not None
            else 0.0
        )
        angular_z = (
            float(angular_wire[0])
            if angular_wire is not None
            else 0.0
        )

        node.send_velocity(linear_x, angular_z)
        synchronise()