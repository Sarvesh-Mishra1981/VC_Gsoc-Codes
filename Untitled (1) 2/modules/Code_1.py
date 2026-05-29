import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


def main(inputs, outputs, parameters, synchronise):
    if not rclpy.ok():
        rclpy.init(args=None)

    class ForceForward(Node):
        def __init__(self):
            super().__init__("vc_force_forward_test")
            self.pub = self.create_publisher(Twist, "/turtlebot3/cmd_vel", 10)

        def publish_cmd(self):
            msg = Twist()
            msg.linear.x = 0.20
            msg.angular.z = 0.0
            self.pub.publish(msg)

    node = ForceForward()

    while True:
        node.publish_cmd()
        rclpy.spin_once(node, timeout_sec=0.0)
        synchronise()