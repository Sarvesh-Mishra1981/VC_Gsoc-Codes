import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math
import threading

# --- Global storage ---
x, y, yaw = 0.0, 0.0, 0.0

def get_yaw(q):
    """Convert Quaternion to Yaw"""
    t3 = +2.0 * (q.w * q.z + q.x * q.y)
    t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(t3, t4)

class OdomNode(Node):
    def __init__(self, topic):
        super().__init__('vc_odom_node')
        self.sub = self.create_subscription(Odometry, topic, self.callback, 10)
    def callback(self, msg):
        global x, y, yaw
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = get_yaw(msg.pose.pose.orientation)

def main(inputs, outputs, parameters, synchronise):
    if not rclpy.ok(): rclpy.init()
    
    # You can change the topic here if needed
    node = OdomNode("/turtlebot3/odom")
    
    # Run ROS 2 spin in background
    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()

    while rclpy.ok():
        # Output the pose array that the Brain block expects
        outputs.write_array("Odom", [x, y, 0.0, 0.0, 0.0, yaw])
        synchronise()
