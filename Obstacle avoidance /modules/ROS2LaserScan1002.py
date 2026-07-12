#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

robot_pose = None

class OdomSubscriber(Node):
    def __init__(self, topic):
        super().__init__('odom_subscriber')
        self.subscription = self.create_subscription(Odometry, topic, self.callback, 10)

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
        enable = inputs.read_number('Enable')
    except Exception:
        auto_enable = True
    rclpy.init()
    odom_sub = OdomSubscriber(parameters.read_string("ROSTopic"))
    try:
        while auto_enable or inputs.read_number('Enable'):
            robot_pose = None
            rclpy.spin_once(odom_sub, timeout_sec=0)
            if robot_pose is not None:
                outputs.share_array("Out", robot_pose)   
            synchronise()  
    except Exception as e:
        pass
    finally:
        synchronise()     
        odom_sub.destroy_node()
        rclpy.shutdown()