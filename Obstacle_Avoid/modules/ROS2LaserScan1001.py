#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import LaserScan

bridge = CvBridge()
measure = None

# ros2 node class
class LaserSubscriber(Node):

    def __init__(self, topic):

        super().__init__('laser_subscriber')

        self.subscription = self.create_subscription(
            LaserScan, topic, self.callback, 10)

        self.subscription  # prevent unused variable warning


    def callback(self, msg):

        global measure

        measure = []
        for i in range(len(msg.ranges)):
            measure.extend((str(msg.ranges[i]),))



def main(inputs, outputs, parameters, synchronise):
    
    global measure

    auto_enable = False
    try:
        enable = inputs.read_number('Enable')
    except Exception:
        auto_enable = True

    rclpy.init()
    laser_subscriber = LaserSubscriber(parameters.read_string("ROSTopic"))

    try:
        while auto_enable or inputs.read_number('Enable'):
            measure = None
            rclpy.spin_once(laser_subscriber)

            if measure is not None:
                outputs.share_array("Out",measure)   

            synchronise()  
    except Exception as e:
        print('Error:', e)
        pass
    finally:
        print("Exiting")
        synchronise()     
        laser_subscriber.destroy_node()
        rclpy.shutdown()
