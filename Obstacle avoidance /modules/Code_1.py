#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class VFFBrain(Node):
    def __init__(self):
        super().__init__('vff_brain')
        self.reached_pub = self.create_publisher(Bool, '/webgui/target_reached', 1)
    def mark_reached(self):
        msg = Bool()
        msg.data = True
        self.reached_pub.publish(msg)

def absolute2relative(x_abs, y_abs, robotx, roboty, robott):
    dx = x_abs - robotx
    dy = y_abs - roboty
    x_rel = dx * math.cos(-robott) - dy * math.sin(-robott)
    y_rel = dx * math.sin(-robott) + dy * math.cos(-robott)
    return x_rel, y_rel

def main(inputs, outputs, parameters, synchronise):
    auto_enable = False
    try:
        enable = inputs.read_number('Enable')
    except Exception:
        auto_enable = True

    rclpy.init()
    brain_node = VFFBrain()

    try:
        while auto_enable or inputs.read_number('Enable'):
            laser_data = inputs.read_array('LaserIn')
            odom_data = inputs.read_array('OdomIn')
            target_data = inputs.read_array('TargetIn')

            if laser_data is not None and odom_data is not None and target_data is not None:
                robotx, roboty, robott = odom_data[0], odom_data[1], odom_data[2]
                target_abs_x, target_abs_y = target_data[0], target_data[1]

                target_rel_x, target_rel_y = absolute2relative(target_abs_x, target_abs_y, robotx, roboty, robott)
                
                distance_to_target = math.sqrt(target_rel_x**2 + target_rel_y**2)
                if distance_to_target < 2.5:
                    brain_node.mark_reached()

                target_weight = 4.0
                if distance_to_target > 0:
                    carForce = [(target_rel_x / distance_to_target) * target_weight, 
                                (target_rel_y / distance_to_target) * target_weight]
                else:
                    carForce = [0.0, 0.0]

                obs_x = 0.0
                obs_y = 0.0
                
                for i in range(len(laser_data)):
                    dist = float(laser_data[i])
                    if dist < 3.0:
                        angle = math.radians(i - 90)
                        repulsive_magnitude = 1.0 / (dist ** 2)
                        obs_x -= repulsive_magnitude * math.cos(angle)
                        obs_y -= repulsive_magnitude * math.sin(angle)

                obstacle_weight = 12.0
                obsForce = [obs_x * obstacle_weight, obs_y * obstacle_weight]

                avgForce = [carForce[0] + obsForce[0], carForce[1] + obsForce[1]]

                v = 3.0
                w = avgForce[1] * 0.35
                
                if w > 1.2: w = 1.2
                if w < -1.2: w = -1.2

                outputs.share_array("VelsOut", [v, 0.0, 0.0, 0.0, 0.0, w])

            rclpy.spin_once(brain_node, timeout_sec=0)
            synchronise()  
    except Exception as e:
        pass
    finally:
        synchronise()     
        brain_node.destroy_node()
        rclpy.shutdown()