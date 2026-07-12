#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

velocities = 0

class VelPublisher(Node):
    def __init__(self, topic):
        super().__init__('vel_publisher')
        self.publisher_ = self.create_publisher(Twist, topic, 1)
        timer_period = 0.1  # Fast updates
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        global velocities
        msg = Twist()
        try:
            msg.linear.x = float(velocities[0])
            msg.linear.y = float(velocities[1])
            msg.linear.z = float(velocities[2])
            msg.angular.x = float(velocities[3])
            msg.angular.y = float(velocities[4])
            msg.angular.z = float(velocities[5])
        except (IndexError, TypeError):
            return
        self.publisher_.publish(msg)

def main(inputs, outputs, parameters, synchronise):
    global velocities
    auto_enable = False
    try:
        enable = inputs.read_number('Enable')
    except Exception:
        auto_enable = True

    rclpy.init()
    vel_publisher = VelPublisher(parameters.read_string('ROSTopic'))

    try:
        while auto_enable or inputs.read_number('Enable'):
            v_input = inputs.read_array('Vels')
            if v_input is not None:
                velocities = v_input
                rclpy.spin_once(vel_publisher, timeout_sec=0) 
            synchronise()   
    except Exception as e:
        pass
    finally:
        synchronise()
        vel_publisher.destroy_node()
        rclpy.shutdown()