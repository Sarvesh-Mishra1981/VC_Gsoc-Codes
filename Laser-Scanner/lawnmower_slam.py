#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.exceptions import ParameterAlreadyDeclaredException

def normalize_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a

def yaw_from_quat(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )

class LawnmowerBot(Node):
    def __init__(self):
        super().__init__("lawnmower_slam")

        # Safely force use_sim_time to False by default.
        # This prevents the 20Hz control timer from freezing when Gazebo is not bridging the /clock topic.
        try:
            self.declare_parameter("use_sim_time", False)
        except ParameterAlreadyDeclaredException:
            pass

        self.cmd_topic = "/turtlebot3/cmd_vel"
        self.odom_topic = "/turtlebot3/odom"
        self.scan_topic = "/turtlebot3/laser/scan"

        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)

        # CRITICAL FIX: Changed odom QoS from Reliable (10) to qos_profile_sensor_data.
        # Gazebo parameters bridge publishes odom in Best Effort mode; subscribing with Reliable
        # causes ROS 2 to silently drop the connection (you get have_odom=False forever!).
        self.create_subscription(
            Odometry, 
            self.odom_topic, 
            self.odom_callback, 
            qos_profile_sensor_data
        )

        # LaserScan uses best-effort sensor QoS
        self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            qos_profile_sensor_data,
        )

        self.have_odom = False
        self.have_scan = False

        self.yaw = 0.0
        self.ranges = []
        self.angle_min = 0.0
        self.angle_increment = 0.0

        # State machine settings
        # Start in INITIAL_TURN state so the robot turns 90 degrees left/right upon startup first!
        self.state = "INITIAL_TURN"
        self.init_yaw_set = False
        
        self.turn_dir = 1
        self.target_yaw = 0.0
        self.shift_count = 0

        self.forward_speed = 0.22
        self.turn_speed = 0.45
        self.wall_distance = 0.60
        self.shift_ticks = 45

        # Timer running at 20Hz (every 0.05s)
        self.create_timer(0.05, self.control_loop)
        # Debug running every 2.0s
        self.create_timer(2.0, self.debug_status)

        self.get_logger().info("Started lawnmower bot")
        self.get_logger().info(f"cmd_vel: {self.cmd_topic}")
        self.get_logger().info(f"odom: {self.odom_topic} (QoS Optimized)")
        self.get_logger().info(f"scan: {self.scan_topic}")

    def odom_callback(self, msg):
        self.have_odom = True
        self.yaw = yaw_from_quat(msg.pose.pose.orientation)

    def scan_callback(self, msg):
        self.have_scan = True
        self.ranges = list(msg.ranges)
        self.angle_min = msg.angle_min
        self.angle_increment = msg.angle_increment

    def debug_status(self):
        self.get_logger().info(
            f"have_odom={self.have_odom}, have_scan={self.have_scan}, state={self.state}"
        )

        if not self.have_odom:
            self.get_logger().warn("No odom received from /turtlebot3/odom - Check Gazebo & QoS Bridge!")

        if not self.have_scan:
            self.get_logger().warn("No scan received from /turtlebot3/laser/scan - Check Gazebo & QoS Bridge!")

    def get_front_distance(self):
        front = []

        for i, dist in enumerate(self.ranges):
            angle = self.angle_min + i * self.angle_increment

            if -0.35 <= angle <= 0.35:
                if math.isfinite(dist) and dist > 0.05:
                    front.append(dist)

        if not front:
            return 10.0

        return min(front)

    def turn_control(self):
        err = normalize_angle(self.target_yaw - self.yaw)

        if abs(err) < 0.08:
            return True, 0.0

        w = 2.0 * err

        if w > self.turn_speed:
            w = self.turn_speed
        if w < -self.turn_speed:
            w = -self.turn_speed

        if abs(w) < 0.20:
            w = 0.20 if w > 0 else -0.20

        return False, w

    def control_loop(self):
        twist = Twist()

        # Stop and wait if sensor topics aren't receiving data
        if not self.have_odom or not self.have_scan:
            self.cmd_pub.publish(twist)
            return

        front = self.get_front_distance()

        # 1. INITIAL STARTUP 90 DEGREE TURN
        if self.state == "INITIAL_TURN":
            if not self.init_yaw_set:
                self.target_yaw = normalize_angle(self.yaw + self.turn_dir * math.radians(90))
                self.init_yaw_set = True
                self.get_logger().info(f"Startup: Initiating initial turn to {self.target_yaw:.2f} rad first.")
            
            done, w = self.turn_control()
            twist.angular.z = w

            if done:
                self.state = "FORWARD"
                self.get_logger().info("Initial startup turn complete. Now proceeding to FORWARD sweep.")

        # 2. STANDARD SWEEPING STATES
        elif self.state == "FORWARD":
            twist.linear.x = self.forward_speed

            if front < self.wall_distance:
                twist.linear.x = 0.0
                self.target_yaw = normalize_angle(
                    self.yaw + self.turn_dir * math.radians(90)
                )
                self.state = "TURN_1"
                self.get_logger().info(f"Obstacle at {front:.2f} m. Turning.")

        elif self.state == "TURN_1":
            done, w = self.turn_control()
            twist.angular.z = w

            if done:
                self.shift_count = self.shift_ticks
                self.state = "SHIFT"
                self.get_logger().info("Turn 1 done. Lane shift.")

        elif self.state == "SHIFT":
            twist.linear.x = self.forward_speed
            self.shift_count -= 1

            if front < self.wall_distance or self.shift_count <= 0:
                twist.linear.x = 0.0
                self.target_yaw = normalize_angle(
                    self.yaw + self.turn_dir * math.radians(90)
                )
                self.state = "TURN_2"
                self.get_logger().info("Starting turn 2.")

        elif self.state == "TURN_2":
            done, w = self.turn_control()
            twist.angular.z = w

            if done:
                self.turn_dir *= -1
                self.state = "FORWARD"
                self.get_logger().info("Turn 2 done. Moving forward.")

        self.cmd_pub.publish(twist)

    def stop(self):
        self.cmd_pub.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = LawnmowerBot()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
