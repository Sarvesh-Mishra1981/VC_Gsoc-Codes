import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import math
import threading
import cv2

# ==========================================
# 1. ROS 2 NODE (Handles all Communication)
# ==========================================
class TurtleBotController(Node):
    def __init__(self):
        super().__init__('ultimate_lawnmower_node')
        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/turtlebot3/odom', self.odom_cb, 10)
        self.laser_sub = self.create_subscription(LaserScan, '/turtlebot3/laser/scan', self.laser_cb, 10)
        # Publisher
        self.vel_pub = self.create_publisher(Twist, '/turtlebot3/cmd_vel', 10)
        
        # Internal Data
        self.pose = [0.0, 0.0, 0.0] # x, y, yaw
        self.laser_ranges = []
        
    def odom_cb(self, msg):
        self.pose[0] = msg.pose.pose.position.x
        self.pose[1] = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.pose[2] = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0-2.0*(q.y*q.y + q.z*q.z))

    def laser_cb(self, msg):
        self.laser_ranges = msg.ranges

    def send_vel(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.vel_pub.publish(msg)

# ==========================================
# 2. MAPPING LOGIC
# ==========================================
class RobotMapper:
    def __init__(self):
        self.l_map = np.zeros((600, 600), dtype=np.float32)
        self.img = np.full((600, 600), 128, dtype=np.uint8)

    def update(self, x, y, yaw, ranges):
        if not ranges: return self.img
        # Simple map: 1 meter = 40 pixels, center is 300,300
        rx, ry = int(300 + x*40), int(300 - y*40)
        angles = np.linspace(-math.pi/2, math.pi/2, len(ranges))
        
        for i in range(0, len(ranges), 10):
            dist = ranges[i]
            if dist > 3.5 or dist < 0.1: continue
            angle = yaw + angles[i]
            hx, hy = int(300 + (x + dist*math.cos(angle))*40), int(300 - (y + dist*math.sin(angle))*40)
            if 0 <= hx < 600 and 0 <= hy < 600:
                self.l_map[hy, hx] += 1.0 # Occupied
        
        np.clip(self.l_map, 0, 5, out=self.l_map)
        self.img[self.l_map > 1.0] = 0
        self.img[self.l_map <= 1.0] = 255
        return self.img

# ==========================================
# 3. MAIN BLOCK FUNCTION
# ==========================================
def main(inputs, outputs, parameters, synchronise):
    if not rclpy.ok(): rclpy.init()
    
    bot = TurtleBotController()
    mapper = RobotMapper()
    
    # Run ROS 2 in background
    threading.Thread(target=rclpy.spin, args=(bot,), daemon=True).start()
    
    # FSM State
    state = "FORWARD"
    turn_dir = 1
    target_yaw = 0.0
    timer = 0
    
    print("ALL-IN-ONE BLOCK STARTED. Driving TurtleBot3...")

    while rclpy.ok():
        x, y, yaw = bot.pose
        ranges = bot.laser_ranges
        
        # --- NAVIGATION ---
        v, w = 0.0, 0.0
        if ranges:
            front = min(list(ranges[0:20]) + list(ranges[-20:]) or [10.0])
            
            if state == "FORWARD":
                v, w = 0.3, 0.0
                if front < 0.7:
                    state = "TURN"
                    target_yaw = yaw + (turn_dir * math.radians(90))
            
            elif state == "TURN":
                err = target_yaw - yaw
                while err > math.pi: err -= 2*math.pi
                while err < -math.pi: err += 2*math.pi
                v, w = 0.0, (0.3 if err > 0 else -0.3)
                if abs(err) < 0.1:
                    state = "SHIFT"
                    timer = 60 # Move forward for 60 ticks
            
            elif state == "SHIFT":
                v, w = 0.3, 0.0
                timer -= 1
                if timer <= 0:
                    state = "FORWARD"
                    turn_dir *= -1 # Reverse turn for next wall
        
        bot.send_vel(v, w)
        
        # --- MAPPING ---
        map_img = mapper.update(x, y, yaw, ranges)
        # We use standard CV2 to show the map directly since we have no wires
        cv2.imshow("Live Lawnmower Map", map_img)
        cv2.waitKey(1)

        synchronise()
