import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
import threading

# ==========================================
# 1. ROS 2 NODE (Navigation Only)
# ==========================================
class TurtleBotController(Node):
    def __init__(self):
        super().__init__('turtlebot_lawnmower_only')
        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/turtlebot3/odom', self.odom_cb, 10)
        self.laser_sub = self.create_subscription(LaserScan, '/turtlebot3/laser/scan', self.laser_cb, 10)
        # Publisher
        self.vel_pub = self.create_publisher(Twist, '/turtlebot3/cmd_vel', 10)
        
        self.pose = [0.0, 0.0, 0.0] 
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

def normalize_angle(angle):
    while angle > math.pi: angle -= 2 * math.pi
    while angle < -math.pi: angle += 2 * math.pi
    return angle

# ==========================================
# 2. MAIN BLOCK
# ==========================================
def main(inputs, outputs, parameters, synchronise):
    if not rclpy.ok(): rclpy.init()
    
    bot = TurtleBotController()
    threading.Thread(target=rclpy.spin, args=(bot,), daemon=True).start()
    
    # FSM State
    state = "FORWARD"
    turn_dir = 1
    target_yaw = 0.0
    timer = 0
    
    # Tuning Settings
    FORWARD_SPEED = 0.4
    TURN_SPEED = 0.3
    WALL_DIST = 0.8
    SHIFT_TICKS = 100 

    print("ALL-IN-ONE NAVIGATION STARTED. No Display Required.")

    while rclpy.ok():
        x, y, yaw = bot.pose
        ranges = bot.laser_ranges
        
        v, w = 0.0, 0.0
        if ranges:
            # Simple front distance check
            front = min(list(ranges[0:20]) + list(ranges[-20:]) or [10.0])
            
            if state == "FORWARD":
                v, w = FORWARD_SPEED, 0.0
                if front < WALL_DIST:
                    state = "TURN_1"
                    target_yaw = normalize_angle(yaw + (turn_dir * math.radians(85)))
            
            elif state == "TURN_1":
                err = normalize_angle(target_yaw - yaw)
                v, w = 0.0, (TURN_SPEED if err > 0 else -TURN_SPEED)
                if abs(err) < 0.1:
                    state = "SHIFT"
                    timer = SHIFT_TICKS
            
            elif state == "SHIFT":
                v, w = FORWARD_SPEED, 0.0
                timer -= 1
                if timer <= 0:
                    state = "TURN_2"
                    target_yaw = normalize_angle(yaw + (turn_dir * math.radians(85)))
            
            elif state == "TURN_2":
                err = normalize_angle(target_yaw - yaw)
                v, w = 0.0, (TURN_SPEED if err > 0 else -TURN_SPEED)
                if abs(err) < 0.1:
                    state = "FORWARD"
                    turn_dir *= -1 

        bot.send_vel(v, w)
        synchronise()
