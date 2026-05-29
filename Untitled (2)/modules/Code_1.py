import sys
import numpy as np
import cv2
import math
from math import sqrt, atan2, pi
import threading
import os

# Pure ROS 2 Imports
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

# OMPL Imports
from ompl import base as ob
from ompl import geometric as og

# Global ROS 2 Node so we don't initialize multiple times
_amazon_node = None
_rclpy_initialized = False

class AmazonRobotNode(Node):
    def __init__(self):
        super().__init__('amazon_vc_node')
        
        # Exact topics from 'ros2 topic list'
        self.cmd_vel_pub = self.create_publisher(Twist, '/amazon_robot/cmd_vel', 10)
        self.elevator_pub = self.create_publisher(Float64, '/platform/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/amazon_robot/odom', self.odom_callback, 10)
        
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def set_v(self, v):
        msg = Twist()
        msg.linear.x = float(v)
        self.cmd_vel_pub.publish(msg)

    def set_w(self, w):
        msg = Twist()
        msg.angular.z = float(w)
        self.cmd_vel_pub.publish(msg)

    def lift(self):
        msg = Float64()
        msg.data = 0.5  # Positive Z velocity to lift
        self.elevator_pub.publish(msg)

    def putdown(self):
        msg = Float64()
        msg.data = -0.5  # Negative Z velocity to drop
        self.elevator_pub.publish(msg)

def get_ros_node():
    global _amazon_node, _rclpy_initialized
    if not _rclpy_initialized:
        rclpy.init(args=None)
        _rclpy_initialized = True
    if _amazon_node is None:
        _amazon_node = AmazonRobotNode()
        # Run rclpy.spin in a background thread so the block loop doesn't freeze
        spin_thread = threading.Thread(target=rclpy.spin, args=(_amazon_node,), daemon=True)
        spin_thread.start()
    return _amazon_node


def main(inputs, outputs, parameters, synchronise):
    # Auto-enable for VisualCircuit
    auto_enable = False
    try:
        enable = inputs.read_number("Enable")
    except Exception:
        auto_enable = True

    # 1. Initialize our Pure ROS 2 node
    robot = get_ros_node()

    # 2. CALIBRATION & CONSTANTS
    SHELF_OFFSET_X = 0.20  
    SHELF_OFFSET_Y = 0.00  
    MAP_OFFSET_X = 0.0
    MAP_OFFSET_Y = 0.0
    WORLD_WIDTH = 20.62
    WORLD_HEIGHT = 13.6

    SHELF_BASE_X = 3.728
    SHELF_BASE_Y = 0.579
    SHELF_POS = (SHELF_BASE_X + SHELF_OFFSET_X, SHELF_BASE_Y + SHELF_OFFSET_Y)
    DROP_POS = (0.0, 0.0)
    DOCKING_OFFSET = 1.0
    SHELF_DOCK_POS = (SHELF_POS[0] - DOCKING_OFFSET, SHELF_POS[1])

    STATE_PLAN_TO_SHELF = 0
    STATE_MOVING_TO_SHELF = 1
    STATE_ALIGN_180_PRE_LIFT = 2  
    STATE_LIFTING = 3
    STATE_PLAN_TO_HOME = 4
    STATE_MOVING_HOME = 5
    STATE_DROPPING = 6
    STATE_FINISHED = 7

    # 3. HELPER CLASSES
    class MapProcessor:
        def __init__(self):
            # Safe image loading
            possible_paths = [
                '/resources/exercises/amazon_warehouse/images/map.png',
                '/opt/jderobot/resources/exercises/amazon_warehouse/images/map.png',
                'map.png'
            ]
            self.raw_map = None
            for path in possible_paths:
                if os.path.exists(path):
                    self.raw_map = cv2.imread(path)
                    break
            
            if self.raw_map is None:
                self.raw_map = np.ones((800, 1200, 3), dtype=np.uint8) * 255

            self.grid_map = self.raw_map.copy()
            if len(self.grid_map.shape) > 2:
                self.grid_map = cv2.cvtColor(self.grid_map, cv2.COLOR_BGR2GRAY)

            self.height, self.width = self.grid_map.shape
            self.scale_x = self.width / WORLD_WIDTH
            self.scale_y = self.height / WORLD_HEIGHT
           
            _, self.cost_map = cv2.threshold(self.grid_map, 127, 255, cv2.THRESH_BINARY)
            kernel_size = int(0.4 * self.scale_x)
            kernel = np.ones((kernel_size, kernel_size), np.uint8)
            self.cost_map = cv2.erode(self.cost_map, kernel, iterations=1)

        def world_to_grid(self, x, y):
            adj_x = x + MAP_OFFSET_X
            adj_y = y + MAP_OFFSET_Y
            px = int((self.width / 2) + (adj_x * self.scale_x))
            py = int((self.height / 2) - (adj_y * self.scale_y))
            px = max(0, min(self.width - 1, px))
            py = max(0, min(self.height - 1, py))
            return px, py

        def is_valid(self, x, y):
            px, py = self.world_to_grid(x, y)
            return self.cost_map[py, px] > 128

    class CustomStateValidityChecker(ob.StateValidityChecker):
        def __init__(self, space_information, map_proc):
            super(CustomStateValidityChecker, self).__init__(space_information)
            self.map_proc = map_proc

        def isValid(self, state):
            return bool(self.map_proc.is_valid(state[0], state[1]))

    class OMPLPlanner:
        def __init__(self, map_proc):
            self.map = map_proc
            self.space = ob.RealVectorStateSpace(2)
            bounds = ob.RealVectorBounds(2)
            bounds.setLow(0, -WORLD_WIDTH/2)
            bounds.setHigh(0, WORLD_WIDTH/2)
            bounds.setLow(1, -WORLD_HEIGHT/2)
            bounds.setHigh(1, WORLD_HEIGHT/2)
            self.space.setBounds(bounds)
            
            # OMPL 2.0.0 FIX: Use SimpleSetup to avoid C++ bad_cast
            self.ss = og.SimpleSetup(self.space)
            self.checker = CustomStateValidityChecker(self.ss.getSpaceInformation(), self.map)
            self.ss.setStateValidityChecker(self.checker)

        def get_path(self, start_x, start_y, goal_x, goal_y):
            if not self.map.is_valid(start_x, start_y) or not self.map.is_valid(goal_x, goal_y):
                print("ERROR: Start or Goal is inside an obstacle.")
                return []

            # Clear memory from previous runs
            self.ss.clear()

            start = ob.State(self.space)
            start[0] = start_x
            start[1] = start_y
            
            goal = ob.State(self.space)
            goal[0] = goal_x
            goal[1] = goal_y
           
            # SimpleSetup perfectly handles C++ casting internally
            self.ss.setStartAndGoalStates(start, goal)
           
            if self.ss.solve(1.0):
                path = self.ss.getSolutionPath()
                ps = og.PathSimplifier(self.ss.getSpaceInformation())
                ps.simplify(path, 1.0)
                path.interpolate(50)
                return [(s[0], s[1]) for s in path.getStates()]
            return []

    # 4. STARTUP
    mapper = MapProcessor()
    planner = OMPLPlanner(mapper)

    current_state = STATE_PLAN_TO_SHELF
    active_path = []
    path_index = 0
    wait_timer = 0

    print("VisualCircuit Pure ROS 2 Block Started!")

    # 5. MAIN LOOP
    while (auto_enable or inputs.read_number('Enable')) and rclpy.ok():
        
        # VISUAL DEBUGGING
        debug_img = np.copy(mapper.raw_map)
        px, py = mapper.world_to_grid(robot.x, robot.y)
        cv2.circle(debug_img, (px, py), 10, (0, 0, 255), -1) 
        
        if len(active_path) > 0:
            for pt in active_path:
                ppx, ppy = mapper.world_to_grid(pt[0], pt[1])
                cv2.circle(debug_img, (ppx, ppy), 2, (255, 0, 0), -1)

        # Output the debug image to the block's output pin
        try:
            outputs.share_image("MapDebug", debug_img)
        except Exception:
            pass

        # STATE MACHINE
        if current_state == STATE_PLAN_TO_SHELF:
            path_to_dock = planner.get_path(robot.x, robot.y, SHELF_DOCK_POS[0], SHELF_DOCK_POS[1])
            if path_to_dock:
                active_path = path_to_dock
                active_path.append(SHELF_POS)
                path_index = 0
                current_state = STATE_MOVING_TO_SHELF

        elif current_state == STATE_MOVING_TO_SHELF or current_state == STATE_MOVING_HOME:
            if path_index < len(active_path):
                target = active_path[path_index]
                dx = target[0] - robot.x
                dy = target[1] - robot.y
                dist = sqrt(dx**2 + dy**2)
                is_last_point = (path_index == len(active_path) - 1)
               
                if current_state == STATE_MOVING_HOME:
                    speed_gain = 0.2; max_speed = 0.25; turn_gain = 0.2; max_turn = 0.15
                    acceptance_radius = 0.1
                else:
                    speed_gain = 0.4; max_speed = 0.6; turn_gain = 0.15; max_turn = 0.15
                    acceptance_radius = 0.01 if is_last_point else 0.2

                target_yaw = atan2(dy, dx)
                yaw_err = target_yaw - robot.yaw
                while yaw_err > pi: yaw_err -= 2*pi
                while yaw_err < -pi: yaw_err += 2*pi

                linear_v = speed_gain * dist
                angular_w = turn_gain * yaw_err
               
                if current_state == STATE_MOVING_TO_SHELF and is_last_point:
                    if abs(yaw_err) < 0.2: angular_w = 0.0
                    if dist < 0.25: linear_v = 0.04

                if abs(yaw_err) > 0.3: linear_v = 0.0
                linear_v = max(0.0, min(max_speed, linear_v))
                angular_w = max(-max_turn, min(max_turn, angular_w))

                robot.set_v(linear_v)
                robot.set_w(angular_w)

                if dist < acceptance_radius:
                    path_index += 1
            else:
                robot.set_v(0)
                robot.set_w(0)
                if current_state == STATE_MOVING_TO_SHELF:
                    for _ in range(30): synchronise()
                    current_state = STATE_ALIGN_180_PRE_LIFT
                else:
                    current_state = STATE_DROPPING

        elif current_state == STATE_ALIGN_180_PRE_LIFT:
            dx = DROP_POS[0] - robot.x
            dy = DROP_POS[1] - robot.y
            target_yaw = atan2(dy, dx)
           
            yaw_err = target_yaw - robot.yaw
            while yaw_err > pi: yaw_err -= 2*pi
            while yaw_err < -pi: yaw_err += 2*pi
           
            robot.set_v(0)
            rot_speed = 0.3 * yaw_err
            robot.set_w(max(-0.3, min(0.3, rot_speed)))
           
            if abs(yaw_err) < 0.05:
                robot.set_w(0)
                for _ in range(10): synchronise() 
                current_state = STATE_LIFTING

        elif current_state == STATE_LIFTING:
            robot.lift()
            wait_timer += 1
            if wait_timer > 60:
                stop_msg = Float64()
                stop_msg.data = 0.0
                robot.elevator_pub.publish(stop_msg) # Stop lifting
                current_state = STATE_PLAN_TO_HOME
                wait_timer = 0

        elif current_state == STATE_PLAN_TO_HOME:
            active_path = planner.get_path(robot.x, robot.y, DROP_POS[0], DROP_POS[1])
            if active_path:
                path_index = 0
                current_state = STATE_MOVING_HOME

        elif current_state == STATE_DROPPING:
            robot.putdown()
            wait_timer += 1
            if wait_timer > 60:
                stop_msg = Float64()
                stop_msg.data = 0.0
                robot.elevator_pub.publish(stop_msg) # Stop dropping
                current_state = STATE_FINISHED
                wait_timer = 0

        elif current_state == STATE_FINISHED:
            robot.set_v(0)
            robot.set_w(0)

        # Critical: Required for VisualCircuit Execution
        synchronise()
