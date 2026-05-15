import numpy as np
import math

# ==========================================
# 1. HELPER CLASSES & FUNCTIONS
# ==========================================
class RobotMapper:
    def __init__(self):
        # Local map storage
        self.l_map = np.zeros((970, 1500), dtype=np.float32)
        self.image_map = np.full((970, 1500), 128, dtype=np.uint8)
        self.laser_angles = np.linspace(-math.pi/2, math.pi/2, 360)
        
    def bresenham_line(self, x0, y0, x1, y1):
        points = []
        dx, dy = abs(x1 - x0), abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        while True:
            points.append((x0, y0))
            if x0 == x1 and y0 == y1: break
            e2 = 2 * err
            if e2 > -dy: err -= dy; x0 += sx
            if e2 < dx: err += dx; y0 += sy
        return points

    def update_map(self, px, py, pyaw, laser_scan):
        # Convert World Coordinates to Pixel Coordinates
        # (Assuming center of map 750,485 and 50px/meter scale)
        r_x = int(750 + px * 50)
        r_y = int(485 - py * 50)
        
        for i in range(0, len(laser_scan), 5): # Step by 5 for speed
            dist = laser_scan[i]
            if dist > 8.0 or dist < 0.1: continue

            angle = pyaw + self.laser_angles[i % 360]
            hx = int(750 + (px + dist * math.cos(angle)) * 50)
            hy = int(485 - (py + dist * math.sin(angle)) * 50)

            line = self.bresenham_line(r_x, r_y, hx, hy)
            for (cx, cy) in line[:-1]:
                if 0 <= cx < 1500 and 0 <= cy < 970:
                    self.l_map[cy, cx] -= 0.2
            if 0 <= hx < 1500 and 0 <= hy < 970:
                self.l_map[hy, hx] += 0.5

        np.clip(self.l_map, -5.0, 5.0, out=self.l_map)
        self.image_map[:] = 128
        self.image_map[self.l_map < -1.5] = 255 # Free
        self.image_map[self.l_map > 1.5] = 0   # Occupied
        return self.image_map

def normalize_angle(angle):
    while angle > math.pi: angle -= 2 * math.pi
    while angle < -math.pi: angle += 2 * math.pi
    return angle

# ==========================================
# 2. MAIN BLOCK (VisualCircuit Entry Point)
# ==========================================
def main(inputs, outputs, parameters, synchronise):
    # --- PERSISTENT STATE (Defined once) ---
    mapper = RobotMapper()
    state = 0           # 0: FORWARD, 1: TURN1, 2: SHIFT, 3: TURN2
    turn_dir = 1        # 1: LEFT, -1: RIGHT
    target_yaw = 0.0    
    timer = 0           
    
    # Tuning
    FORWARD_SPEED = 0.4
    TURN_SPEED = 0.3
    WALL_DIST = 0.8
    LANE_SHIFT_TICKS = 100 

    print("Custom LawnMower Block Started...")

    while True:
        # 1. READ INPUTS (From ROS2 Odometer & Laser blocks)
        odom = inputs.read_array("Odom")   # Format: [x, y, z, roll, pitch, yaw]
        laser = inputs.read_array("Laser") # Format: [ranges...]
        
        if odom is None or laser is None:
            synchronise()
            continue

        px, py, pyaw = odom[0], odom[1], odom[5]
        
        # 2. MAPPING
        map_img = mapper.update_map(px, py, pyaw, laser)
        outputs.write_image("Map", map_img)

        # 3. NAVIGATION LOGIC (FSM)
        # Check distance in front cone
        front_cone = laser[0:15] + laser[-15:]
        front_dist = min([d for d in front_cone if d > 0.1] or [10.0])
        
        v, w = 0.0, 0.0

        if state == 0: # STATE_FORWARD
            v, w = FORWARD_SPEED, 0.0
            if front_dist < WALL_DIST:
                state = 1
                target_yaw = normalize_angle(pyaw + (turn_dir * math.radians(90)))
        
        elif state == 1: # STATE_TURN_1
            err = normalize_angle(target_yaw - pyaw)
            v, w = 0.0, (TURN_SPEED if err > 0 else -TURN_SPEED)
            if abs(err) < 0.1:
                state = 2
                timer = LANE_SHIFT_TICKS
        
        elif state == 2: # STATE_SHIFT
            v, w = FORWARD_SPEED, 0.0
            timer -= 1
            if timer <= 0:
                state = 3
                target_yaw = normalize_angle(pyaw + (turn_dir * math.radians(90)))
        
        elif state == 3: # STATE_TURN_2
            err = normalize_angle(target_yaw - pyaw)
            v, w = 0.0, (TURN_SPEED if err > 0 else -TURN_SPEED)
            if abs(err) < 0.1:
                state = 0
                turn_dir *= -1 # Reverse turn direction for next wall

        # 4. WRITE OUTPUTS
        outputs.write_array("Vel", [v, w])

        # 5. SYNC
        synchronise()
