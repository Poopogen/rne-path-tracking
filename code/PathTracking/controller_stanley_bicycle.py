import sys
import numpy as np 
sys.path.append("..")
import PathTracking.utils as utils
from PathTracking.controller import Controller

class ControllerStanleyBicycle(Controller):
    def __init__(self, model, 
                 # TODO 4.3.1: Tune Stanley Gain
                 kp=1.2):
                 #kp=0):#original code, to be replaced
        self.path = None
        self.kp = kp
        self.l = model.l
        self.current_idx = 0

    def set_path(self, path):
        super().set_path(path)
        self.current_idx = 0

    # State: [x, y, yaw, delta, v]
    def feedback(self, info):
        # Check Path
        if self.path is None:
            print("No path !!")
            return None
        
        # Extract State 
        x, y, yaw, delta, v = info["x"], info["y"], info["yaw"], info["delta"], info["v"]

        # Check if reached end of track
        if self.current_idx >= len(self.path) - 5:
            return 0.0

        # Search Front Wheel Target Locally
        front_x = x + self.l*np.cos(np.deg2rad(yaw))
        front_y = y + self.l*np.sin(np.deg2rad(yaw))
        vf = v / np.cos(np.deg2rad(delta)) if np.cos(np.deg2rad(delta)) != 0 else v
        
        min_idx, min_dist = utils.search_nearest_local(self.path, (front_x,front_y), self.current_idx, lookahead=50)
        self.current_idx = min_idx
        target = self.path[min_idx] # [x, y, Direction the path is pointing at idx point(θ_path), curvature] ##NewFeature add notes

        # TODO 4.3.1: Stanley Control for Bicycle Kinematic Model
        #next_delta = 0 #original code, to be replaced
        # Step 1: Find the nearest point on the path. #Already done: self.path[min_idx]
        
        # Step 2: Compute the heading error theta_e (θe) between the vehicle's current AND cross-track error e

        # Step 2a: Heading error theta_e = θ_path - yaw
        ## θ_path: angle from current waypoint toward the next waypoint = tangent direction of the path at that point, in degrees (target[2])
        ## yaw: the vehicle's actual heading (rear wheel direction)
        theta_e = utils.angle_norm(target[2] - yaw)

        # Step 2b: Signed cross-track error e
        # |e| = sqrt((xf - x_path)² + (yf - y_path)²)
            # → already computed by search_nearest_local as min_dist = d²
            # → np.sqrt(min_dist) gives |e| → NO SIGNS

            
        # # yaw_to_front: angle from nearest path point to front axle
        # yaw_to_front = np.rad2deg(np.arctan2(front_y - target[1], front_x - target[0]))
        # # yaw_diff_cross > 0 → front axle is LEFT of path   → -sign = -1  → e < 0
        # # yaw_diff_cross < 0 → front axle is RIGHT of path  → -sign = +1  → e > 0
        # yaw_diff_cross = utils.angle_norm(target[2] - yaw_to_front)
        # # -np.sign(yaw_diff_cross): so that e > 0 when front axle is RIGHT of path and to steer left via arctan(-kp*e/vf) < 0
        # e = -np.sign(yaw_diff_cross) * np.sqrt(min_dist)


        # Offset vector from nearest path point to front axle
        # Offset vector (dx, dy) : vector (an arrow) pointing from the nearest point on the path (target) to the vehicle's front wheels (front).
        dx = front_x - target[0]
        dy = front_y - target[1]

        # Path normal vector (perpendicular to path, pointing LEFT of travel direction)
        # Normal Vector (normal_x, normal_y)
        # path direction vector  = (cos θ,  sin θ)   →  forward path direction
        # left normal vector     = (-sin θ, cos θ)   →  90° left of forward path direction
        # Rotate path direction 90° counterclockwise: (-sin, cos)
        normal_x = -np.sin(np.deg2rad(target[2]))
        normal_y =  np.cos(np.deg2rad(target[2]))

        # Signed cross-track error: 
        # dot product of offset with normal
        # The dot product projects the offset vector onto the left-normal direction:
        
            ## front wheel is LEFT of path  → e > 0  (need to steer RIGHT)
            ## front wheel is RIGHT of path → e < 0  (need to steer LEFT)
            ## front wheel ON the path      → e = 0  (no correction needed)
            
        e = dx * normal_x + dy * normal_y   




        # Step 3: δ = θ_e + arctan(-kp *e / v_f)
        ## negative sign -kp: if e > 0 (left), steer right (negative delta)
        
        # Speed-adaptive gain in Stanley control:
            ## At high speed: v_f is large → kp * e / v_f is small → gentle steering (safe)
            ## At low speed: v_f is small → kp * e / v_f is large → more steering correction (responsive)
        
        # Small kp: Gentle correction — slow to return to path, less oscillation
        # Large kp: Aggressive correction — returns to path quickly, may overshoot/oscillate
        next_delta = theta_e + np.rad2deg(np.arctan2(-self.kp * e, vf))

        # [end] TODO 4.3.1
    
        return next_delta
