import sys
import numpy as np 
sys.path.append("..")
import PathTracking.utils as utils
from PathTracking.controller import Controller

class ControllerPurePursuitBicycle(Controller):
    def __init__(self, model, 
                 # TODO 4.2.1: Tune Pure Pursuit Gain
                 kp=0.5, Lfc=0.8):
                 #kp=1, Lfc=0):#original code, to be replaced
        self.path = None
        self.kp = kp
        self.Lfc = Lfc
        self.dt = model.dt
        self.l = model.l
        self.current_idx = 0

    def set_path(self, path):
        super().set_path(path)
        self.current_idx = 0

    # State: [x, y, yaw, v, l]
    def feedback(self, info):
        # Check Path
        if self.path is None:
            print("No path !!")
            return None
        
        # Extract State 
        x, y, yaw, v = info["x"], info["y"], info["yaw"], info["v"]

        # Check if reached end of track
        if self.current_idx >= len(self.path) - 5:
            return 0.0

        min_idx, min_dist = utils.search_nearest_local(self.path, (x,y), self.current_idx, lookahead=50)
        self.current_idx = min_idx
        
        Ld = self.kp*v + self.Lfc 
        
        # TODO 4.2.1: Pure Pursuit Control for Bicycle Kinematic Model
        #next_delta = 0 #original code, to be replaced

        '''
        Ld = self.kp*v + self.Lfc 
        Low velocity v → small Ld ≈ self.Lfc → more reactive but less smooth path tracking.
        High velocity v → large Ld → smoother but less reactive path tracking.

        kp : The gain that scales lookahead distance with vehicle speed. 
        Lfc : The minimum (constant) lookahead distance, independent of speed. 
        It prevents from collapsing to 0 when the vehicle is stationary (v=0), which would cause a division-by-zero or degenerate geometry in:
        delta = arctan(2*L*sin(alpha) / Ld)
        '''

        # Step 1: Find the nearest point on the path.
        lookahead_idx = min_idx #
        # Step 2: Find the first point on the path (Lookhead point(xg,yg)) that is at least Ld away from the nearest point.
        for i in range(min_idx, len(self.path)):
            dist = np.sqrt((self.path[i,0] - x)**2 + (self.path[i,1] - y)**2)
            if dist >= Ld:
                lookahead_idx = i
                break
        target = self.path[lookahead_idx]

        # Step 3: Calculate the steering angle δ(delta) to move the vehicle toward the ”look-ahead” point.
        # yaw: theta in lecture notes (angle of the vehicle's heading)
        # alpha = atan2(yg - y, xg - x) - yaw
        theta_target = np.rad2deg(np.arctan2(target[1] - y, target[0] - x))
        alpha = utils.angle_norm(theta_target - yaw)

        # Steering angle formula: delta = arctan(2*L*sin(alpha) / Ld)
        next_delta = np.rad2deg(np.arctan2(2 * self.l * np.sin(np.deg2rad(alpha)), Ld))



        # [end] TODO 4.2.1
        return next_delta
