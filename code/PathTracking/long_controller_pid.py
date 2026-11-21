import sys
import numpy as np 
sys.path.append("..")
import PathTracking.utils as utils
from PathTracking.controller import Controller

class PIDLongController(Controller):
    def __init__(self, model, a_range, kp=1.5, ki=5, kd=0):
        self.path = None
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.acc_ep = 0
        self.last_ep = 0
        self.dt = model.dt
        self.a_range = a_range
        self.current_idx = 0
    
    def set_path(self, path):
        super().set_path(path)
        self.acc_ep = 0
        self.last_ep = 0
        self.current_idx = 0
    
    def feedback(self, info):
        # Check Path
        if self.path is None:
            print("No path !!")
            return None, None
        
        # Extract State
        x, y, yaw, v = info["x"], info["y"], info["yaw"], info["v"]

        # Check if reached end of track
        if self.current_idx >= len(self.path) - 1:
            # Brake to 0 speed using PID when finishing the track
            v_ref = 0.0
            target = self.path[-1]
            return np.clip(-v, self.a_range[0], self.a_range[1]), target
        else:
            # Search Nearest Target Locally
            min_idx, min_dist = utils.search_nearest_local(self.path, (x,y), self.current_idx, lookahead=50)
            self.current_idx = min_idx
            target = self.path[min_idx]
            v_ref = target[4]
        
        # TODO 3.2: PID Control for Longitudinal Motion
        #next_a = v_ref - v #original code, to be replaced

        # v_ref is the reference speed from the trajectory, v is the current speed of the vehicle
        # kp (1/s) is the proportional gain, ki (1/s^2) is the integral gain, kd (s) is the derivative gain

        ep = v_ref - v                                          # P: current error
        self.acc_ep += ep * self.dt                             # I: accumulated error
        #self.acc_ep = np.clip(self.acc_ep, self.a_range[0]/self.ki, self.a_range[1]/self.ki)  # anti-windup
        ded = (ep - self.last_ep) / self.dt                     # D: error derivative
        next_a = self.kp * ep + self.ki * self.acc_ep + self.kd * ded # PID control law 
        next_a = np.clip(next_a, self.a_range[0], self.a_range[1]) # Constrain acceleration to a_range
        self.last_ep = ep
        #print(f"idx={self.current_idx}/{len(self.path)-1} | v={v:.2f} v_ref={v_ref:.2f} ep={ep:.2f} acc_ep={self.acc_ep:.2f} next_a={next_a:.2f}")

        # [end] TODO 3.2

        return next_a, target