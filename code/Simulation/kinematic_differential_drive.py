import numpy as np
import sys
sys.path.append("..")
from Simulation.utils import State, ControlState
from Simulation.kinematic import KinematicModel

class KinematicModelDifferentialDrive(KinematicModel):
    def __init__(self, r, l, dt=0.05):
        # Simulation delta time
        self.r = r
        self.l = l
        self.dt = dt
    
    def step(self, state:State, cstate:ControlState) -> State:
        # from lecture notes:
        # x1dot (right wheel speed)= r*rw/2
        # w1 (right wheel angular velocity)= r*rw/(2*l) 
        # x2dot (left wheel speed)= r*lw/2 
        # w2 (left wheel angular velocity)= - r*lw/(2*l) #negative sign because left wheel contributes to negative angular velocity 

        x1dot = self.r*np.deg2rad(cstate.rw) / 2
        w1 = np.rad2deg(self.r*np.deg2rad(cstate.rw) / (2*self.l))
        x2dot = self.r*np.deg2rad(cstate.lw) / 2
        w2 = np.rad2deg(self.r*np.deg2rad(cstate.lw) / (2*self.l))
        v = x1dot + x2dot #
        w = w1 - w2 #negative sign because left wheel contributes to negative angular velocity

        x = state.x + v * np.cos(np.deg2rad(state.yaw)) * self.dt
        y = state.y + v * np.sin(np.deg2rad(state.yaw)) * self.dt
        yaw = (state.yaw + w * self.dt) % 360
        state_next = State(x, y, yaw, v, w)
        return state_next