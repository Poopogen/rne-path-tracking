import numpy as np
import sys
sys.path.append("..")
from Simulation.utils import State, ControlState
from Simulation.kinematic import KinematicModel

class KinematicModelBicycle(KinematicModel):
    def __init__(self,
            l = 30,     # distance between rear and front wheel
            dt = 0.05
        ):
        # Distance from center to wheel
        self.l = l
        # Simulation delta time
        self.dt = dt

    def step(self, state:State, cstate:ControlState) -> State:
        # TODO 2.3.1: Bicycle Kinematic Model
        #v, w, x, y, yaw = 0, 0, state.x, state.y, state.yaw #original code, to be replaced

        #state: velocity v (m/s), angular velocity w (degree/s), position x,y (m), yaw(theta in notes) (degree)
        #control state: cstate.v (m/s^2) is the linear velocity command, cstate.delta (degree) is the steering angle command
        
        v = state.v + cstate.a * self.dt # update velocity after contrrol command
        
        # Update Angular Velocity using Bicycle Model: w = v / l * tan(delta)
        w = np.rad2deg(v / self.l * np.tan(np.deg2rad(cstate.delta))) # update angular velocity after control command
        
        # Update Position using Bicycle Model: x = x + v * cos(yaw) * dt, y = y + v * sin(yaw) * dt
        x = state.x + v * self.dt * np.cos(np.deg2rad(state.yaw)) 
        y = state.y + v * self.dt * np.sin(np.deg2rad(state.yaw))
        
        # Update Yaw Angle using Bicycle Model
        yaw = (state.yaw + w * self.dt) % 360


        # [end] TODO 2.3.1
        state_next = State(x, y, yaw, v, w)
        return state_next
