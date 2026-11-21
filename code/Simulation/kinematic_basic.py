import numpy as np
import sys
sys.path.append("..")
from Simulation.utils import State, ControlState
from Simulation.kinematic import KinematicModel

class KinematicModelBasic(KinematicModel):
    def __init__(self, dt=0.05):
        # Simulation delta time
        self.dt = dt

    def step(self, state:State, cstate:ControlState) -> State:
        v = cstate.v # vehicle’s speed v (linear velocity) from control state
        w = cstate.w # vehicle’s angular speed w (angular velocity)from control state
        # update x and y position and yaw θ in object(vehicle) state by eq. 1
        x = state.x + v * np.cos(np.deg2rad(state.yaw)) * self.dt
        y = state.y + v * np.sin(np.deg2rad(state.yaw)) * self.dt
        # Problem 2.1.1: Update Yaw Angle
        yaw = (state.yaw + state.w * self.dt) % 360
        state_next = State(x, y, yaw, v, w)
        return state_next
