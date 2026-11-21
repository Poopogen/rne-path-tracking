from sre_parse import State
import sys
import numpy as np 
sys.path.append("..")
import PathTracking.utils as utils
from PathTracking.controller import Controller

class ControllerLQRBicycle(Controller):
    def __init__(self, model, Q=None, R=None, control_state='steering_angle'):
        self.path = None
        if control_state == 'steering_angle':
            self.Q = np.eye(2)
            self.R = np.eye(1)
            # TODO 4.4.1: Tune LQR Gains
            # Q (State Weight Matrix): Penalizes deviations from the desired state (error).
            # R (Control Weight Matrix): Penalizes the control effort (energy expenditure).
            self.Q[0,0] = 10 #tune: weight for cross-track error e (meters); higher → more aggressive lateral correction to reduce e
            self.Q[1,1] = 100 #tune: weight for heading error θ_e (radians); higher → more aggressive heading alignment to reduce heading error
            self.R[0,0] = 2.0 #tune: weight for steering angle δ (radians); higher → Smoother but slower response to reduce control effort
            
            # original code, to be replaced
            # self.Q[0,0] = 1
            # self.Q[1,1] = 1 
            # self.R[0,0] = 1
        elif control_state == 'steering_angular_velocity':
            self.Q = np.eye(3)
            self.R = np.eye(1)
            # TODO 4.4.4: Tune LQR Gains
            self.Q[0,0] = 100
            self.Q[1,1] = 500
            self.Q[2,2] = 8 #tune: weight for steering angle δ (radians); higher → more aggressive correction to reduce heading error(δ penalty), but may cause oscillation if too high
            self.R[0,0] = 1 #5.3 #tune: weight for steering angular velocity δ̇ (rad/s); higher → smoother but slower response to reduce control effort
            #Bryson's rule: Rate-limit delta change → ±25° (0.4363 rad) ==> set R[0,0] to (1/(0.4363 rad)^2) ≈ 5.26
            # Units: CTE error e (meters), Heading error θ_e (radians), Steering angle δ (radians), Steering angular velocity δ̇ (rad/s)
            # Relative importance: Jθe (Q[0,0]) : Je (Q[1,1]) : Jδ (Q[2,2]) =  100:1:1
            


            # original code, to be replaced
            # self.Q[0,0] = 1
            # self.Q[1,1] = 1
            # self.Q[2,2] = 1
            # self.R[0,0] = 1
        self.pe = 0
        self.pth_e = 0
        self.pdelta = 0
        self.dt = model.dt
        self.l = model.l
        self.control_state = control_state
        self.current_idx = 0

    def set_path(self, path):
        super().set_path(path)
        self.pe = 0
        self.pth_e = 0
        self.pdelta = 0
        self.current_idx = 0

    def _solve_DARE(self, A, B, Q, R, max_iter=150, eps=0.01): # Discrete-time Algebra Riccati Equation (DARE)
        P = Q.copy() # Initialize P with Q (state weight matrix) #NewFeature add notes
        for i in range(max_iter):
            temp = np.linalg.inv(R + B.T @ P @ B)
            Pn = A.T @ P @ A - A.T @ P @ B @ temp @ B.T @ P @ A + Q # DARE equation (10) in HW2 PDF #NewFeature add notes
            if np.abs(Pn - P).max() < eps:
                break
            P = Pn
        return Pn

    # State: [x, y, yaw, delta, v]
    def feedback(self, info):
        # Check Path
        if self.path is None:
            print("No path !!")
            return None
        
        # Extract State 
        x, y, yaw, delta, v = info["x"], info["y"], info["yaw"], info["delta"], info["v"]
        yaw = utils.angle_norm(yaw)

        # Check if reached end of track
        if self.current_idx >= len(self.path) - 3:
            return 0.0
        
        # Search Nesrest Target
        #min_idx, min_dist = utils.search_nearest(self.path, (x,y))
        min_idx, min_dist = utils.search_nearest_local(self.path, (x,y), self.current_idx, lookahead=50)
        target = self.path[min_idx]
        target[2] = utils.angle_norm(target[2])
        
        if self.control_state == 'steering_angle':
            # TODO 4.4.1: LQR Control for Bicycle Kinematic Model with steering angle as control input
            #next_delta = 0 #original code, to be replaced
            
            
            # Heading error θ_e = θ_path - θ (radians), consistent with Eq.(13)(14)
                ## θ_path: angle from current waypoint toward the next waypoint = tangent direction of the path at that point, in degrees (target[2])
                ## yaw: the vehicle's actual heading (rear wheel direction)
            th_e = np.deg2rad(utils.angle_norm(target[2] - yaw))

            # # Cross-track error e: 
            dx = x - target[0]
            dy = y - target[1]
            normal_x = -np.sin(np.deg2rad(target[2]))
            normal_y =  np.cos(np.deg2rad(target[2]))
            e = dx * normal_x + dy * normal_y


            # State vector x = [e, θ_e]ᵀ (units: meters, radians)
            state = np.array([[e], [th_e]])

            # Linearized discrete system matrices from Eq.(13)(14):
            # ė  = -v·θ_e  →  A[0,1] = -v·dt
            # θ̇_e = -v/L·δ →  B[1,0] = -v/L·dt
            A = np.array([[1.0, -v * self.dt],
                          [0.0,  1.0        ]])
            B = np.array([[0.0              ],
                          [-v / self.l * self.dt]])

            # Solve DARE → optimal gain K = (R + BᵀPB)⁻¹(BᵀPA)  Eq.(12)
            P = self._solve_DARE(A, B, self.Q, self.R) #Eq.(10)
            K = np.linalg.inv(self.R + B.T @ P @ B) @ (B.T @ P @ A) #Eq.(12)

            # Optimal control u = -Kx (radians) → convert to degrees for simulator
            u = -K @ state
            next_delta = np.rad2deg(u[0, 0])
            self.current_idx = min_idx

            # [end] TODO 4.4.1
        elif self.control_state == 'steering_angular_velocity':
            # TODO 4.4.4: LQR Control for Bicycle Kinematic Model with steering angular velocity as control input
            #next_delta = 0# original code, to be replaced


            th_e = np.deg2rad(utils.angle_norm(target[2] - yaw))

            # # Cross-track error e: 
            dx = x - target[0]
            dy = y - target[1]
            normal_x = -np.sin(np.deg2rad(target[2]))
            normal_y =  np.cos(np.deg2rad(target[2]))
            e = dx * normal_x + dy * normal_y

            state = np.array([[e],
                              [th_e],
                              [np.deg2rad(delta)]])
            # Feedforward steering angle for path curvature: δ_ff = arctan(κ·L)
            # Shifts δ penalty from absolute value to deviation from required curve-tracking angle
            # kappa = target[3]
            # delta_ff = np.arctan(kappa * self.l)

            # state = np.array([[e],
            #                   [th_e],
            #                   [np.deg2rad(delta) - delta_ff]])  # δ̃ = δ - δ_ff

            # Extended A: δ is now a state; θ̇_e = -v/L·δ → A[1,2] = -v/L·dt
            A = np.array([[1.0, -v * self.dt,          0.0],
                          [0.0,  1.0,         -v / self.l * self.dt],
                          [0.0,  0.0,          1.0               ]])
            B = np.array([[0.0      ],
                          [0.0      ],
                          [self.dt  ]])  # δ_{k+1} = δ_k + δ̇·dt

            P = self._solve_DARE(A, B, self.Q, self.R)
            K = np.linalg.inv(self.R + B.T @ P @ B) @ (B.T @ P @ A)

            u = -K @ state  # u = δ̇ (rad/s)
            # Integrate: δ_{k+1} = δ_k + δ̇·dt
            next_delta = np.rad2deg(np.deg2rad(delta) + u[0, 0] * self.dt)
            self.current_idx = min_idx
            # [end] TODO 4.4.4
        
        return next_delta
