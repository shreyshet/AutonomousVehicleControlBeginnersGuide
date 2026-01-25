import numpy as np
from math import sin, cos, tan, atan2
from scipy.optimize import minimize

class MPCController:
    """
    Controller class by Model Predictive Control algorithm
    """
    def __init__(self, spec, course, T = 6, dt = 0.1, color='b'):
        # MPC parameters
        self.T = T # Prediction horizon
        self.DT = dt # Time step
        self.WHEEL_BASE_M = spec.wheel_base_m
        self.DRAW_COLOR = color
        
        # State and Input weights for optimization
        # [x, y, yaw, v]
        self.Q = np.diag([10.0, 10.0, 15.0, 20.0])
        # [accel, steer]
        self.R = np.diag([0.1, 5.0])

        self.course = course
        self.target_accel_mps2 = 0.0
        self.target_steer_rad = 0.0
        self.target_yaw_rate_rps = 0.0
        self.predicted_states = None

    def _linearize_model(self, state, steer, target_v):
        v = target_v #state.get_speed_mps()
        phi = state.yaw_rad
        L = self.WHEEL_BASE_M
        dt = self.DT

        # A matrix (State transition)
        A = np.eye(4)
        A[0, 2] = -v * np.sin(phi) * dt
        A[0, 3] = np.cos(phi) * dt
        A[1, 2] = v * np.cos(phi) * dt
        A[1, 3] = np.sin(phi) * dt
        A[2, 3] = np.tan(steer) * dt / L

        # B matrix (Input transition)
        B = np.zeros((4, 2))
        B[2, 1] = v * dt / (L * np.cos(steer)**2)
        B[3, 0] = dt

        return A, B

    def _cost_func(self, u_flat, state, ref_traj, A, B):
        """
        u_flat: The optimization variables (accel, steer for T steps)
        state: The current vehicle state
        ref_traj: The pre-calculated target points (Passed as argument)
        A, B: Linearized matrices
        """
        u = u_flat.reshape(self.T, 2)
        cost = 0.0
        
        # Current state values
        x = state.get_x_m()
        y = state.get_y_m()
        phi = state.yaw_rad
        v = state.get_speed_mps()
        curr_state = np.array([x, y, phi, v])

        for t in range(self.T):
            # Exponential weight: weights increase as we look further into the future
            # This allows the car to deviate slightly NOW to achieve a better path LATER
            time_weight = (t + 1) / self.T 

            # Linear Prediction: s_next = A*s + B*u
            curr_state = A @ curr_state + B @ u[t]

            # Use the ref_traj passed as an argument
            ref = np.array(ref_traj[t]) # [x, y, yaw, v]
            
            # State error cost
            error = curr_state - ref
            cost += error.T @ self.Q @ error * time_weight
            
            # Input effort cost
            cost += u[t].T @ self.R @ u[t]

        return cost

    def update(self, state, time_s):
        # 1. Define target speed (20 kph)
        target_v = 20.0 / 3.6  # Convert kph to mps (~5.56)
        
        # 2. Get Reference Trajectory
        nearest_idx = self.course.search_nearest_point_index(state)
        ref_traj = self._get_ref_trajectory(state)
        self.ref_traj = ref_traj  # Save for debugging/drawing
        # 3. Linearize around 20 kph and current heading
        A, B = self._linearize_model(state, self.target_steer_rad, target_v)

        # 2. Optimization setup
        if not hasattr(self, 'prev_u'): self.prev_u = np.zeros(self.T * 2) # Initial guess
        bounds = [(-3.0, 3.0), (np.radians(-35), np.radians(35))] * self.T # Accel and Steer limits

        # 3. Solve
        res = minimize(self._cost_func, self.prev_u, args=(state, self.ref_traj, A, B), 
                       method='SLSQP', bounds=bounds)

        if res.success:
            self.prev_u = res.x # Save for next time
            u_opt = res.x.reshape(self.T, 2)
            self.target_accel_mps2 = u_opt[0, 0]
            self.target_steer_rad = u_opt[0, 1]
            
            # Convert steering to yaw rate for vehicle state update
            v = state.get_speed_mps()
            self.target_yaw_rate_rps = v * tan(self.target_steer_rad) / self.WHEEL_BASE_M
            
            # Save predicted states for drawing
            self._update_predictions(state, u_opt)

    def _update_predictions(self, state, u_opt):
        x, y, yaw, v = state.get_x_m(), state.get_y_m(), state.yaw_rad, state.get_speed_mps()
        self.predicted_states = []
        for t in range(self.T):
            x += v * cos(yaw) * self.DT
            y += v * sin(yaw) * self.DT
            yaw += v * tan(u_opt[t, 1]) / self.WHEEL_BASE_M * self.DT
            v += u_opt[t, 0] * self.DT
            self.predicted_states.append((x, y))

    # Getter methods matching the repo's naming convention
    def get_target_accel_mps2(self):
        return self.target_accel_mps2

    def get_target_steer_rad(self):
        return self.target_steer_rad

    def get_target_yaw_rate_rps(self):
        return self.target_yaw_rate_rps

    def _get_ref_trajectory(self, state):
        """
        Creates a list of target states [x, y, yaw, v] for the prediction horizon
        """
        nearest_idx = self.course.search_nearest_point_index(state)
        ref = []
        target_v = 20.0 / 3.6 # Our 20 kph target in m/s

        for i in range(self.T):
            # We look ahead in the course. 
            # You can increase the 'stride' to look further ahead at higher speeds.
            idx = min(nearest_idx + i, self.course.length() - 1)
            
            ref.append([
                self.course.point_x_m(idx),
                self.course.point_y_m(idx),
                self.course.point_yaw_rad(idx),      # We can set target yaw to 0 or use course.point_yaw_rad(idx)
                target_v  # Constant 20 kph
            ])
        return ref
    
    def draw(self, axes, elems):
        if self.predicted_states:
            px = [p[0] for p in self.predicted_states]
            py = [p[1] for p in self.predicted_states]
            predict_plot, = axes.plot(px, py, color=self.DRAW_COLOR, linestyle='--', label="MPC Prediction")
            elems.append(predict_plot)