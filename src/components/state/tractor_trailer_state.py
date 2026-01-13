import numpy as np
from math import cos, sin
import sys
from pathlib import Path

# Path hack to ensure it can find the original state.py in the same folder
abs_dir_path = str(Path(__file__).absolute().parent)
sys.path.append(abs_dir_path)
from state import State

class TractorTrailerState(State):
    """
    Tractor-Trailer state(x, y, yaw, trailer_yaw, speed) data and logic class
    """

    def __init__(self, x_m=0.0, y_m=0.0, yaw_rad=0.0, trailer_yaw_rad=0.0, speed_mps=0.0, color='k'):
        """
        Constructor
        x_m, y_m: Tractor's position [m]
        yaw_rad: Tractor's yaw angle [rad]
        trailer_yaw_rad: Trailer's yaw angle [rad]
        speed_mps: Vehicle speed [m/s]
        """
        # Initialize the base State (x, y, yaw, speed)
        super().__init__(x_m, y_m, yaw_rad, speed_mps, color)
        
        # New state variable for the trailer
        self.trailer_yaw_rad = trailer_yaw_rad
        self.trailer_yaw_history = [self.trailer_yaw_rad]

    def update(self, accel_mps2, yaw_rate_rps, time_s, trailer_l_m=5.0):
        """
        Function to update state of both tractor and trailer
        accel_mps2: Acceleration [m/s^2]
        yaw_rate_rps: Tractor's yaw rate [rad/s]
        time_s: Time interval per cycle [sec]
        trailer_l_m: Distance from hitch to trailer axle [m]
        """
        # 1. Prepare current tractor state for the static motion_model
        last_state = np.array([[self.x_m],
                               [self.y_m],
                               [self.yaw_rad],
                               [self.speed_mps]])
        
        next_input = np.array([[accel_mps2],
                               [yaw_rate_rps]])
        
        # Conceptual logic for TractorTrailerState.update
        yaw_diff = abs(self.yaw_rad - self.trailer_yaw_rad)

        # If the trailer is at a sharp angle, it "drags" the tractor, slowing it down
        if yaw_diff > np.radians(30):
            self.speed_mps *= 0.9  # Reduce speed by 10% due to drag
        
        # 2. Update Tractor State using the repository's built-in matrix math
        next_state = self.motion_model(last_state, next_input, time_s)

        # 3. Compute Trailer Articulation Physics
        # The change in trailer angle (d_yaw1) = (v / L) * sin(yaw_tractor - yaw_trailer)
        yaw_diff = self.yaw_rad - self.trailer_yaw_rad
        trailer_yaw_rate = (self.speed_mps / trailer_l_m) * sin(yaw_diff)
        

        # 4. Save results back to member variables
        self.x_m = next_state[0, 0]
        self.y_m = next_state[1, 0]
        self.yaw_rad = next_state[2, 0]
        self.speed_mps = next_state[3, 0]
        
        # Update trailer yaw using Euler integration
        self.trailer_yaw_rad += trailer_yaw_rate * time_s

        # 5. Speed limits (standard repo logic)
        if abs(self.speed_mps) < self.STOP_SPEED_MPS: self.speed_mps = 0.0
        if self.speed_mps > self.MAX_SPEED_MPS: self.speed_mps = self.MAX_SPEED_MPS
        if self.speed_mps < self.MIN_SPEED_MPS: self.speed_mps = self.MIN_SPEED_MPS

        # 6. Update histories
        self.x_history.append(self.x_m)
        self.y_history.append(self.y_m)
        self.trailer_yaw_history.append(self.trailer_yaw_rad)

    def get_trailer_yaw(self):
        return self.trailer_yaw_rad

    def x_y_yaw(self):
        """
        Helper for the tractor pose
        """
        return np.array([[self.x_m], [self.y_m], [self.yaw_rad]])