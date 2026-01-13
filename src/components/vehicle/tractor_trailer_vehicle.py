
import sys
from pathlib import Path
from math import cos, sin
import numpy as np

# Add current directory to sys.path so it can find its neighbors
abs_dir_path = str(Path(__file__).absolute().parent)
sys.path.append(abs_dir_path)

# Now import the existing repo components
from body import Body
from chassis import Chassis
from four_wheels_vehicle import FourWheelsVehicle
from trailer_body import TrailerBody

# Define your TrailerBody and TractorTrailerVehicle here...
class TractorTrailerVehicle(FourWheelsVehicle):
    def __init__(self, state, spec, controller=None, show_zoom=True):
        super().__init__(state, spec, show_zoom)
        self.trailer_body = TrailerBody(spec)
        self.controller = controller

    def update(self, time_s):
        """
        Updates the vehicle state using the Pure Pursuit controller
        """
        if self.controller:
            # 1. Trigger the controller's internal calculations
            self.controller.update(self.state, time_s)

            # 2. Retrieve the calculated values using the repo's getter methods
            accel = self.controller.get_target_accel_mps2()
            yaw_rate = self.controller.get_target_yaw_rate_rps()
            
            # 3. Apply the inputs to the tractor-trailer state
            self.state.update(accel, yaw_rate, time_s, self.spec.trailer_len_m)
        else:
            # Fallback if no controller is attached
            self.state.update(0.0, 0.0, time_s, self.spec.trailer_len_m)
    
    def draw(self, axes, elems):
        # 1. Get tractor pose
        x_m = self.state.get_x_m()
        y_m = self.state.get_y_m()
        yaw_rad = self.state.yaw_rad
        pose = np.array([[x_m], [y_m], [yaw_rad]])

        # 2. Draw Tractor (Body and Chassis)
        self.body.draw(axes, pose, elems)
        self.chassis.draw(axes, pose, elems)

        # 3. Calculate Hitch Point (Rear axle of tractor)
        hitch_x = x_m - self.spec.r_len_m * np.cos(yaw_rad)
        hitch_y = y_m - self.spec.r_len_m * np.sin(yaw_rad)
        
        # 4. Draw Trailer
        self.trailer_body.draw(axes, hitch_x, hitch_y, self.state.trailer_yaw_rad, elems)

        # 5. Handle Zoom (Standard repo behavior)
        if self.show_zoom:
            axes.set_xlim(x_m - self.spec.area_size, x_m + self.spec.area_size)
            axes.set_ylim(y_m - self.spec.area_size, y_m + self.spec.area_size)
        
        # Add this to draw the Pure Pursuit target point
        if self.controller:
            self.controller.draw(axes, elems)