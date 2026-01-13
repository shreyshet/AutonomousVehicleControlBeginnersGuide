"""
visualize_vehicle.py

Author: Shisato Yano
"""

# import path setting
import numpy as np
import sys
from pathlib import Path

abs_dir_path = str(Path(__file__).absolute().parent)
relative_path = "/../../src/components/"

sys.path.append(abs_dir_path + relative_path + "visualization")
sys.path.append(abs_dir_path + relative_path + "state")
sys.path.append(abs_dir_path + relative_path + "vehicle")


# import component modules
from global_xy_visualizer import GlobalXYVisualizer
from min_max import MinMax
from time_parameters import TimeParameters
from vehicle_specification import VehicleSpecification
from state import State
from four_wheels_vehicle import FourWheelsVehicle
from tractor_trailer_vehicle import TractorTrailerVehicle
from tractor_trailer_state import TractorTrailerState

# flag to show plot figure
# when executed as unit test, this flag is set as false
show_plot = True


def main():
    x_lim, y_lim = MinMax(-50, 50), MinMax(-50, 50)
    vis = GlobalXYVisualizer(x_lim, y_lim, TimeParameters(span_sec=30), show_zoom=False)

    spec = VehicleSpecification()
    spec.trailer_len_m = 7.0  # Length of the trailer
    
    # Start with both aligned at 0 rad
    state = TractorTrailerState(x_m=0, y_m=0, yaw_rad=0.1, trailer_yaw_rad=0.0, speed_mps=2.0)
    vehicle = TractorTrailerVehicle(state, spec, show_zoom=False)

    vis.add_object(vehicle)
    
    # Manually adding a steering input to see the articulation
    # In a real sim, you'd use a controller here
    def control_loop(time_s):
        vehicle.update(0.1, 0.05, time_s) # Slow accel + constant turn
    
    vis.draw()


# execute main process
if __name__ == "__main__":
    main()
