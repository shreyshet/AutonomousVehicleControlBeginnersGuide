"""
visualize_mpc.py
Author: Gemini Thought Partner
"""

import sys
from pathlib import Path
import numpy as np

# Path configuration to find repo components
abs_dir_path = str(Path(__file__).absolute().parent)
print("Absolute directory path:", abs_dir_path)
relative_path = "/../../../components/"

sys.path.append(abs_dir_path + relative_path + "visualization")
sys.path.append(abs_dir_path + relative_path + "state")
sys.path.append(abs_dir_path + relative_path + "vehicle")
sys.path.append(abs_dir_path + relative_path + "course/cubic_spline_course")
sys.path.append(abs_dir_path + relative_path + "control/mpc_controller") # Folder for your new MPC

from global_xy_visualizer import GlobalXYVisualizer
from min_max import MinMax
from time_parameters import TimeParameters
from vehicle_specification import VehicleSpecification
from state import State
from four_wheels_vehicle import FourWheelsVehicle
from cubic_spline_course import CubicSplineCourse
from mpc_controller import MPCController

def main():
    # 1. Setup Environment
    x_lim, y_lim = MinMax(-5, 60), MinMax(-20, 25)
    vis = GlobalXYVisualizer(x_lim, y_lim, TimeParameters(span_sec=30))

    # 2. Create a challenging course
    course = CubicSplineCourse([0.0, 15.0, 30.0, 45.0, 60.0],
                               [0.0, 10.0, -10.0, 15.0, 0.0],
                               20)
    vis.add_object(course)

    # 3. Define Vehicle and MPC Controller
    spec = VehicleSpecification(area_size=20.0)
    state = State(x_m=0.0, y_m=0.0, yaw_rad=0.0, speed_mps=0.0)
    
    # Initialize MPC
    mpc = MPCController(spec, course, T=10, dt=0.1)
    
    # Create Vehicle
    vehicle = FourWheelsVehicle(state, spec, controller=mpc)
    vis.add_object(vehicle)

    # 4. Start Simulation
    vis.draw()

if __name__ == "__main__":
    main()