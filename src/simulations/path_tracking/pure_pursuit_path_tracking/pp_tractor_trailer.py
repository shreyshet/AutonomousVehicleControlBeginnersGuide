"""
pure_pursuit_path_tracking.py

Author: Shisato Yano
"""

# import path setting
import sys
from pathlib import Path

abs_dir_path = str(Path(__file__).absolute().parent)
relative_path = "/../../../components/"

sys.path.append(abs_dir_path + relative_path + "visualization")
sys.path.append(abs_dir_path + relative_path + "state")
sys.path.append(abs_dir_path + relative_path + "vehicle")
sys.path.append(abs_dir_path + relative_path + "course/cubic_spline_course")
sys.path.append(abs_dir_path + relative_path + "control/pure_pursuit")


# import component modules
from global_xy_visualizer import GlobalXYVisualizer
from min_max import MinMax
from time_parameters import TimeParameters
from vehicle_specification import VehicleSpecification
from state import State
from four_wheels_vehicle import FourWheelsVehicle
from cubic_spline_course import CubicSplineCourse
from pure_pursuit_controller import PurePursuitController


# flag to show plot figure
# when executed as unit test, this flag is set as false
show_plot = True
from tractor_trailer_state import TractorTrailerState
from tractor_trailer_vehicle import TractorTrailerVehicle

def main():
    # set simulation parameters
    x_lim, y_lim = MinMax(-5, 55), MinMax(-20, 25)
    vis = GlobalXYVisualizer(x_lim, y_lim, TimeParameters(span_sec=25))

    # create course data instance
    course = CubicSplineCourse([0.0, 10.0, 25, 40, 50],
                               [0.0, 4, -12, 20, -13],
                               20)
    vis.add_object(course)

    # create vehicle's spec instance
    spec = VehicleSpecification(area_size=20.0)
    spec.trailer_len_m = 7.0 # Define the trailer length here
    
    # 2. Use TractorTrailerState instead of State
    # Initial trailer yaw should usually match the tractor yaw (0.0)
    state = TractorTrailerState(color=spec.color, trailer_yaw_rad=0.0)
    
    # create controller instance (This stays the same, tracking the tractor)
    pure_pursuit = PurePursuitController(spec, course)

    # 3. Use TractorTrailerVehicle instead of FourWheelsVehicle
    # Note: Ensure TractorTrailerVehicle can accept 'controller' in __init__
    vehicle = TractorTrailerVehicle(state, spec, controller=pure_pursuit)
    vis.add_object(vehicle)

    if not show_plot: vis.not_show_plot()
    vis.draw()

# execute main process
if __name__ == "__main__":
    main()
