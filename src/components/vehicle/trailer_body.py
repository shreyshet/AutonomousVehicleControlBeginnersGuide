
import numpy as np
import sys
from pathlib import Path
sys.path.append(str(Path(__file__).absolute().parent) + "/../array")
from xy_array import XYArray
from body import Body

class TrailerBody(Body):
    def __init__(self, spec):
        super().__init__(spec)
        # Trailer box: starts at hitch (0,0) and extends backward
        # Format: [[x1, x2, x3, x4, x1], [y1, y2, y3, y4, y1]]
        t_len = spec.trailer_len_m
        t_wid = spec.width_m
        contour = np.array([[0.0, -t_len, -t_len, 0.0, 0.0],
                            [t_wid, t_wid, -t_wid, -t_wid, t_wid]])
        self.array = XYArray(contour)

    def draw(self, axes, hitch_x, hitch_y, trailer_yaw, elems):
        # Homogeneous transformation based on trailer's own yaw and the hitch position
        transformed_array = self.array.homogeneous_transformation(hitch_x, hitch_y, trailer_yaw)
        
        body_plot, = axes.plot(transformed_array.get_x_data(), transformed_array.get_y_data(), 
                               lw=self.spec.line_w, color=self.spec.color, ls=self.spec.line_type)
        elems.append(body_plot)