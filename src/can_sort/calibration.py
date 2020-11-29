#!/usr/bin/env python3
"""! This python library responsible for the calibration of the camera's output.
The script gets the coordinated of 2 calibration points in pixels, and converts
it to meters, using linearization.
The library returns the linearization constants.
"""

import sympy
from sympy import symbols, Eq, solve


class Calibration:
    """! This python library responsible for the calibration of the camera's output.
    The class gets the coordinated of 2 calibration points in pixels, and converts
    it to meters, using linearization.
    """
    def __init__(self, point1, point2):
        """ Initialize environment
        """
        # Using only x and y coordinates (x and y axes are different between the camera and the robot)
        self.y1_pix, self.x1_pix, _ = point1
        self.y2_pix, self.x2_pix, _ = point2

    def convert_position(self):
        """ this function takes the coordinated of 2 calibration points in pixels,
        and converts it to meters using linearization.
        By measuring in the lab, we know that the locations of the calibration points in meters
        are (0.51, -0.54) and (0.84, 0.06).
        """
        # Defining the linearization constants as symbols
        a, b, m, n = symbols(r'a, b, m, n')

        # Equations for the x components (equal to the known x component in meters)
        eq_x1 = Eq(self.x1_pix * m + n, 0.51)
        eq_x2 = Eq(self.x2_pix * m + n, 0.84)

        # Solving the linearization equation for x to find the linearization constants for y
        sol_x = solve((eq_x1, eq_x2), (m, n))
        m, n = sol_x[m], sol_x[n]

        # Equations for the y components (equal to the known y component in meters)
        eq_y1 = Eq(self.y1_pix * a + b, -0.54)
        eq_y2 = Eq(self.y2_pix * a + b, -0.06)

        # Solving the linearization equation for y to find the linearization constants for x
        sol_y = solve((eq_y1, eq_y2), (a, b))
        a, b = sol_y[a], sol_y[b]

        return a, b, m, n
