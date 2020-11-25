#!/usr/bin/env python3
"""! This python library responsible for the calibration of the camera's output.
The script gets the coordinated of 2 calibration piunts in pixels, and converts
it to meters.
"""

import sympy
from sympy import symbols, Eq, solve


class Calibration():
    def __init__(self, x1_pix, y1_pix, x2_pix, y2_pix):
        pass

    def convert_position(self, x1_pix, y1_pix, x2_pix, y2_pix):
        """ this function takes the 
        """
        a, b, m, n = symbols(r'a, b, m, n')
        eq_x1 = Eq(x1_pix*m + n, 0.51)
        eq_x2 = Eq(x2_pix*m + n, 0.84)
        
        solve((eq_x1, eq_x2), (m, n))

        eq_y1 = Eq(y1_pix*a + b, -0.54)
        eq_y2 = Eq(y2_pix*a + b, -0.06)

        solve((eq_y1, eq_y2), (a, b))

        return a, b, m, n