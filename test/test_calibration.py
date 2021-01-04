# !/usr/bin/env python3
""" Unit testing calibration.py """
import rospy
import numpy
import unittest
from can_sort.calibration import Calibration


class TestCalibration(unittest.TestCase):
    """ Tests the python calibration library
    """
    def __init__(self, *args, **kwargs):
        """ Initialize environment
        """
        super(TestCalibration, self).__init__(*args, **kwargs)
        self.point1_pix = [722.5, 937.5, 0]   # 1st calibration point [pixels] - measured from image, z in not relevant
        self.point2_pix = [403.5, 417.5, 0]   # 2nd calibration point [pixels] - measured from image, z in not relevant
        self.point1_measure = [0.55, -0.50, 0]   # 1st calibration point [meters] - measured in lab, z in not relevant
        self.point2_measure = [0.80, -0.10, 0]   # 2nd calibration point [meters] - measured in lab, z in not relevant
        self.calibration = Calibration(self.point1_pix, self.point2_pix)

    def test_calibration_values(self):
        """ Function to check the python calibration library.
        We know the pixel-to-meter conversion for the following points from measurements in the lab:
        point1 = (722.5, 937.5, _)[pixels] = (0.55, -0.50, _)[meters], (z in not relevant)
        point2 = (403.5, 417.5, _)[pixels] = (0.80, -0.10, _)[meters], (z in not relevant)
        So we can check the calibration library using those two points.
        """
        # Find the linearization constants
        a, b, m, n = self.calibration.convert_position()

        # Calculate the new (x, y) for both point using linear equations (in meters)
        # The coordinate system is different between the camera and the robot, so x --> y and y --> x
        x1 = m * self.point1_pix[1] + n
        x2 = m * self.point2_pix[1] + n
        y1 = a * self.point1_pix[0] + b
        y2 = a * self.point2_pix[0] + b

        # Test the calibration
        numpy.testing.assert_almost_equal([x1, y1], [self.point1_measure[0], self.point1_measure[1]], decimal=1)
        numpy.testing.assert_almost_equal([x2, y2], [self.point2_measure[0], self.point2_measure[1]], decimal=1)


if __name__ == "__main__":
    import rosunit
    rosunit.unitrun(can_sort, "test_calibration", TestCalibration)
