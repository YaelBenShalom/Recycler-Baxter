 
#!/usr/bin/env python3

""" Unit testing calibration.py """
import rospy
import numpy
import unittest
# from can_sort.calibration import Calibration # TODO

class TestCalibration(unittest.TestCase):
    """ Checks the python calibration package
    """
    def __init__(self, *args, **kwargs): # TODO
        super(TestCalibration, self).__init__(*args, **kwargs)
    #     W = 8           # initializing the width of the figure eight
    #     H = 5           # initializing the height of the figure eight
    #     self.T = 60     # initializing the amount of time it takes to complete the figure eight
    #     self.figure_eight = FigureEight(self.T, H, W)

    # def test_equal_values_on_init(self):
    #     """ Function to check the figure eight calculations at time t = 0 sec
    #     """
    #     numpy.testing.assert_almost_equal(self.figure_eight.test_output(0), (0, 0, 0.418879020478639, 0.523598775598299, 0, 0, 0.670533602144726, 0), decimal = 2)

    # def test_equal_values_after_half_cycle(self):
    #     """ Function to check the figure eight calculations at time t = T/2 = 30 sec
    #     """
    #     numpy.testing.assert_almost_equal(self.figure_eight.test_output(self.T/2), (0, 0, -0.418879020478639, 0.523598775598299, 0, 0, 0.670533602144726, 0), decimal = 2)

if __name__ == "__main__":
    import rosunit
    rosunit.unitrun(can_sort, "test_calibration", TestCalibration)