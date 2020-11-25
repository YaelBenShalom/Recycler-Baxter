#!/usr/bin/env python3
"""! This node is responsible for locating and classifying the objects located in front
of the robot.

Publishes:
    image_pub - published the circled image after detecting the bottles and cans 

Services:
    state_srv - a service containing information about the objects type (-1, 0, 1),
                have they been sorted yet (True, False), and their location (x, y, z,
                while the z coordinate is always -1).
"""

import rospy
import rospkg
import sys
import os
import numpy as np
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
from can_sort.msg import Object
from can_sort.srv import Board, BoardResponse
import pyrealsense2 as rs
from can_sort.calibration import Calibration


class Detect():
    """! A class for classifying bottles and cans for sorting by baxter.
    A holding class for the node's functions. See file level doc string for 
    more detailed information about the ROS API. 
    """

    def __init__(self):
        # Identification Parameters
        # TODO - replace with yaml parameters
        # self.can_diameter_min = 30       # [units are pixels]
        # self.can_diameter_max = 40       # [units are pixels]
        # self.bottle_diameter_min = 25    # [units are pixels]
        # self.bottle_diameter_max = 28    # [units are pixels]
        self.can_diameter_min = rospy.get_param("can_diameter_min")             # Initializing cans minimum diameter [pixels]
        self.can_diameter_max = rospy.get_param("can_diameter_max")             # Initializing cans maximum diameter [pixels]
        self.bottle_diameter_min = rospy.get_param("bottle_diameter_min")       # Initializing bottles minimum diameter [pixels]
        self.bottle_diameter_max = rospy.get_param("bottle_diameter_max")       # Initializing bottles maximum diameter [pixels]

        # Object Type Definitions
        # TODO - replace with yaml parameters
        # self.ERROR = -1     # Initializing object type - error
        # self.BOTTLE = 0     # Initializing object type - bottle
        # self.CAN = 1        # Initializing object type - can
        self.ERROR = rospy.get_param("ERROR")       # Initializing object type - error
        self.BOTTLE = rospy.get_param("BOTTLE")     # Initializing object type - bottle
        self.CAN = rospy.get_param("CAN")           # Initializing object type - can
        
        # self.R = rospy.get_param("~pub_freq")           # initializing the frequency at which to publish messages
        self.rate = rospy.Rate(100)
        self.detection_mode = True
        self.objects = []

        #initialize publisher
        self.image_pub = rospy.Publisher('/image_out', Image, queue_size=1)
        rospy.logdebug(f"Publisher initialized")
        
        #initialize service
        self.state_srv = rospy.Service('board_state', Board, self.get_board_state)
        rospy.logdebug(f"Service initialized")

        self.bridge = CvBridge()


    def image_processing(self):
        """! Process a new incoming image from the realsense. 
        This function is responsible for processing the image into a an 
        OpenCV friendly format, and for storing it as a class variable.
        Inputs:
          None
        
        Returns:
          Showing the circled image.
          Returns nothing. 
        """    
        # Configure color stream
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 6)

        # Recording video to bagfile
        config.enable_record_to_file("camera_video3")  # Comment this if you want to work of saved bagfile
        # config.enable_device_from_file("camera_video")  # Uncomment this if you want to work of saved bagfile
        # config.enable_device_from_file("bagfiles/camera_video")  # Uncomment this if you want to work of saved bagfile

        # Start streaming
        pipeline.start(config)

        # Waiting for the image to stabilize
        rospy.sleep(2)

        try:
            while True:
                # Wait for a coherent color frame
                frames = pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                if not color_frame:
                    continue

                # Convert image to numpy array
                self.img = np.asanyarray(color_frame.get_data())
                height, width = self.img.shape[:2]
                self.img = cv.resize(self.img, (int(2*width), int(2*height)), interpolation = cv.INTER_CUBIC)
                self.img = self.img[500:1500, 1500:2700]

                # Check the file name was right
                if self.img is None: 
                    sys.exit("""could not read the image""")

                paint_image = self.paint_circles(self.img, self.img, (0, 255, 0), 10, 20)

                # Find cans - blue
                paint_image = self.paint_circles(self.img, self.img, (0, 0, 255), self.can_diameter_min, self.can_diameter_max)

                # Find bottles - red
                paint_image = self.paint_circles(self.img, paint_image, (255, 0, 0), self.bottle_diameter_min, self.bottle_diameter_max)
                
                img_out = self.bridge.cv2_to_imgmsg(paint_image, "bgr8")
                self.image_pub.publish(img_out)

                # Show image
                cv.namedWindow("detected_circles", cv.WINDOW_AUTOSIZE)
                cv.imshow("detected_circles", paint_image)
                key = cv.waitKey(1)
                
                # Press esc or 'q' to close the image window
                if key & 0xFF == ord('q') or key == 27:
                    cv.destroyAllWindows()
                    break

        finally:
            # Stop streaming
            pipeline.stop()


    def get_board_state(self, srv):
        """! Run object detection to produce board state from stored image.
        Inputs:
          service (srv) - the board_state service.
        
        Returns:
          BoardResponse (lisrvst) - returning a service response of the objects type and location
        """
        # This is just the script for testing, everything here needs to change
        # Set local so no change during run
        rospy.logdebug(f"get_board_state service")
        img = self.img
        self.objects = []

        self.detect_calibration_points(img)

        response = BoardResponse()
        if len(self.detect_cans(img)) !=0:
          self.objects.extend(self.detect_cans(img))
        if len(self.detect_bottles(img)) != 0:
          self.objects.extend(self.detect_bottles(img))
        response.objects = self.objects
        print(f"response is {response}")

        return response


    def detect_calibration_points(self, image):
        """! This function detects bottles located on the table.
        Inputs:
          Image (img) - the stored image.
        
        Returns:
          Bottles (list) - A list of bottles' state
                          (type - BOTTLE, sorted - False, location)
        """
        rospy.logdebug(f"Detecting Calibration Points")
        circles = self.detect_circles(image, 10, 20)
        print ("circles: ", circles)
        return self.a, self.b, self.m, self.n


    def detect_cans(self, image):
        """! This function detects cans located on the table.
        Inputs:
          Image (img) - the stored image.
        
        Returns:
          Cans (list) - A list of cans' state
                       (type - CAN, sorted - False, location)
        """
        rospy.logdebug(f"Detecting Cans")
        circles = self.detect_circles(image, self.can_diameter_min, self.can_diameter_max)
        rospy.logdebug(f"circles = {circles}")
        cans = []
        for c in circles[0]:
            can = Object()
            can.type = self.CAN
            can.sorted = False 
            can.location.x = -0.000780212*c[1] + 1.112115
            can.location.y = -0.0007730055*c[0] + 0.2219635
            can.location.z = -1 # This value will be overwrite be the motion node
            cans.append(can)

        return cans


    def detect_bottles(self, image):
        """! This function detects bottles located on the table.
        Inputs:
          Image (img) - the stored image.
        
        Returns:
          Bottles (list) - A list of bottles' state
                          (type - BOTTLE, sorted - False, location)
        """
        rospy.logdebug(f"Detecting Bottles")
        circles = self.detect_circles(image, self.bottle_diameter_min, self.bottle_diameter_max)
        rospy.logdebug(f"circles = {circles}")
        bottles = []
        for c in circles[0]:
            bottle = Object()
            bottle.type = self.BOTTLE
            bottle.sorted = False 
            bottle.location.x = -0.000780212*c[1] + 1.112115
            bottle.location.y = -0.0007730055*c[0] + 0.2219635
            bottle.location.z = -1 # This value will be overwrite be the motion node
            bottles.append(bottle)
            
        return bottles


    def detect_circles(self, image, min_dia = 10, max_dia = 30):
        """! This function detects circle pattern in the stored image 
        Inputs:
          Image (img) - the stored image.
          min_dia (int) - the object minimum diameter
          max_dia (int) - the object maximum diameter

        Returns:
          circles (list) - A list of the circles in the image
        """
        # Go to greyscale   
        grey = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        _, grey_thresh = cv.threshold(grey, 200, 255, cv.THRESH_TRUNC)

        # Add blur to the image for better detection
        grey_blur = cv.medianBlur(grey_thresh, 5)
        grey_blur2 = cv.GaussianBlur(grey_blur, (5,5), 0)

        # Run the algorithm, get the circles
        rows = grey_blur2.shape[0]
        circles = cv.HoughCircles(grey_blur2, cv.HOUGH_GRADIENT, 1, rows / 20, 
                                param1 = 100, param2 = 30,
                                minRadius = min_dia, maxRadius = max_dia)
        return circles


    def paint_circles(self, image, paint_image, color, min_dia, max_dia = 10):
        """! This function finds all the circles with a specified diameter and paints them.
        Inputs:
          image (img) - the stored image.
          paint_image (img) - the circle-painted image returned last time the function was called
                              (If the function was never called before, image = paint_image).
          color (rgb) - the desired color for the painted circles
          min_dia (int) - the object minimum diameter
          max_dia (int) - the object maximum diameter

        Returns:
          paint_image (img) - A circle-painted image.
        """     
        # Finding the circles in the image    
        circles = self.detect_circles(image, min_dia, max_dia)
        
        # Paint the circles onto our paint image
        if circles is not None: 
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                # print ("circle: ", i)
                center = (i[0], i[1])
                cv.circle(paint_image, center, 1, (0, 100, 100), 3)
                cv.circle(paint_image, center, i[2], color, 3)

        return paint_image       


def main():
    """ The main() function """
    rospy.init_node("object_detection", log_level = rospy.DEBUG)
    rospy.logdebug(f"classification node started")
    detect = Detect()
    detect.image_processing()
    # rospy.spin()
    rospy.sleep(1)
    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
