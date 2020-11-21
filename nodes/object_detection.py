#!/usr/bin/env python3
"""! This node is responsible for locating and classifying the sorting objects.

Subscribes:
    /cameras/right_hand_camera - Image, Photos from the baxter hand camera. 

Publishes:
    Nothing Presently. 

Services:
    Nothing Presently.

TODO
"""

import rospy
import rospkg
import sys
import os
import numpy as np
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from can_sort.msg import Object
from can_sort.srv import Board, BoardResponse
import pyrealsense2 as rs


class Detect():
    """! A class for classifying bottles and cans for sorting by baxter.
    A holding class for the node's functions. See file level doc string for 
    more detailed information about the ROS API. 
    """

    def __init__(self):
        # Identification Parameters
        # TODO - replace with yaml parameters
        self.can_diameter_min = 21       # [units are pixels]
        self.can_diameter_max = 26       # [units are pixels]
        self.bottle_diameter_min = 10    # [units are pixels]
        self.bottle_diameter_max = 15    # [units are pixels]
        # self.can_diameter_min = rospy.get_param("can_diameter_min")             # Initializing cans minimum diameter [pixels]
        # self.can_diameter_max = rospy.get_param("can_diameter_max")             # Initializing cans maximum diameter [pixels]
        # self.bottle_diameter_min = rospy.get_param("bottle_diameter_min")       # Initializing bottles minimum diameter [pixels]
        # self.bottle_diameter_max = rospy.get_param("bottle_diameter_max")       # Initializing bottles maximum diameter [pixels]

        # Object Type Definitions
        # TODO - replace with yaml parameters
        self.ERROR = -1     # Initializing object type - error
        self.BOTTLE = 0     # Initializing object type - bottle
        self.CAN = 1        # Initializing object type - can
        # self.ERROR = rospy.get_param("ERROR")       # Initializing object type - error
        # self.BOTTLE = rospy.get_param("BOTTLE")     # Initializing object type - bottle
        # self.CAN = rospy.get_param("CAN")           # Initializing object type - can
        
        # self.R = rospy.get_param("~pub_freq")           # initializing the frequency at which to publish messages
        self.rate = rospy.Rate(100)
        self.detection_mode = True

        # # Info for default image 
        # self.rospack = rospkg.RosPack()
        # path = self.rospack.get_path(name = 'can_sort') + '/camera_images/11.10.20/'
        # self.image_directory = path
        # self.image_name = "bottle_top_1.jpg"
    
        # #TODO: figure out correct commenting style 
        # ##! Stores the current image. 
        # self.set_default_image()
        
        # #self.setup_image_stream()


        # Configure color stream
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Recording video to bagfile
        # config.enable_record_to_file("bagfiles/camera_video2")  # Comment this if you want to work of saved bagfile
        config.enable_device_from_file("bagfiles/camera_video") # Uncomment this if you want to work of saved bagfile

        # Start streaming
        pipeline.start(config)
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
                self.img = cv.resize(self.img, (int(2.5*width), int(2.5*height)), interpolation = cv.INTER_CUBIC)
                # self.img = self.img[390:790, 680:1080]
                # Check the file name was right
                if self.img is None: 
                    sys.exit("""could not read the image. Make sure you are running 
                    the script from the /scripts folder.""")

                # Show image
                cv.namedWindow("detected_circle", cv.WINDOW_AUTOSIZE)
                cv.imshow("detected_circles", self.img)
                key = cv.waitKey(1)

                # Press esc or 'q' to close the image window
                if key & 0xFF == ord('q') or key == 27:
                            cv.destroyAllWindows()
                            break

        finally:
            # Stop streaming
            pipeline.stop()


    def setup_image_stream(self):
        """! Initialize the ros subscript to incoming images and CVbridges. 
        This is required for normal operations since this is how the node
        gets it's images for processing. However, during unit testing other
        means of loading images may be appropriate. 
        """
        rospy.loginfo("Setting up image stream")
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/cameras/right_hand_camera', Image,
                                    self.image_callback, queue_size=1)
        self.capture = cv.VideoCapture(2)  # TODO - which camera??


    def setup_services(self):
        """! Set up the the ros services provided by this node. 
        This should be called after all other setup has been performed to
        prevent invalid service calls.
        """
        rospy.loginfo("Setting up services")
        state = rospy.ServiceProxy('get_board_state', Board, self.get_board_state) # Call the get_board_state service


    # def set_default_image(self):
    #     """! Load a default image from the local files. Used for testing. 
    #     """  
    #     # Read in the image
    #     self.img = cv.imread(self.image_directory + self.image_name)
    #     # Check the file name was right
    #     if self.img is None: 
    #         rospy.logwarn("WARNING: could not read default image")


    def image_callback(self, image_message): 
        """! Handle a new incoming image from the robot. 
        This function is responsible for processing the image into a an 
        OpenCV friendly format, and for storing it as a class variable.
         
        @param image_message, a ross Image message for processing. 
        """         
        rospy.logdebug("Processing incoming image")               
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_message,
                                    desired_encoding='passthrough')
        except CvBridgeError as e:
            rospy.logwarn(e)

        #TODO: store centrally or remove.
        (rows,cols,channels) = cv_image.shape


    def get_board_state(self, srv):
        """! Run object detection to produce board state from stored image.
        Currently incomplete 
        """

        # This is just the script for testing, everything here needs to change
        # Set local so no change during run
        img = self.img
        
        rospy.loginfo("calling service")
        rospy.wait_for_service('get_board_state')
        state_service = rospy.ServiceProxy('get_board_state', Board, self.get_board_state) # Call the get_board_state service

        objects = [] 
        objects.extend(self.detect_cans(img))
        objects.extend(self.detect_bottles(img))
        print(objects)
        
        # # Find cans - blue
        # paint_image = self.paint_circles(img, img, (0, 0, 255), 21, 26)

        # # Find bottles - red
        # paint_image = self.paint_circles(img, paint_image, (255, 0, 0), 10, 15)

        # # Show image
        # cv.namedWindow("detected_circles", cv.WINDOW_AUTOSIZE)
        # cv.imshow("detected_circles", paint_image)
        # key = cv.waitKey(1)

        # # Press esc or 'q' to close the image window
        # if key & 0xFF == ord('q') or key == 27:
        #     cv.destroyAllWindows()

        return BoardResponse(objects)
      

    def detect_cans(self, image):
        """! This function detects cans located on the table.
        Inputs:
          Image (img) - the stored image.
        
        Returns:
          Cans (list) - A list of cans' state
                       (type - CAN, sorted - False, location)
        """
        rospy.loginfo("Detecting Cans")
        circles = self.detect_circles(image, self.can_diameter_min, self.can_diameter_max)
        rospy.loginfo(f"circles = {circles}")
        cans = []
        for c in circles[0]:
            can = Object()
            can.type = self.CAN
            can.sorted = False 
            can.location.x = c[0]
            can.location.y = c[1]
            can.location.z = -1 #TODO: Implement a decent vertical offset 
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
        rospy.loginfo("Detecting Bottles")
        circles = self.detect_circles(image, self.bottle_diameter_min, self.bottle_diameter_max)
        rospy.loginfo(f"circles = {circles}")
        bottles = []
        for c in circles[0]:
            bottle = Object()
            bottle.type = self.BOTTLE
            bottle.sorted = False 
            bottle.location.x = c[0]
            bottle.location.y = c[1]
            bottle.location.z = -1 #TODO: Implement a decent vertical offset 
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

        # Blur the image
        grey_med_blur = cv.medianBlur(grey_thresh, 5)
        grey_gauss_blur = cv.GaussianBlur(grey_med_blur, (5,5), 0)

        # Run the algorithm, get the circles
        rows = grey_gauss_blur.shape[0]
        circles = cv.HoughCircles(grey_gauss_blur, cv.HOUGH_GRADIENT, 1, rows / 16, 
                                param1 = 100, param2 = 30,
                                minRadius = min_dia, maxRadius = max_dia)
        return circles


    def paint_circles(self, image, paint_image, color, min_dia, max_dia = 10):
        """! This function finds all the specified circles and paints them.
        It is for testing purpose only
        """         
        # Go to greyscale   
        grey = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        _, grey_thresh = cv.threshold(grey, 200, 255, cv.THRESH_TRUNC)

        # Blur the image
        grey_med_blur = cv.medianBlur(grey_thresh, 5)
        grey_gauss_blur = cv.GaussianBlur(grey_med_blur, (5,5), 0)

        # Run the algorithm, get the circles
        rows = grey_gauss_blur.shape[0]
        circles = cv.HoughCircles(grey_gauss_blur, cv.HOUGH_GRADIENT, 1, rows / 16, 
                                param1 = 100, param2 = 30,
                                minRadius = min_dia, maxRadius = max_dia)

        # Paint the circles onto our paint image
        if circles is not None: 
                circles = np.uint16(np.around(circles))
                for i in circles[0, :]:
                    print ("circle: ", i)
                    center = (i[0], i[1])
                    cv.circle(paint_image, center, 1, (0, 100, 100), 3)
                    diameter = i[2]
                    cv.circle(paint_image, center, diameter, color, 3)
        return circles, paint_image


    def run_detection(self):
        while not rospy.is_shutdown():
            rospy.logdebug(f"Run Message")
            if self.detection_mode:
                self.setup_services()
                self.rate.sleep()
                cv.destroyAllWindows()
            else:
                rospy.logdebug(f"In pause mode")
            self.rate.sleep()
        


def main():
    """ The main() function """
    rospy.init_node("object_detection", log_level = rospy.DEBUG)
    rospy.logdebug(f"classification node started")
    detect = Detect()
    detect.run_detection()
    rospy.spin()

    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
