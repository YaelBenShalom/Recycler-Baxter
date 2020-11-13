#!/usr/bin/env python3
"""! This node is responsible for locating and classifying the sorting objects.

Subscribes:
    /cameras/right_hand_camera - Image, Photos from the baxter hand camera. 

Publishes:
    Nothing Presently. 

Services:
    Nothing Presently.


bridge = CvBridge()
cv_image = bridge.imgmsg_to_cv2(image_message, desired_encoding='passthrough')
image_message = bridge.cv2_to_imgmsg(cv_image, encoding="passthrough")
TODO

"""

import rospy
import sys
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from can_sort.msg import Object
from can_sort.srv import Board


class Classification():
    """! A class for classifying bottles and cans for sorting by baxter.
    A holding class for the node's functions. See file level doc string for 
    more detailed information about the ROS API. 
    """
    def __init__(self):
        # Identification Paramiters
        self.can_diameter = 80       # [units are pixels]
        self.bottle_diameter = 60    # [units are pixels]
        
        # Info for default image 
        self.image_directory ="../camera_images/11.10.20/"
        self.image_name = "bottle_top_1.jpg"
    
        #TODO: figure oout correct commenting style 
        ##! Stores the current image. 
        self.img = self.set_default_image()
        #self.setup_image_stream()

    def setup_image_stream(self):
        """! Initialize the ros subscript to incomming images and CVbridges. 
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
        s = rospy.service('get_board_state', Board, handle_board_state)
        pass

    def set_default_image(self):
        """! Load a default image from the local files. Used for testing. 
        """  
        
        # Read in the image
        self.img = cv.imread(self.image_directory + self.image_name)

        # Check the file name was right
        if self.img is None: 
            rospy.logwarn("WARNING: could not read default image")

    def image_callback(self, image_message): 
        """! Handle a new incomming image from the robot. 
        This function is responsible for processing the image into a an 
        OpenCV friendly format, and for storing it as a class variable.
         
        @param image_message, a ross Image message for processing. 
         
        """         
        rospy.logdebug("Processing incomming image")               
        try:
            cv_image = self.bridge.imgmsg_to_cv(image_message,
                                    desired_encoding='passthrough')
        except CvBridgeError as e:
            rospy.logwarn(e)

        #TODO: store centrally or remove.
        (rows,cols,channels) = cv_image.shape

    def handle_board_state(self, srv):
        """! Run object detection to produce board state from last stored image.
        
        """
        return []
      


    
    def classify_objects(self):
        """
        TODO - Documentaio/n
        """
        # TODO
        pass




def main():
    """ The main() function """
    rospy.init_node('object detection', log_level = rospy.DEBUG)
    rospy.logdebug(f"classification node started")
    classification = Classification()
    while not rospy.is_shutdown():
        rospy.logdebug("Start classifying")
        classification.classify_objects()
    cv.destroyAllWindows()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
