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
import cv2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image, CameraInfo, CompressedImage



class Classification():
    """! A class for classifying bottles and cans for sorting by baxter.
    A holding class for the node's functions. See file level doc string for 
    more detailed information about the ROS API. 
    """
    def __init__(self):
        # Identification Paramiters
        self.can_diameter = 80       # [units are pixels]
        self.bottle_diameter = 60    # [units are pixels]
    
        ##! Stores the current image. 
        self.image = self.set_default_image()
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
        self.capture = cv2.VideoCapture(2)  # TODO - which camera??

    def setup_services(self):
        """! Set up the the ros services provided by this node. 
        This should be called after all other setup has been performed to
        prevent invalid service calls.
        """
        rospy.loginfo("Setting up services")
        pass


    def image_callback(self, image_message): 
        """! Handle a new incomming image from the robot. 
        This function is responsible for processing the image into a an 
        OpenCV friendly format, and for storing it as a class variable.
         
        @param image_message, a ross Image message for processing. 
         
        """         
        rospy.logdebug("Processing incomming image")               
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_message,
                                    desired_encoding='passthrough')
        except CvBridgeError as e:
            rospy.logwarn(e)

        #TODO: store centrally or remove.
        (rows,cols,channels) = cv_image.shape

    def handle_board_state(self, srv):
        """! Run object detection to produce board state from last stored image.
        
        """
      


    
    def classify_objects(self):
        """
        TODO - Documentaion
        """
        # TODO
        pass




def main():
    """ The main() function """
    rospy.init_node('object detection', log_level = rospy.DEBUG)
    rospy.logdebug(f"classification node started")
    classification = Classification()
    while not rospy.is_shutdown():
        rospy.logdebug(f"Start classifying")
        classification.classify_objects()
    rospy.spin()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
