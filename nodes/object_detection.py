import rospy
import sys
import cv2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
# from std_msgs.msg import String

"""
bridge = CvBridge()
cv_image = bridge.imgmsg_to_cv2(image_message, desired_encoding='passthrough')
image_message = bridge.cv2_to_imgmsg(cv_image, encoding="passthrough")
TODO

"""

class Classification():
    """
    TODO - Documentaion
    """
    def __init__(self):
        can_diameter = 360 # TODO - mesure [units are pixels]
        bottle_diameter = 120 # TODO - mesure [units are pixels]
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/cameras/???', Image,
                                    self.image_callback, queue_size=1) # TODO - which camera??
        self.capture = cv2.VideoCapture(2)  # TODO - which camera??


    def image_callback(self, image_message): 
        """
        TODO - Documentaion
        """                         
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_message,
                                    desired_encoding='passthrough')
        except CvBridgeError as e:
            print(e)

        (rows,cols,channels) = cv_image.shape


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