#! /usr/bin/python3

# Copyright (c) 2013-2015, Rethink Robotics
# All rights reserved.
#
# Code Modified from Rethink Robotics:

import os
import sys
import argparse
import rospy
import cv2
import cv_bridge
from sensor_msgs.msg import Image
from can_sort.srv import DisplayImage, DisplayImageResponse


class Display_Image:
    """ A class for sending images of the current object to be recyled to Baxter's Display
    """
    def __init__(self):
        """ Initialize environment
        """
        self.display_srv = rospy.Service('display_image', DisplayImage, self.send_image_srv)
        rospy.logdebug(f"Service initialized")

    def send_image_srv(self, req):
        """
        Send the image located at the specified path to the head
        display on Baxter.
        :param path: path to the image file to load and send
        Args:
          req: Service request, an integer determinig which image to display
        
        Returns:
          DisplayImageResponse() (srv): returns a service respond that displays an image on Baxter's Display
        
        """
        #### WE WANT TO PASS A RELATIVE PATH INSTEAD OF USING THE ABOLUTE PATH
        if req.img_num == 1:
            path = "/images/head_images/recycle.jpg"

        elif req.img_num == 2:
            path = "/images/head_images/dr_pepper.png"

        elif req.img_num == 3:
            path = "/images/head_images/juice.png"

        elif req.img_num == 4:
            path = "/images/head_images/happy_bin.png"

        elif req.img_num == 5:
            path = "/images/head_images/mystery_man.png"

        img = cv2.imread(path)
        msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
        pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)
        pub.publish(msg)
        rospy.sleep(1)
        return DisplayImageResponse()

#Main Loop
def main():
    rospy.init_node('disp_img', anonymous=True)
    display = Display_Image()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
