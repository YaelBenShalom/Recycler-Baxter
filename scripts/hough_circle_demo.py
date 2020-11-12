"""! This script is a test of the object segmentation code for open CV.
It is primiarily intended as a demo and should not be used without modification
in the final robot setup.

If problems, contact: Jake
"""

import cv2 as cv
import sys
from os import listdir
import numpy as np

# name of the file
file_name = "bottle_top_1.jpg"


# Name of the directory where our images live.
directory_name ="../camera_images/11.10.20/"

# Read in the image
img = cv.imread(directory_name + file_name)

# Check the file name was right
if img is None: 
    sys.exit("could not read the image.")

print(img.item(1,1,1))#ignore this


def paint_circles(image, paint_image, color, min_rad, max_rad = 30):
    """! This function finds all the specified circles and paints them.
    """ 
    # Go to greyscale   
    grey = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    # Blur the image, this was in the tutorial who knows why
    grey = cv.medianBlur(grey, 5)
    
    # Run the algorithm, get the circles
    rows = grey.shape[0]
    circles = cv.HoughCircles(grey, cv.HOUGH_GRADIENT, 1, rows / 8, 
                              param1 = 100, param2 = 30,
                              minRadius = min_rad, maxRadius = max_rad)
    
    # Paint the circles onto our paint image
    if circles is not None: 
        circles = np.uint16(np.around(circles))
        for i in circles[0,:]:
            print (i)
            center = (i[0], i[1])
            cv.circle(paint_image, center, 1, (0, 100, 100), 3)
            radius = i[2]
            cv.circle(paint_image, center, radius, color, 3)

    return(paint_image)


# Find cans  
paint_image= paint_circles(img, img, (0, 0, 255), 25)


# Find bottles
paint_image= paint_circles(img, paint_image, (255, 0, 0), 20)

cv.imshow("detected circles",paint_image )

k = cv.waitKey(0)


