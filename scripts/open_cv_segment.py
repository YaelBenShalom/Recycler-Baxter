"""! This script is a test of the object segmentation code for open CV.
It is primiarily intended as a demo and should not be used without modification
in the final robot setup.

If problems, contact: Jake
"""

import cv2 as cv
import sys
from os import listdir
import numpy as np

directory_name ="../camera_images/11.10.20/"

file_name = "bottle_top_1.jpg"
img = cv.imread(directory_name + file_name)

if img is None: 
    sys.exit("could not read the image.")

print(img.item(1,1,1)
def paint_circles(image, paint_image, color, min_rad):
    
    grey = cv.cvtColor(img, cv.COLOR_BGR2GRAY)


    grey = cv.medianBlur(grey, 5)

    rows = grey.shape[0]
    circles = cv.HoughCircles(grey, cv.HOUGH_GRADIENT, 1, rows / 8, 
                              param1 = 100, param2 = 30,
                              minRadius = min_rad, maxRadius = 30)

    if circles is not None: 
        circles = np.uint16(np.around(circles))
        for i in circles[0,:]:
            print (i)
            center = (i[0], i[1])
            cv.circle(paint_image, center, 1, (0, 100, 100), 3)
            radius = i[2]
            cv.circle(paint_image, center, radius, color, 3)

    return(paint_image)



paint_image= paint_circles(img, img, (0, 0, 255), 25)


paint_image= paint_circles(img, paint_image, (255, 0, 0), 20)

cv.imshow("detected circles",paint_image )

k = cv.waitKey(0)


