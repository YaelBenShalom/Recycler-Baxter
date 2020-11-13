#!/usr/bin/env python3

"""! This script is a test of the object segmentation code for open CV.
It is primiarily intended as a demo and should not be used without modification
in the final robot setup.
If problems, contact: Jake
"""

import cv2 as cv
import sys
from os import listdir
import numpy as np
import time
import pyrealsense2 as rs


# Configure color stream
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Recording video to bagfile
config.enable_record_to_file("bagfiles/camera_video")

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
        img = np.asanyarray(color_frame.get_data())

        # Check the file name was right
        if img is None: 
            sys.exit("""could not read the image. Make sure you are running 
            the script from the /scripts folder.""")


        def remove_table(image, point = (1,1)):
            """! Subtract the pixel value at the target point from all other pixels. 
            Ideally this works to remove the table coloring from the image making the
            objects we care about sharper. 
            NOTE: This is pretty useless.
            @param image an opencv image. 
            @param point the location to use as a reference. 
            @returns image with all pixels less by the value of probe_point's location.
            """
            table_color = np.full(image.shape,([image.item(point[0],point[1],0),
                                                image.item(point[0],point[1],1),
                                                image.item(point[0],point[1],2)]))
            table_color = table_color.astype(image.dtype) # make new image same type as original
            print(table_color)
            print(image)
            cv.imshow("test", image)
            return(cv.subtract(image, table_color))

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
                for i in circles[0, :]:
                    print (i)
                    center = (i[0], i[1])
                    cv.circle(paint_image, center, 1, (0, 100, 100), 3)
                    radius = i[2]
                    cv.circle(paint_image, center, radius, color, 3)

            return(paint_image)


        # Find cans  
        paint_image = paint_circles(img, img, (0, 0, 255), 25)

        # Find bottles
        paint_image = paint_circles(img, paint_image, (255, 0, 0), 20)

        # Find bottle tabs
        paint_image = paint_circles(img, paint_image, (0, 255, 0), 20, max_rad = 10)

        remove_table(img)

        # Show image
        cv.namedWindow('detected_circles', cv.WINDOW_AUTOSIZE)
        cv.imshow("detected_circles", paint_image)
        key = cv.waitKey(1)

        # Press esc or 'q' to close the image window
        if key & 0xFF == ord('q') or key == 27:
                    cv.destroyAllWindows()
                    break

finally:
    # Stop streaming
    pipeline.stop()