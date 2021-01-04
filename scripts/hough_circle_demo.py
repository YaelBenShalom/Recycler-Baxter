#!/usr/bin/env python3

"""! This script is a test of the object segmentation code for open CV.
It is primarily intended as a demo and should not be used without modification
in the final robot setup.
"""

import cv2 as cv
import sys
from os import listdir
import numpy as np
import time
import pyrealsense2 as rs


def remove_table(image, point=(1, 1)):
    """! Subtract the pixel value at the target point from all other pixels. 
    Ideally this works to remove the table coloring from the image making the
    objects we care about sharper. 
    NOTE: This is pretty useless.
    @param image an opencv image. 
    @param point the location to use as a reference. 
    @returns image with all pixels less by the value of probe_point's location.
    """
    table_color = np.full(image.shape, ([image.item(point[0], point[1], 0),
                                         image.item(point[0], point[1], 1),
                                         image.item(point[0], point[1], 2)]))
    table_color = table_color.astype(image.dtype)  # make new image same type as original
    # print(table_color)
    # print(image)
    # cv.imshow("test", image)
    return cv.subtract(image, table_color)


def paint_circles(image, paint_image, color, min_rad, max_rad=10):
    """! This function finds all the specified circles and paints them.
    """
    # cv.imshow("img", img)
    # crop_img = img[400:800, 700:1100]
    # cv.imshow("crop_img", crop_img)
    # Go to greyscale   
    grey = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    _, grey = cv.threshold(grey, 200, 255, cv.THRESH_TRUNC)
    # cv.imshow("th3", grey)

    # Blur the image
    # grey = cv.blur(grey,( 5,5))
    # cv.imshow("blur", grey)
    grey = cv.medianBlur(grey, 5)
    # cv.imshow("medianBlur", grey)
    grey = cv.GaussianBlur(grey, (5, 5), 0)
    # cv.imshow("GaussianBlur", grey)

    # Run the algorithm, get the circles
    rows = grey.shape[0]
    circles = cv.HoughCircles(grey, cv.HOUGH_GRADIENT, 1, rows / 16,
                              param1=100, param2=30,
                              minRadius=min_rad, maxRadius=max_rad)

    # Paint the circles onto our paint image
    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            print("circle: ", i)
            center = (i[0], i[1])
            cv.circle(paint_image, center, 1, (0, 100, 100), 3)
            radius = i[2]
            cv.circle(paint_image, center, radius, color, 3)

    return paint_image


# Configure color stream
pipeline = rs.pipeline()
config = rs.config()
# config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Recording video to bagfile
# config.enable_record_to_file("bagfiles/camera_video2")  # Comment this if you want to work of saved bagfile
config.enable_device_from_file("bagfiles/camera_video")  # Uncomment this if you want to work of saved bagfile

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
        height, width = img.shape[:2]
        img = cv.resize(img, (int(2.5 * width), int(2.5 * height)), interpolation=cv.INTER_CUBIC)
        # img = img[390:790, 680:1080]
        # Check the file name was right
        if img is None:
            sys.exit("""could not read the image. Make sure you are running 
            the script from the /scripts folder.""")

        # Find cans - blue
        paint_image = paint_circles(img, img, (0, 0, 255), 21, 26)

        # Find bottles - red
        paint_image = paint_circles(img, paint_image, (255, 0, 0), 10, 16)

        remove_table(img)

        # Show image
        cv.namedWindow("detected_circle", cv.WINDOW_AUTOSIZE)
        cv.imshow("detected_circles", paint_image)
        key = cv.waitKey(1)

        # Press esc or 'q' to close the image window
        if key & 0xFF == ord('q') or key == 27:
            cv.destroyAllWindows()
            break

finally:
    # Stop streaming
    pipeline.stop()
