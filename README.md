# Bottle Sorting Baxter
## ME495: Embedded Systems Final Project (Fall 2020)

## Group Members:
- Jake Ketchum
- Kailey Smith
- Yael Ben Shalom
- Mingqing Yuan
- Chris Aretakis


## Project Overview
In this project, we programmed a Rethink Baxter robot to sort bottles and cans located in front of it, ant throw them into separate recycle bins. We used computer vision to detect and locate the variety of randomly placed bottles and cans, and used MoveIt to control the robot.

The Baxter in action:

![Sorting](https://github.com/ME495-EmbeddedSystems/final-project-scott-s-bot-for-tots/blob/master/videos/Baxter_sorting.gif)

## User Guide

### Dependencies Installation
Install the `Intel Realsense` packages as specified at the following link. The libraries are required while the developer and debug packages are on https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages

### Quickstart Guide
...

## System Architecture and High Level Concepts

### Nodes
**object_detection.py** - Object detection node. The node uses the `pyrealsense2` library to get an image from the Realsense deapth camera, and use the computer vision library `OpenCV` to detect and classify the objects in the image.
This node classifies 3 different types of objects:
1. Calibration points - 2 points in known position, used to calibrate the image and convert the units from pixels to meters (painted in green).
2. Cans - Unknown number of cans in unknown positions (painted in red).
3. Bottles - Unknown number of bottles in unknown positions (painted in blue).

After the classification process, the node returns a painted image (with circles in different radiuses and colors for each object), and saves the locaion and classification of the cans and bottles on `Board.srv` service (when called).

The painted image:

![Computer_vision](https://github.com/ME495-EmbeddedSystems/final-project-scott-s-bot-for-tots/blob/master/images/computer_vision.png)

**recycle** - Robot operation node...

The robot motion:

![Sorting](https://github.com/ME495-EmbeddedSystems/final-project-scott-s-bot-for-tots/blob/master/videos/Baxter_sorting2.gif)

**calibration.py** - Python library that responsible for the calibration of the camera's output (converts a point from pixels to meters).
The script gets the coordinated of 2 calibration points in pixels, and converts
it to meters, using linearization.
The library returns the linearization constants:
- x(meters) = m * x(pixels) + n
- y(meters) = a * y(pixels) + b
The `object_detection` node uses this library to convert the points found on the image from pixels to meters.

### Launchfiles
**baxter_move.launch** - The project launchfile...

**camera.launch** - The camera launchfile...

### Test files
**test_calibration.py** - A test file that tests the python calibration library.
The testfile tests the calibration accuracy using 2 known calibration points:
1. point1 = [722.5, 937.5] (pixels) = [0.55, -0.50] (meters)
2. point2 = [403.5, 417.5] (pixels) = [0.80, -0.10] (meters)
The pixels values were measured from the image and the meters values were measured in the lab.

To run the testfile when running catkin_make, run `catkin_make run_tests`.

### Algorithms and Libraries Used
**pyrealsense2** - Library for accessing Intel RealSenseTM cameras.

**OpenCV** - Computer vision library. Used to detect and classify the items on the image.

**MoveIt** - Motion planning library. Used to generate high-degree of freedom trajectories to grab the objects and throw them to the trash bins.

**JTAS** - Joint Trajectory Action Server. Enables executing complex trajectories using software built-in to Baxter

### Physical Equipment:
1. Baxter Rethink robot
2. Realsense D435i depth camera
3. Table
4. 2 trash bins
4. Cans and bottles
5. 3D printed grippers (see CAD image and drawing below)

## Future Work
1. **Use machine learning algorithms for better objects classification** - Now, we can only classify specific shapes of bottles and cans. By using machine learning methods, we could classify different types of bottles and cans with the same lable, and throw them to the same trash bin.
2. **Add the ability to detect more types of items** - Now, we can only detect cans and bottles. In the future, we want to be able to detect and recycle a variety of objects, such as paper or different types of plastic. To do so, we need to improve our computer vision node (to detect those items), and improve our gripper.