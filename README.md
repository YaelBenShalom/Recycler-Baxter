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
Install the Intel Realsense packages as specified at the following link. The libraries are required while the developer and debug packages are on https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages

### Quickstart Guide
...

## System Architecture and High Level Concepts

### Nodes
**object_detection** - Object detection node...

**recycle** - Robot operation node...

**calibration.py** - Python library that responsible for the calibration of the camera's output (from pixels to meters).
The script gets the coordinated of 2 calibration points in pixels, and converts
it to meters, using linearization.
The library returns the linearization constants.

### Launchfiles
**baxter_move.launch** - The project launchfile...

**camera.launch** - The camera launchfile...

### Test files
**test_calibration** - A test file that tests the python calibration library.

### Algorithms and Libraries Used
**OpenCV** - Computer vision library...

**MoveIt** - Motion planning library...

### Physical Equipment:
1. Baxter Rethink robot
2. Realsense D435i depth camera
3. Table
4. 2 trash bins
4. Cans and bottles
5. 3D printed grippers (see CAD image and drawing below)

## Future Work
1. Use machine learning algorithms to detect and classify the objects.
2. Add the ability to detect more types of items.