# Recycle Sorting Baxter
## ME495: Embedded Systems Final Project (Fall 2020)

## Group Members:
- Jake Ketchum
- Kailey Smith
- Yael Ben Shalom
- Mingqing Yuan
- Chris Aretakis


## Project Overview
In this project, we programmed a Rethink Baxter robot to sort bottles and cans located in front of it, and drop them into separate recycle bins. We used computer vision to detect and locate a couple of randomly placed bottles and cans, and used MoveIt to control the robot.

The Baxter in action:

![Sorting](https://github.com/ME495-EmbeddedSystems/final-project-scott-s-bot-for-tots/blob/master/videos/Baxter_sorting.gif)

View the full demo [here](https://drive.google.com/file/d/10anAmP4I5A87bw1hqXWV8cNA236D1xCu/view?usp=sharing):

## User Guide

### Dependencies Installation
Install the `Intel Realsense` packages as specified at the following link. The libraries are required while the developer and debug packages are on https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages

```
sudo apt install python3-sphinx
sudo apt install ros-noetic-rosdoc-lite
```

### Quickstart Guide
* In the `/src` directory of your catkin workspace, download `can_sort.rosinstall`
* While still in the `/src` directoty run `wstool init` to initalize the workspace
* To merge the workspace with the rosinstall file, run `wstool merge can_sort.rosinstall`
* To ensure you have the latest version of all packages, run `wstool update`
* Source and `catkin_make` your workspace
* To use the Baxter, plug its ethernet cord into your computer
* To connect to the Baxter, move to your workspace directory and run `source src/can_sort/scripts+pseudocode/Baxter_setup.bash ` 
  * To ensure you have connectect sucessfully, run `ping baxter.local`
* Enable the robot using `rosrun baxter_tools enable_robot.py -e`
  * If you are having issues connecting to Baxter, please follow the full instructions outlined [here](https://nu-msr.github.io/me495_site/lecture13_rethink.html).
* To start soring run `rosrun baxter_interface joint_trajectory_action_server.py &`
* Then run `roslaunch roslaunch can_sort baxter_move.launch object_detection:=true`
* Watch in awe as the Baxter sorts and recycles cans and bottles!

## System Architecture and High Level Concepts
### Nodes
**object_detection.py** - Object detection node. The node uses the `pyrealsense2` library to get an image from the Realsense depth camera, and use the computer vision library `OpenCV` to detect and classify the objects in the image.
This node classifies 3 different types of objects:
1. Calibration points - 2 points in known position, used to calibrate the image and convert the units from pixels to meters (painted in green).
2. Cans - Unknown number of cans in unknown positions (painted in red).
3. Bottles - Unknown number of bottles in unknown positions (painted in blue).

After the classification process, the node returns a painted image (with circles in different radiuses and colors for each object), and saves the location and classification of the cans and bottles on `Board.srv` service (when called).

To call the `Board.srv` service, open a new terminal and run `rosservice call /board_state "use_real: true"` when the `object_detection` node is running.

The painted image:

![Computer_vision](https://github.com/ME495-EmbeddedSystems/final-project-scott-s-bot-for-tots/blob/master/images/computer_vision.png)


**recycle.py** - Robot operation node. This node uses ROS's `MoveIt!` library for motion planning and manipulation (mainly the `compute_cartesian_path` command). After initalizing the move group commander for Baxter's right arm, this node adds a table to the planning scence to ensure that the robot does not collide with the table. A proxy `DisplayImage.srv` service is also created. The arm then moves to a predetermined position out of the camera's field of view and calls the `Board.srv` service. This service returns a list with the position and classification of all bottles and cans in the camera's view. In the last portion of the set-up the robot moves to a predetermined orientation to ensure smooth and predictable motion of the arm (This desired configuration was determined after testing).

With the objects locations and classifications known, the robot then works through a while loop for the entirety of the list length. The loop functions as follows:
1. Move to the home position where the robot is safely above all objects.
2. For the current item in the list, display either the can image or the bottle image, depending on the classification.
3. Next, move to the object's (x,y) corrdinate at a safe z height away. This is the same height for bottles and cans.
4. Then move down to the appropriate perch height, depending on classification. (For example, the robot arm will be position further away from the table for bottle, since those are taller than cans).
5. Once safely at the perch height, move down so that the object is in between the grippers. 
6. Grasp the object.
7. Move back up to the "safe" position as step 3.
8. Move back to the home position. This step was added to ensure predictable behavior of the robot arm. 
9. Depending on the object's classification, move to the appropriate bin. Also, display the recylcing image.
10. Once over the bin, open the grippers and drop the object. Show that the object has been recycled with the bin image. 
11. Repeat for all objects found. 

The robot motion:

![Sorting](https://github.com/ME495-EmbeddedSystems/final-project-scott-s-bot-for-tots/blob/master/videos/Baxter_sorting2.gif)


**disp_img.py** - Displays an image on the Baxter's head display.
The node converts the image to imgmsg (using OpenCV), and publishes the message to the `/robot/xdisplay` display using `DisplayImage.srv` service.


**calibration.py** - Python library that responsible for the calibration of the camera's output (converts a point from pixels to meters).
The script gets the coordinated of 2 calibration points in pixels, and converts
it to meters, using linearization.
The library returns the linearization constants:
- x(meters) = m * x(pixels) + n
- y(meters) = a * y(pixels) + b

The `object_detection` node uses this library to convert the points found on the image from pixels to meters.


### Launchfiles
**baxter_move.launch** - This launch file launches both recyle node and object_detection node. The recyle node runs along with joint_trajectory_server which is required in to plan the trajectory in MoveIt. Also, this launch file includes two files (`baxter_grippers.launch` and `trajectory_execution.launch`) from baxter_moveit_config which is in the MoveIt! Robots package. 

**camera.launch** - This launch file launches The object_detection node (including lauding the parameter server). This launch file is for test and debug purposes only, because it does not activate the entire system. To activate the entire system, run the `baxter_move.launch` launch file.


### Test files
**test_calibration.py** - A test file that tests the python calibration library.
The testfile tests the calibration accuracy using 2 points with known pixel-meter conversion:
1. point1 = [722.5, 937.5] (pixels) = [0.55, -0.50] (meters)
2. point2 = [403.5, 417.5] (pixels) = [0.80, -0.10] (meters)

For those points, the pixel values were measured from the image and the meter values were measured physically in the lab using the Baxter.

To run the testfile when running catkin_make, run `catkin_make run_tests` from the root of workspace.


### Algorithms and Libraries Used
**pyrealsense2** - Library for accessing Intel RealSenseTM cameras.

**OpenCV** - Computer vision library. Used to detect and classify the items on the image.

**MoveIt** - Motion planning library. Used to generate high-degree of freedom trajectories to grab the objects and throw them to the trash bins.

**[MoveIt! Robots](https://github.com/ros-planning/moveit_robots)** - Motion planning library. It contains [baxter_moveit_config](https://github.com/ros-planning/moveit_robots/tree/kinetic-devel/baxter/baxter_moveit_config) package which is required for the operation of this project.

**JTAS** - Joint Trajectory Action Server. Enables executing complex trajectories using software built-in to Baxter


### Physical Equipment:
1. Baxter Rethink robot
2. Realsense D435i depth camera
3. Table
4. 2 trash bins
4. Cans and bottles
5. 3D Printed Bottle/Can Gripper Atachments (see CAD image and drawing below):
  * This gripper was designed to work with most plastic bottles and aluminum cans.
  * The grippers are printed with PLA plastic, although most rigid 3D Printed materials would be appropriate. 
  * The grippers are dsigned to have 1/4" Thick Soft foam adhered to their inner radius, allowing the foam to conform to the bottle and provide extra grip.
  * Make sure to check the shrinkage of your 3D printer and scale the post cutouts appropriatetly so the attachments can attach to Baxter's stock Gripper Posts.
  * The CAD part and drawing files for the 3D Printed Gripper Attachment for Baxter can be found in the CAD Folder of this repository.
  * They can also be exported from OnShape by following this link: [CAD and Drawing](https://cad.onshape.com/documents/55d17f6159ce035c20241bbe/w/991f13206933eabdd1713bd5/e/5e41ab8ba39d8543f1dda1ea)

![Gripper Diagram](https://github.com/ME495-EmbeddedSystems/final-project-scott-s-bot-for-tots/blob/master/images/Baxter%20Bottle-Can%20Gripper%20Diagram.png)

![Gripper Drawing](https://github.com/ME495-EmbeddedSystems/final-project-scott-s-bot-for-tots/blob/master/images/Baxter%20Bottle-Can%20Gripper%20Attachment%20Drawing.png)


## Future Work
1. **Use machine learning algorithms for better objects classification** - Now, we can only classify specific shapes of bottles and cans. By using machine learning methods, we could classify different types of bottles and cans with the same label, and throw them to the same trash bin.
2. **Add the ability to detect more types of items** - Now, we can only detect cans and bottles. In the future, we want to be able to detect and recycle a variety of objects, such as paper or different types of plastic. To do so, we need to improve our computer vision node (to detect those items), and improve our gripper.
3. **Implement the 3D-Printed Grippers** - We did not end up having time to implement the 3D-Printed Grippers for our testing. Using the stock grippers did not provide a very secure grip - so we had to slow down the robot's motion to prevent the bottle from flying out. Using the 3D Printed Grippers with foam padding would allow for a more secure grip, which would let us speed the robot back up. It would also allow grabbing a greater variety of cylindrical objects (from the body) due to the foam's conformability.
4. **Use the Baxter's hand camera to improve gripping accuracy** - We are currently relying solely on the realsense camera to determine the the object location and grasping positions. However, a more robust solution would be to use the baxter camera and ensure that a) an object is being grasped and b) the object being being grasped is actually in the center of the grippers. With the hand camera video, we would be able to adjust and center the center the gripper to ensure there are no object-gripper collisions.