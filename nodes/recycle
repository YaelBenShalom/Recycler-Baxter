#! /usr/bin/env python3
""" This node is responsible for causing Baxter to move objects to their appropriate recycling bins 
after receiving the object type/location data from the object_detection node. It utilizes moveit_commander.

- self.addbox(): Adds recyle bins and table to MoveIt planning scene
- self.get_image(): Gets object type/location data utilizing can_sort service from object_detection node, sets up obj lists
- self.self.initialize_pos(): Initializes Baxter into good position for taking picture of the board without an obstructed view
- main loop (while not rospy.is_shutdown(): Runs through object list, telling Baxter moving each object to the appropriate bin

"""

#required imports
import sys
import rospy
import numpy as np
import math
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import baxter_core_msgs
import baxter_interface
import std_msgs.msg
import cv2
import cv_bridge
import intera_interface
from subprocess import call
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from baxter_core_msgs.msg import EndEffectorState
from moveit_commander import MoveGroupCommander
from baxter_interface import Gripper, CHECK_VERSION
from moveit_commander import RobotCommander
from numpy import result_type
from can_sort.msg import Object
from can_sort.srv import Board, BoardResponse
from sensor_msgs.msg import Image
from can_sort.srv import DisplayImage, DisplayImageResponse

# Recycle Node

class Recycle:
    """ A class for instructing Baxter to recycle bottles and cans """
    def __init__(self):
        """ Initialize environment
        """
        # Initialized baxter
        baxter_interface.robot_enable.RobotEnable()
        rospy.loginfo('Baxter enabled!')
        
        # Setup MoveIt_Commander
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        
        #Define DisplayImage service Proxy
        self.display_img = rospy.ServiceProxy('display_image', DisplayImage)

        # Set right arm paramaters
        self.right_group = moveit_commander.MoveGroupCommander("right_arm", wait_for_servers=10)
        self.right_group.set_goal_position_tolerance(0.001)
        self.right_group.set_goal_orientation_tolerance(0.01)
        self.right_group.set_num_planning_attempts(5)

        # Set right gripper parameters
        self.right_gripper = Gripper('right')
        self.right_gripper.type() == 'electric'
        self.right_gripper.calibrate()
        self.right_gripper.open()
        self.default_gripper_orientation = Quaternion(1.0, 0, 0, 0)
        
        #Z Height set for bottle/can types. For future work, one z-height when using 3D-printed grippers should be sufficient.
        self.can_z = -0.11    # Soda can gripping height for table
        self.bottle_z = -0.05 # Bottle gripping height for table

        # Soda recyling bin position, both bins on the robot's right. 
        # Needs to be adjusted for each new "recycle station" Baxter is set up at.
        self.can_bin_x = 0.2
        self.can_bin_y = -0.9

        # Bottle recycling bin position, both on the robot's right
        # Needs to be adjusted for each new "recycle station" Baxter is set up at.
        self.bottle_bin_x = 0.5
        self.bottle_bin_y = -0.85

        # Clearance height for gripper to ensure no collisions
        self.clearance_z = 0.2

        # Baxter home position
        self.home_x = 0.65
        self.home_y = -0.45
        self.home_z = self.clearance_z
        
        # Create object lists
        self.obj_type_list = []
        self.obj_x_list = []
        self.obj_y_list = []
        self.objects_found = False

        # Call initial functions
        self.add_planning_objs()
        self.get_image()
        self.initialize_pos()

        rospy.sleep(1)

        while not rospy.is_shutdown():
            try:
                if self.objects_found:  
                    object_count = len(self.obj_type_list)  
                    current_object_number = 0
                    while current_object_number < object_count:
                        print('Picking up object number %d \n' % current_object_number)
                        self.move_to_object(self.obj_type_list[current_object_number],
                                            self.obj_x_list[current_object_number],
                                            self.obj_y_list[current_object_number])
                        self.grasp_object(self.obj_type_list[current_object_number])
                        self.move_to_bin(self.obj_type_list[current_object_number],
                                         self.obj_x_list[current_object_number], self.obj_y_list[current_object_number])
                        self.move_to_home()
                        print('Picked object number %d \n' % current_object_number)
                        current_object_number += 1
                    break
                else:
                    print("No objects found")
            except (rospy.ServiceException, rospy.ROSException) as e:
                rospy.logerr("Service call failed: %s" % e)

    #######################################################################################################################
    
    ### Functions: ###
                
    def add_planning_objs(self):
        """ Add table and trash bins to planning scene
        Args:
          None
          
        Returns:
          None 
        """
        table_name = "table"
        table_pose = PoseStamped()
        table_pose.header.frame_id = "base"
        table_pose.pose.position.x = 1.2  # This will change depending on how far the table is from the Baxter
        table_pose.pose.position.y = -0.35
        table_pose.pose.position.z = -0.55
        table_pose.pose.orientation.w = 1.0
        # Dimension
        self.scene.add_box(table_name, table_pose, size=(1.83, 0.75, 0.75))

        bin1_name = "can_bin"  # Closer to the robot
        bin1_pose = PoseStamped()
        bin1_pose.header.frame_id = "base"
        bin1_pose.pose.position.x = 0.3  # This will change depending on how far the bin is from the Baxter
        bin1_pose.pose.position.y = -1.0
        bin1_pose.pose.position.z = -0.55
        bin1_pose.pose.orientation.w = 1.0
        # Dimension
        self.scene.add_box(bin1_name, bin1_pose, size=(0.3, 0.52, 0.77))

        bin2_name = "bottle_bin"  # Farther from the robot
        bin2_pose = PoseStamped()
        bin2_pose.header.frame_id = "base"
        bin2_pose.pose.position.x = 0.65  # This will change depending on how far the bin is from the Baxter
        bin2_pose.pose.position.y = -1.0
        bin2_pose.pose.position.z = -0.55
        bin2_pose.pose.orientation.w = 1.0
        # Dimension
        self.scene.add_box(bin2_name, bin2_pose, size=(0.3, 0.52, 0.77))

        
    def get_image(self):
        """ Getting board state from the board_state service. Sets object_found and object list variables
        Args:
          None
          
        Returns:
          None 
        """
        # move arm to a good position before the initial poses
        right = baxter_interface.Limb('right')

        rcmd = {"right_e0": 0.07593204900032798,
                "right_e1": 1.3479856173543072,
                "right_s0": -0.5629709491539469,
                "right_s1": -0.9211554631251909,
                "right_w0": 3.0104372962251245,
                "right_w1": -1.0971797585350422,
                "right_w2": -0.09088836168221076}

        right.move_to_joint_positions(rcmd)

        # Set up Board service to get list of objects from camera
        get_obj_pos = rospy.ServiceProxy('board_state', Board)
        object_list = self.get_obj_pos(use_real=True)
         
        #Determine if objects were found. 
        if len(object_list) > 0:
            self.objects_found = True
            print(f" found {len(object_list.objects)} objects")
            
            #Set up obj_x_list (list of objects' x coordinate)
            #Set up obj_y_list (list of objects' y coordinate)
            #Set up obj_type_list (list of type of each object (can or bottle))
            for item in object_list.objects:
                self.obj_x_list.append(item.location.x)
                self.obj_y_list.append(item.location.y)
                if item.type == 0:
                    self.obj_type_list.append("bottle")
                else:
                    self.obj_type_list.append("soda")

            print(f"x list {self.obj_x_list}")
            print(f"y list {self.obj_y_list}")
            print(f"type list is {self.obj_type_list}")
        
        else:
            self.objects_found = False
        
        #Stop object_detection node to prevent it from taking too much processing power/time
        call(["rosnode", "kill", "object_detection"])
        rospy.sleep(2)

        
    def initialize_pos(self):
        """ Moves arm to a initial position before the initial poses
        The initial position ensures no interference to the image
        Args:
          None
          
        Returns:
          None 
        """
        right = baxter_interface.Limb('right')

        rcmd = {"right_e0": 0.584830175381314,
                "right_e1": 1.601475942552372,
                "right_s0": 0.12732040539448933,
                "right_s1": -1.19573802415668,
                "right_w0": 2.923000391315656,
                "right_w1": -1.2225826879446748,
                "right_w2": -0.033364082136507746}

        right.move_to_joint_positions(rcmd)

        
    def move_to_home(self):
        """ This function moves the robot's arm to the home position
        Args:
          None
          
        Returns:
          None 
        """
        print("Moving to the home position")
        
        # Set pose for move to home x,y,z,gripper orientation position
        home_pose = Pose()
        home_pose.position = Point(self.home_x, self.home_y, self.home_z)
        home_pose.orientation = self.default_gripper_orientation
        
        # Set waypoint, plan, execute, clear pose targets
        waypoints = [home_pose]
        (plan, fraction) = self.right_group.compute_cartesian_path(waypoints, 0.1, 0.0)
        print(f"fraction of traj is {fraction}")
        time = self.right_group.get_planning_time()
        print(f"time is {time}")
        self.right_group.execute(plan, wait=True)
        self.right_group.stop()
        self.right_group.clear_pose_targets()
        print('At the home position')
        rospy.sleep(1)
        waypoints = [] # Reset waypoints list

        
    def move_to_object(self, obj_type, obj_x, obj_y):
        """ This function moves the robot's arm to a given position
        Args:
          obj_type (string): Name of the current object, either "soda" or "can"
          obj_x (double): x-position of the object
          obj_y (double): y-position of the object
          
        Returns:
          None 
        """
        print("Moving to object perch position")
        if obj_type == "soda":
            self.display_img(img_num=2)
        else:
            self.display_img(img_num=3)

        waypoints = []

        # Move to object x,y position at clearance z height
        self.obj_pose = Pose()
        self.obj_pose.position = Point(obj_x, obj_y, self.clearance_z)  # Example position
        self.obj_pose.orientation = self.default_gripper_orientation
        waypoints.append(self.obj_pose)

        # Move down in z to object height
        print("Moving to the perch position")
        self.obj_pose = Pose()
        if obj_type == "soda":
            self.obj_pose.position = Point(obj_x, obj_y, self.can_z)  # Example position
            self.obj_pose.orientation = self.default_gripper_orientation

            mid_pose = Pose()
            mid_pose.position = Point(obj_x, obj_y, (self.can_z + 0.05))
            mid_pose.orientation = self.default_gripper_orientation

        else: #obj_type = "bottle"
            self.obj_pose.position = Point(obj_x, obj_y, self.bottle_z)  # Example position
            self.obj_pose.orientation = self.default_gripper_orientation

            mid_pose = Pose()
            mid_pose.position = Point(obj_x, obj_y, (self.bottle_z + 0.05))
            mid_pose.orientation = self.default_gripper_orientation
            
        #Append midpoint pose (allows for smoother/slower motion to prevent dropping objects)
        #Then plan, execute, clear waypoints list
        waypoints.append(mid_pose)
        (plan, fraction) = self.right_group.compute_cartesian_path(waypoints, 0.01, 0.0)
        self.right_group.execute(plan, wait=True)
        self.right_group.stop()
        self.right_group.clear_pose_targets()
        print("At the perch position")
        rospy.sleep(1)
        waypoints = []
        
        #Append final pose, plan, execute, clear waypoints list
        waypoints.append(self.obj_pose)
        (plan, fraction) = self.right_group.compute_cartesian_path(waypoints, 0.01, 0.0)
        self.right_group.execute(plan, wait=True)
        self.right_group.stop()
        self.right_group.clear_pose_targets()
        print("At the grasping position")
        rospy.sleep(1)
        waypoints = []

        
    def grasp_object(self, obj_type):
        """ This function makes the robot's arm grasp an object, depends on the object's type
        Args:
          obj_type (string): Name of the current object, either "soda" or "can"
          
        Returns:
          None 
        """
        #command_position(50) closes grippers correct distance for soda can
        if obj_type == "soda":
            self.right_gripper.command_position(50)  
            rospy.sleep(1)
            
        #command_position(0) closes grippers correct distance for bottle
        else: #obj_type = "bottle"
            self.right_gripper.command_position(0)  

            rospy.sleep(1)
        print("Closed the gripper")

        
    def move_to_bin(self, obj_type, obj_x, obj_y):
        """ This function moves the robot's arm to the appropriate trash bin (depends on the objects type)
        Args:
          obj_type (string): Name of the current object, either "soda" or "can"
          obj_x (double): x-position of the object
          obj_y (double): y-position of the object
          
        Returns:
          None 
        """
        self.display_img(img_num=1)
        waypoints = []

        # Move to object x,y position at clearance z height
        raised_pose = Pose()
        raised_pose.position = Point(obj_x, obj_y, self.clearance_z)
        raised_pose.orientation = self.default_gripper_orientation
        waypoints.append(raised_pose)

        # Move back to the home position to ensure a safe path to the bin
        mid_pose = Pose()
        if obj_type == "soda":
            mid_pose.position = Point(self.home_x, self.home_y, self.clearance_z)
            mid_pose.orientation = self.default_gripper_orientation
        else: #obj_type = "bottle"
            mid_pose.position = Point(self.home_x, self.home_y, self.clearance_z)
            mid_pose.orientation = self.default_gripper_orientation
            
        #Append midpoint pose (allows for smoother/slower motion to prevent dropping objects)
        #Then plan, execute, clear waypoints list
        waypoints.append(mid_pose)
        (plan, fraction) = self.right_group.compute_cartesian_path(waypoints, 0.01, 0.0)
        self.right_group.execute(plan, wait=True)
        self.right_group.stop()
        self.right_group.clear_pose_targets()
        rospy.sleep(1)
        waypoints = []

        # Move Object to corresponding bin
        bin_pose = Pose()
        if obj_type == "soda":
            bin_pose.position = Point(self.can_bin_x, self.can_bin_y, self.clearance_z)
            # bin_pose.position = Point(self.bottle_bin_x,self.bottle_bin_y,self.clearance_z)
            bin_pose.orientation = self.default_gripper_orientation
        else: #obj_type = "bottle"
            bin_pose.position = Point(self.bottle_bin_x, self.bottle_bin_y, self.clearance_z)  # Example position
            bin_pose.orientation = self.default_gripper_orientation

        waypoints.append(bin_pose)
        print("Go to bin")
        (plan, fraction) = self.right_group.compute_cartesian_path(waypoints, 0.01, 0.0)
        self.right_group.execute(plan, wait=True)
        self.right_group.stop()
        self.right_group.clear_pose_targets()
        print("At bin")
        rospy.sleep(1)
        waypoints = []

        # Drop Object
        self.right_gripper.open()
        print("Open gripper")
        self.display_img(img_num=4)
        rospy.sleep(1)
        print('Object Recycled')
     


def main():
    """ The main() function.
    """
    rospy.init_node('baxter_recycle', log_level=rospy.DEBUG)
    mover = Recycle()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
