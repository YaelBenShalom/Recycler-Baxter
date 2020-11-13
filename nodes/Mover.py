#! /usr/bin/env python3

import sys
import rospy
import numpy 
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import baxter_core_msgs
import baxter_interface
import std_msgs.msg
import cv2
import cv_bridge 
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from baxter_core_msgs.msg import EndEffectorState 
from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest
from moveit_commander import MoveGroupCommander
from baxter_interface import Gripper, CHECK_VERSION
from moveit_commander import RobotCommander
from numpy import result_type


class Mover:
    def __init__(self):
        '''Initialize environment'''
    
        #Initialized baxter
        baxter_interface.robot_enable.RobotEnable()
        rospy.loginfo('Baxter enabled!')
        

        #IK service
        self.left_IK_srv = rospy.ServiceProxy("ExternalTools/left/PositionKinematicsNode/IKService", SolvePositionIK)
        self.right_IK_srv = rospy.ServiceProxy("ExternalTools/right/PositionKinematicsNode/IKService", SolvePositionIK)
        
        # Object location service (vision)
        '''
        Create a srv Object_location

        eg: 
            ---
            float32 x_loc
            float32 y_loc
            float32 z_loc
            float32 object_diameter
            bool    object_found

        '''
        # obj_loc_srv = rospy.ServiceProxy("object_location_service", Object_location)
        
        
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        #Left arm
        self.left_group = moveit_commander.MoveGroupCommander("left_arm")
        pose_left = Pose()
        pose_left.position = Point(0.0, 0.0, 0.0)
        pose_left.orientation = Quaternion (0.0 ,0.0, 0.0, 1.0)
        self.request_pos(pose_left,"left",self.left_group)


        #Right arm (Using this arm)

        self.right_group = moveit_commander.MoveGroupCommander("right_arm") 
        self.right_group.set_goal_position_tolerance(0.001)
        self.right_group.set_goal_orientation_tolerance(0.01)


        #Right gripper
        self.right_gripper = Gripper('right',CHECK_VERSION)
        self.right_gripper.calibrate()
        self.right_gripper.open()
        
        #Default gripper orientation for all poses
        self.default_gripper_orientation = Quaternion(1.0,0.0,0.0,0.0) #Need to change this value
        
        #Soda Can default values
        self.can_z = 0.1;
        self.can_bin_x = 0.3;
        self.can_bin_y = 0.3;
        
        #Water bottle Default values
        self.bottle_z = 0.1;
        self.bottle_bin_x = 0.3;
        self.bottle_bin_y = 0.3;
        
        self.clearence_z = 0.4;  #Clearence Z Height
        self.current_x = 0;
        self.current_y = 0;
        
        self.home_x = 0;
        self.home_y = 0;
        self.home_z = self.clearence_z;


        #End-effector subscribe topic
        self.end_eff_sub = rospy.Subscriber("/robot/end_effector/right_gripper/state", EndEffectorState, self.end_eff_callback) 
        rospy.sleep(1)
        
        #Add table
        self.add_box()
        
        #move to home
        self.move_to_home()
        
        #Temporary Testing Values until vision service works
        self.obj_x_list = [0.1, -0.2, 0.3, 0.5]
        self.obj_y_list = [0.1, -0.2, 0.3, 0.5]
        self.obj_type_list = ["soda", "bottle", "soda", "bottle"]
        self.objects_found =  True
        
        rospy.sleep(10)



        while not rospy.is_shutdown():
            try:   
                #Uncomment once service is written
                
                #rospy.wait_for_service('get_object_list',1.0)
                #Example for response from vision service
                #res = obj_loc_srv.call('''Object_list from vision''')
                
                #res attributes =:
                #objects_found =  True or False
                #obj_type_list = list of strings, i.e. ["soda", "bottle", "soda", "bottle"]
                #obj_x list of doubles for x position of each object, i.e. [0.1, -0.2, 0.3, 0.5]
                #obj_y list of doubles for y position of each object, i.e. [0.1, -0.2, 0.3, 0.5]

                if self.objects_found: #res.objects_found
                    object_count = length(self.obj_type_list) #res.obj_type_list
                    current_object_number = 0
                    while current_object_number < object_count:
                        self.move_to_object(self.obj_type_list(current_object_number),self.obj_x_list(current_object_number),self.obj_y_list(current_object_number))
                        self.grasp_object(self.obj_type_list(current_object_number))
                        self.move_to_bin(self.obj_type_list(current_object_number))
                        self.move_to_home()
                        # current_object_number++
                else:
                    print("No objects found")
            except (rospy.ServiceException, rospy.ROSException) as e:
                rospy.logerr("Service call failed: %s" % e)
            

    def add_box(self):
        '''
        Add table to planning scene
        '''
        self.box_name = "table"
        self.box_pose = PoseStamped()
        self.box_pose.header.frame_id = "base"
        self.box_pose.pose.position.x = 0.9 ## This will change depending on how far the table is from the Baxter
        self.box_pose.pose.position.z = -0.5
        self.box_pose.pose.orientation.w = 1.0
        #Dimension
        self.scene.add_box(self.box_name,self.box_pose,size =(0.75,1.83,0.75))

##########################################################################

    def request_pos(self,pose,arm,group):  
        #Set stamped pose
        pose_stamped = PoseStamped()
        pose_stamped.pose = pose
        pose_stamped.header.frame_id = "base" 
        pose_stamped.header.stamp = rospy.Time.now()

        #IK request
        IK_req = SolvePositionIKRequest()
        IK_req.pose_stamp.append(pose_stamped)

        #Arm service
        arm_srv = "ExternalTools/" + arm + "/PositionKinematicsNode/IKService"

        try:
            rospy.wait_for_service(arm_srv,3.0)
            if arm == "right":
                #Response
                IK_res = self.right_IK_srv(IK_req)
            else:
                IK_res = self.left_IK_srv(IK_req)
        except (rospy.ServiceException, rospy.ROSException) as err_msg:
            rospy.logerr("Service request failed: " % err_msg)
            sys.exit("Error!")
        
        #If IK response is valid, plan and execute the trajectory
        if IK_res.result_type[0] > 0:
            limb_joints = dict(zip(IK_res.joints[0].name, IK_res.joints[0].position))
            group.set_start_state_to_current_state()
            group.set_joint_value_target(limb_joints)
            plan = group.plan(limb_joints)
            rospy.sleep(2.0)
            group.execute(plan)
            rospy(0.5)
            self.current_x = pose.position.x
            self.currenty_y = pose.posiiton.y
            return True
        else:
            return False

        
##########################################################################

    def move_to_home(self):  #Move to home position  
        # Hard (optional) method:
        #Set planning scene with all objects at beginning
        #Add/attach bottle to robot on pickup
        #Remove current bottle from planning scene
        
        #Easy method, no planning scene objects needed for other bottles, current bottle, bins
        
        #Set pose to home Z height at current x,y position
        raise_z_pose = Pose()
        raise_z_pose.position = Point(self.current_x,self.current_y,self.clearence_z)
        raise_z_pose.orientation = self.default_gripper_orientation #Default Gripper Orientation
        
        #Request move service
        self.request_pos(raise_z_pose,"right",self.right_group)
        
        #Set pose for move to home x,y,z position
        home_pose = Pose()
        home_pose.position = Point(self.home_x,self.home_y,self.home_z)
        home_pose.orientation = self.default_gripper_orientation #Default Gripper Orientation
        
        #Request move service
        self.request_pos(home_pose,"right",self.right_group)

        

    #Get force on the gripper
    def end_eff_callback(self,res):
        self.gripper_force = res.force


        
    def move_to_object(self,obj_type,obj_x,obj_y):
        
        #Move to object x,y position at clearence z height
        obj_pose = Pose()
        if  obj_type == "soda":
            obj_pose.position = Point(obj_x,obj_y,self.clearence_z)    #Example position
            obj_pose.orientation = self.default_gripper_orientation
        else:
            obj_pose.position = Point(obj_x,obj_y,self.clearence_z)    #Example position
            obj_pose.orientation = self.default_gripper_orientation
        self.request_pos(obj_pose,'right',self.right_group) 

        #Move down in z to object height
        obj_pose = Pose()
        if  obj_type == "soda":
            obj_pose.position = Point(obj_x,obj_y,self.can_z)    #Example position
            obj_pose.orientation = self.default_gripper_orientation
        else:
            obj_pose.position = Point(obj_x,obj_y,self.bottle_z)    #Example position
            obj_pose.orientation = self.default_gripper_orientation
        
        self.request_pos(obj_pose,'right',self.right_group)
    


    def grasp_object(self,obj_type):  
        
        #add/attach bottle to robot (don't technically need)
        if  obj_type == "soda":
            self.right_gripper.close()  #Need to set close distance for bottle and can somehow  (There is a self.type = 'electric' in gripper.close() function we may need to use)
            rospy.sleep(1)
        else:
            self.right_gripper.close()  #Need to set close distance for bottle and can somehow
            rospy.sleep(1)
        
        if gripper_force == 0:   # or less than a certain value (bad grip)
            print('Object Grasp Failed')
            self.right_gripper.open()
            #remove object from robot arm
            self.move_to_home()
            #Call Reset vision scene

        # find distance of limb from nearest line of sight object (Adjustment for z_pos)
        #dist = baxter_interface.analog_io.AnalogIO('right_hand_range').state()
        #print('Distance is %f' % dist)

            
    def move_to_bin(self,obj_type):
        
        #Move to object x,y position at clearence z height
        raised_pose = Pose()
        raised_pose.position = Point(self.current_x,self.current_y,self.clearence_z)  
        raised_pose.orientation = self.default_gripper_orientation
        self.request_pos(raised_pose,'right',self.right_group)
        
        #Move Object to corresponding bin
        bin_pose = Pose()
        if  obj_type == "soda":
            bin_pose.position = Point(self.can_bin_x,self.can_bin_y,self.clearence_z)    #Example position
            bin_pose.orientation = self.default_gripper_orientation
        else:
            bin_pose.position = Point(self.bottle_bin_x,self.bottle_bin_y,self.clearence_z)    #Example position
            bin_pose.orientation = self.default_gripper_orientation
        self.request_pos(bin_pose,'right',self.right_group) 
        rospy.sleep(1)
        
        #Drop Object
        self.right_gripper.open()
        rospy.sleep(1)
        print('Object Recycled')
        #Remove Object from Robot


def main():
    """ 
    The main() function. 
    """
    rospy.init_node('baxter_mover', log_level=rospy.DEBUG)
    mover = Mover()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass




