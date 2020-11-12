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
        obj_loc_srv = rospy.ServiceProxy("object_location_service", Object_location)
        
        
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
        self.bottle_z = 1;
        self.can_z = 2;
        self.high_z = 1;

        #End-effector subscribe topic
        self.end_eff_sub = rospy.Subscriber("/robot/end_effector/right_gripper/state", EndEffectorState, end_eff_callback) 
        rospy.sleep(1)

        self.gripper_force = 0
        
        #Add table
        self.add_box()

        self.move_to_vision()
        rospy.sleep(10)



        while not rospy.is_shutdown():
            try:    
                rospy.wait_for_service('object_location_service',1.0)
                #Example for response from vision service
                res = obj_loc_srv.call('''Object_location from vision''')

                if res.object_found:
                    self.grasp_object(res.x,res.y,res.z,obj_diameter)
                else:
                    print("No objetc found")
            except (rospy.ServiceException, rospy.ROSException) as e:
                rospy.logerr("Service call failed: %s" % e)
            

    def add_box(self):
        '''
        Add table to planning scene
        '''
        self.box_name = "table"
        self.box_pose = PoseStamped()
        self.box_pose.header.frame_id = "base"
        self.box_pose.pose.position.x = 0.5
        self.box_pose.pose.position.z = -0.2
        self.box_pose.pose.orientation.w = 1.0
        #Dimension
        self.scene.add_box(self.box_name,self.box_pose,size(1,1.5,0.3))






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
            return True
        else:
            return False



    def move_to_home(self):  #Move to home position
        
        #Easy method, no planning scene objects needed for other bottles, current bottle, bins
        #1. Get current gripper x,y pose
        #2. Move to home Z height at current x,y position
        #3. Move to home x,y position
        
        # Hard:
        #Set planning scene with all objects at beginning
        #Add/attach bottle to robot on pickup
        #Remove current bottle from planning scene
        
        
        #move to home is just moving to a single home pose
        
        
        #Set pose
        pose = Pose()
        pose.position = Point(0.2,0.1,0.3)
        pose.orientation = self.default_gripper_orientation #Default Gripper Orientation
        
        #Request service
        self.request_pos(pose,"right",self.right_group)

    

    #Get force on the gripper, confirm object is grabbed 
    def end_eff_callback(self,res):
        self.gripper_force = res.force




    def move_to_object(self,obj_type):
        
        #For Soda Can
        #Position = point(soda.x,soda.y,high z)
        #Orientation =  self.default_gripper_orientation
        #self.request_pos
        
        #Position = point(soda.x,soda.y,soda.z)
        #Orientation =  self.default_gripper_orientation
        #self.request_pos
        
        #For Water Bottle
        #Position = point(bottle.x,bottle.y,high z)
        #Orientation =  self.default_gripper_orientation
        #self.request_pos
        
        #Position = point(bottle.x,bottle.y,bottle.z)
        #Orientation =  self.default_gripper_orientation
        #self.request_pos
        
        #
        if  obj_type == "soda":
            #Move to place1
            pose = Pose()
            pose.position = Point(0.5,0.2,0.2)    #Example position
            pose.orientation = Quaternion(1.0,0.0,0.0,0.0)
        else:
            #Move to place2
            pose = Pose()
            pose.position = Point(-0.3,-0.2,0.2)    #Example position
            pose.orientation = Quaternion(1.0,0.0,0.0,0.0)
        self.request_pos(pose,'right',self.right_group)
    


    def grasp_object(self,obj_type):
    #def grasp_object(self, x_pos,y_pos,z_pos,obj_type):
        
        #add/attach bottle to robot (don't technically need)
        
        
        #If Soda
            #Close to Soda diameter
            #Check Force
            #IF force = 0
                #remove object from robot arm
                #move to home, retake photo and reset plannign scene
         #else:
            #Close to bottle diameter
            #Check Force
            #IF force = 0
                #remove object from robot arm
                #move to home, retake photo and reset plannign scene
        
        pose = Pose()
        pose.position = Point(x_pos,y_pos,0.0)
        pose.orientation = Quaternion(1.0,0.0,0.0,0.0)
        self.request_pos(pose,'right',self.right_group)
        rospy.sleep(0.5)

        # find distance of limb from nearest line of sight object (Adjustment for z_pos)
        dist = baxter_interface.analog_io.AnalogIO('right_hand_range').state()
        rospy.sleep(1)
        if  obj_type == "soda":
            print('Out of range')
            z = '''... '''
        else: 
            print('Distance is %f' % dist)
            z = '''dist'''


        pose2 = Pose()
        pose2.position = Point(x_pos,y_pos,z)
        pose2.orientation = Quaternion(1.0,0.0,0.0,0.0)
        self.request_pos(pose2,'right',self.right_group)
        rospy.sleep(0.5)
        self.right_gripper.close()        #Grab the bottle
        rospy.sleep(0.5)
        
        #If gripper fetches nothing then goes back to vision state
         #For grasp function, if gripper force is not what is expected, move to home and re-take picture
        if gripper_force == 0:   # or less than a certain value (bad grip)
            self.right_gripper.open()
            self.move_to_home()
        else:
            pose = Pose()
            pose.position = Point(x_pos,y_pos,0.3)
            pose.orientation = Quaternion(1.0,0.0,0.0,0.0)
            self.request_pos(pose,'right',self.right_group)    #Lift up
            if gripper_force == 0:
                self.right_gripper.open()
                self.move_to_vision()
                return
            self.move_to_destination(obj_diameter)
            self.right_gripper.open()
            self.move_to_vision()

            
    def move_to_bin(self,object_diameter?):
        
        #If Soda Can
        #Position = point(soda.x,soda.y,high z)
        #Orientation =  self.default_gripper_orientation
        #self.request_pos
        
        #Position = point(soda_bin.x,soda_bin.y,high.z)
        #Orientation =  self.default_gripper_orientation
        #self.request_pos
        
        # Open Gripper
        
        ##########################
        
        # else (For Water Bottle)
        #Position = point(bottle.x,bottle.y,high z)
        #Orientation =  self.default_gripper_orientation
        #self.request_pos
        
        #Position = point(bottle_bin.x,bottle_bin.y,high.z)
        #Orientation =  self.default_gripper_orientation
        #self.request_pos
        
        # Open Gripper
        
        #########################
        
        #Remove dropped object from list of objects
        
        
        #
        if  obj_type == "soda":
            #Move to place1
            pose = Pose()
            pose.position = Point(0.5,0.2,0.2)    #Example position
            pose.orientation = Quaternion(1.0,0.0,0.0,0.0)
        else:
            #Move to place2
            pose = Pose()
            pose.position = Point(-0.3,-0.2,0.2)    #Example position
            pose.orientation = Quaternion(1.0,0.0,0.0,0.0)
        self.request_pos(pose,'right',self.right_group)




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




