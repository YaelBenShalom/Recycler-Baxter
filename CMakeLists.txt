cmake_minimum_required(VERSION 3.0.2)
project(can_sort)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    baxter_core_msgs
    baxter_interface
    geometry_msgs
    message_generation
    moveit_msgs
    rospy
    sensor_msgs
    std_msgs
    tf
)

## Set-up the workspace
catkin_python_setup()

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Generate messages in the 'msg' folder
add_message_files(DIRECTORY msg
    FILES
    Object.msg
)

## Generate services in the 'srv' folder
add_service_files(DIRECTORY srv
    FILES 
    Board.srv
    DisplayImage.srv
)

## Generate added messages and services with any dependencies
generate_messages(
    DEPENDENCIES
    geometry_msgs
    moveit_msgs
    std_msgs
)

## The catkin_package macro generates cmake config files for the package
catkin_package(
   CATKIN_DEPENDS
   baxter_core_msgs
   baxter_interface
   geometry_msgs
   message_runtime
   moveit_msgs
   rospy
   sensor_msgs
   std_msgs
)

## Executable scripts (Python etc.) for installation
catkin_install_python(
  PROGRAMS
  nodes/recycle.py
  nodes/recycle_ML.py
  nodes/object_detection.py
  nodes/object_detection_ML.py
  nodes/disp_img.py
  src/${PROJECT_NAME}/calibration.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Add rostest files
if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    catkin_add_nosetests(test/test_calibration.py)
endif()
