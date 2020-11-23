cmake_minimum_required(VERSION 3.0.2)
project(can_sort)

find_package(catkin REQUIRED COMPONENTS
    baxter_core_msgs
    baxter_interface
    geometry_msgs
    message_generation
    moveit_msgs
    rospy
    std_msgs
    tf
)

if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
endif()

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

)

generate_messages(
    DEPENDENCIES geometry_msgs moveit_msgs std_msgs
)

catkin_package(
   CATKIN_DEPENDS
   baxter_core_msgs
   baxter_interface
   geometry_msgs
   message_runtime
   moveit_msgs
   rospy
   std_msgs
)




catkin_install_python(
  PROGRAMS
  nodes/recycle
  nodes/object_detection.py
  nodes/Mover.py
  System_operation/small_table_raised
  System_operation/test_vision
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


