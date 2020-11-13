cmake_minimum_required(VERSION 3.0.2)
project(can_sort)

find_package(catkin REQUIRED COMPONENTS
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

#add_service_files(
#    FILES 
#)

generate_messages(
    DEPENDENCIES geometry_msgs moveit_msgs std_msgs
)

catkin_package(
   CATKIN_DEPENDS
   geometry_msgs
   message_runtime
   moveit_msgs
   rospy
   std_msgs
)




catkin_install_python(
  PROGRAMS
  nodes/object_detection.py
  nodes/Mover.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


