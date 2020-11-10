cmake_minimum_required(VERSION 3.0.2)
project(can_sort)

find_package(catkin REQUIRED COMPONENTS
    rospy
    tf
    message_generation
    std_msgs
    geometry_msgs
    moveit_msgs
    rostest
    
)

if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
endif()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_service_files(
    FILES 
)

generate_messages(
    DEPENDENCIES std_msgs geometry_msgs moveit_msgs
)
catkin_package(
   CATKIN_DEPENDS rospy
)




catkin_install_python(
  PROGRAMS nodes/object_detection.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


