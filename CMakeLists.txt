cmake_minimum_required(VERSION 2.8.3)
project(rosboard)

find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
  std_msgs
  sensor_msgs
)

add_message_files(
  FILES
  NavSatFixPath.msg
)

catkin_python_setup()
generate_messages(
    DEPENDENCIES
    sensor_msgs
)
catkin_package()

catkin_install_python(PROGRAMS
  nodes/rosboard_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install header files
install(
    DIRECTORY 
    include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
)
