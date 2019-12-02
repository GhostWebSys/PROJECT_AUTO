cmake_minimum_required(VERSION 2.8.3)
project(project_auto)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
  darknet_ros
  darknet_ros_msgs
  sensor_msgs
)

add_message_files(FILES ControlMsg.msg FILES LaneDetectMsg.msg)

## Uncomment if the package has a setup.py
catkin_python_setup()

generate_messages(DEPENDENCIES std_msgs)

find_package(Boost REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  LIBRARIES project_auto
  CATKIN_DEPENDS roscpp std_msgs darknet_ros darknet_ros_msgs sensor_msgs
  DEPENDS Boost
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}/include
  ${catkin_INCLUDE_DIRS}/src
)

#############
## Install ##
#############
catkin_install_python(PROGRAMS 
  bin/lane_detecter_node
  bin/sensor_cmd_mux
  bin/sim_control
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
