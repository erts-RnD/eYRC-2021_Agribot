cmake_minimum_required(VERSION 3.0.2)
project(tomato_gazebo)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation 
  gazebo_msgs 
  roscpp
  rospy  
  std_srvs 
  std_msgs
  gazebo_ros
)
include_directories( include ${catkin_INCLUDE_DIRS})


find_package(gazebo REQUIRED)
include_directories( include ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

################################################
## Declare ROS messages, services and actions ##
################################################
## Generate messages in the 'msg' folder
#  add_message_files(
#    FILES
#  )

## Generate services in the 'srv' folder
##add_service_files(
##  FILES
##  Gripper.srv

##)



 generate_messages(
   DEPENDENCIES
   std_msgs  
 )

###################################
## catkin specific configuration ##
###################################

catkin_package(
#   LIBRARIES 
# 	DEPENDS 
# 	    roscpp 
# 	    gazebo_ros
#       std_msgs 
#       geometry_msgs 
#       tf
#       message_runtime

)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
