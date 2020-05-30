cmake_minimum_required(VERSION 2.8.3)
project(xentrinobot)

##################
## Dependencies ##
##################

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries 
find_package(catkin REQUIRED COMPONENTS
    roscpp 
    rospy 
    tf2
    tf2_ros
    nav_msgs 
    geometry_msgs 
    sensor_msgs 
    std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES xentrinobot
  CATKIN_DEPENDS roscpp rospy tf2 tf2_ros nav_msgs  geometry_msgs sensor_msgs std_msgs 
  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

##add_library(xentrino_base src/xentrino_base.cpp)
##--add_executable(xentrino_base_node src/xentrino_base_node.cpp)
##--target_link_libraries(xentrino_base_node xentrino_base ${catkin_LIBRARIES})


add_executable(xentrino_base_controller src/base_controller.cpp)
target_link_libraries(xentrino_base_controller ${catkin_LIBRARIES})

#############
## Install ##
#############

# Future work

#############
## Testing ##
#############

# Future work
