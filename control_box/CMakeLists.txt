cmake_minimum_required(VERSION 3.0.2)
project(control_box)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roslaunch
)

# Check that the roslaunch files are valid
roslaunch_add_file_check(launch)

###################################
## catkin specific configuration ##
###################################

catkin_package()

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS})
