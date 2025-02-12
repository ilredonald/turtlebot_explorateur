cmake_minimum_required(VERSION 3.0.2)
project(tuto_ros)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
#find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tuto_ros
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
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

add_executable(noeud src/noeud.cpp)
target_link_libraries(noeud ${catkin_LIBRARIES})

add_executable(move_auto src/move_auto.cpp)
target_link_libraries(move_auto ${catkin_LIBRARIES})

add_executable(trace1 src/trace1.cpp)
target_link_libraries(trace1  ${catkin_LIBRARIES})

add_executable(trace2 src/trace2.cpp)
target_link_libraries(trace2  ${catkin_LIBRARIES})