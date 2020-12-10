cmake_minimum_required(VERSION 3.0.2)
project(bt_tests)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  behaviortree_cpp_v3
  roscpp
  roslib
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES bt_tests
#  CATKIN_DEPENDS behavior_tree_cpp_v3 roscpp roslib
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}, include/bt_tests
)
add_executable(behavior_tester src/behavior_manager.cpp src/dummy_nodes.cpp src/movebase_node.cpp)

target_link_libraries(behavior_tester ${catkin_LIBRARIES})
