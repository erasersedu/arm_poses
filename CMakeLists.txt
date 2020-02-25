cmake_minimum_required(VERSION 2.8.3)
project(arm_poses)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

#set(BOOST_LIBRARYDIR "/usr/include/boost")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(Boost REQUIRED
  system
  filesystem
  date_time
  thread
)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  control_msgs
  geometry_msgs
  moveit_commander
  moveit_core
  moveit_msgs
  moveit_ros_planning_interface
  moveit_visual_tools
  roscpp
  shape_msgs
  tf
  trajectory_msgs
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
#  INCLUDE_DIRS include
#  LIBRARIES arm_poses
  CATKIN_DEPENDS
    actionlib
    control_msgs
    geometry_msgs
    moveit_commander
    moveit_core
    moveit_msgs
    moveit_ros_planning_interface
    moveit_visual_tools
    roscpp
    shape_msgs
    tf
    trajectory_msgs
#  CATKIN_DEPENDS actionlib control_msgs geometry_msgs moveit_commander moveit_core moveit_msgs moveit_ros_planning_interface moveit_visual_tools roscpp shape_msgs tf trajectory_msgs
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
  ${Boost_INCLUDE_DIR}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/arm_poses.cpp
# )


## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME} src/arm_poses.cpp)

#${PROJECT_NAME}
add_executable(${PROJECT_NAME} src/arm_poses.cpp)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

#${PROJECT_NAME}_node
add_executable(${PROJECT_NAME}_node src/arm_poses_node.cpp)

add_dependencies(${PROJECT_NAME}_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

### INSTALL ###

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
