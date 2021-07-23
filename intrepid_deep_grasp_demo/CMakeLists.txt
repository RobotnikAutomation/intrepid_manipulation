cmake_minimum_required(VERSION 3.0.2)
project(intrepid_deep_grasp_demo)

# C++ 14
add_compile_options(-std=c++14)

# Warnings
add_definitions(-W -Wall -Wextra
  -Wwrite-strings -Wunreachable-code -Wpointer-arith
  -Winit-self -Wredundant-decls
  -Wno-unused-parameter -Wno-unused-function)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
	actionlib
	moveit_core
	moveit_ros_planning_interface
	moveit_task_constructor_core
	moveit_task_constructor_msgs
  roscpp
	rosparam_shortcuts
  sensor_msgs
  moveit_msgs
  rcomponent
  moveit_visual_tools
  rviz_visual_tools
)

###################################
## Catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS
	moveit_task_constructor_msgs
 	roscpp
  sensor_msgs
)

###########
## Build ##
###########

# Specify additional locations of header files
# Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(intrepid_deep_grasp_demo
 src/object_builder.cpp
 src/pose_builder.cpp

)

# Declare a C++ executable
add_executable(intrepid_deep_grasp_demo_node
 							 src/deep_grasp_demo.cpp
               src/deep_grasp_demo_node.cpp 
							 src/deep_pick_place_task.cpp
)

add_dependencies(intrepid_deep_grasp_demo_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Specify libraries to link a library or executable target against
target_link_libraries(intrepid_deep_grasp_demo_node
                      ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
  TARGETS
    intrepid_deep_grasp_demo
  ARCHIVE DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION
    ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# # Mark roslaunch files for installation
# install(
#   DIRECTORY
#     launch
#   DESTINATION
#     ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
#
# # Mark config files for installation
# install(
#   DIRECTORY
#     config
#   DESTINATION
#     ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
