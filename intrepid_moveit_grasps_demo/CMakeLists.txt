cmake_minimum_required(VERSION 3.0.2)
project(intrepid_moveit_grasps_demo)

## Compile as C++11, supported in ROS Kinetic and newer

add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  moveit_grasps
  moveit_msgs
  moveit_ros_planning_interface
  roscpp
  moveit_visual_tools
  rviz_visual_tools
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
#  LIBRARIES intrepid_moveit_grasps_demo
#  CATKIN_DEPENDS moveit moveit_grasps
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library

add_library(intrepid_moveit_grasps_demo
  src/object_builder.cpp
)


## Declare a C++ executable

add_executable(intrepid_moveit_grasps_demo_node src/grasp_pipeline_demo.cpp)

## Add cmake target dependencies of the executable

add_dependencies(intrepid_moveit_grasps_demo_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against

target_link_libraries(intrepid_moveit_grasps_demo_node
  ${catkin_LIBRARIES}
  intrepid_moveit_grasps_demo
)

#############
## Install ##
#############

## Mark executables for installation

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

