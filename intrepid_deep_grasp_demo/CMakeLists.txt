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
  rviz
	rosparam_shortcuts
  sensor_msgs
  moveit_msgs
  rcomponent
  moveit_visual_tools
  rviz_visual_tools
  intrepid_manipulation_msgs
  jsk_rviz_plugins
  std_msgs
  tf2
  eigen_conversions
  tf2_ros
  pluginlib
)

# System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(OGRE_OV OGRE)

# Qt 4 or 5
if("${rviz_QT_VERSION}" VERSION_LESS "5")
  find_package(Qt4 ${rviz_QT_VERSION} REQUIRED QtCore QtGui)
  include(${QT_USE_FILE})
  macro(qt_wrap_ui)
    qt4_wrap_ui(${ARGN})
  endmacro()
else()
  find_package(Qt5 ${rviz_QT_VERSION} REQUIRED Core Widgets)
  set(QT_LIBRARIES Qt5::Widgets)
  macro(qt_wrap_ui)
    qt5_wrap_ui(${ARGN})
  endmacro()
endif()
## Prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOMOC_COMPILER_PREDEFINES OFF)
add_definitions(-DQT_NO_KEYWORDS)  


###################################
## Catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES intrepid_manipulation_gui_pluginlib
 CATKIN_DEPENDS
	moveit_task_constructor_msgs
 	roscpp
  sensor_msgs
  pluginlib
 DEPENDS
  EIGEN3
  PCL
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

add_library(intrepid_deep_grasp_demo_lib
 src/object_builder.cpp
 src/pose_builder.cpp
)


# Declare a C++ executable
add_executable(intrepid_deep_grasp_demo_node
 							 src/deep_grasp_demo.cpp
               src/deep_grasp_demo_node.cpp 
							 src/deep_pick_place_task.cpp
               src/deep_pick_task.cpp
)

add_executable(republish_filtered_cloud_node
               src/republish_filtered_cloud.cpp
)

add_executable(calculate_transform_node
               src/calculate_transform.cpp
)

add_dependencies(intrepid_deep_grasp_demo_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(republish_filtered_cloud_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(calculate_transform_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


# Specify libraries to link a library or executable target against
target_link_libraries(intrepid_deep_grasp_demo_node
                      intrepid_deep_grasp_demo_lib
                      ${catkin_LIBRARIES}
)

target_link_libraries(republish_filtered_cloud_node
                      ${catkin_LIBRARIES}
)

target_link_libraries(calculate_transform_node
                      ${catkin_LIBRARIES}
)

add_library(intrepid_manipulation_gui_pluginlib src/intrepid_manipulation_gui.cpp include/intrepid_deep_grasp_demo/intrepid_manipulation_gui.h)

# add_dependencies(intrepid_manipulation_gui_pluginlib src/intrepid_manipulation_gui.cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(intrepid_manipulation_gui_pluginlib
  ${catkin_LIBRARIES}
  ${QT_LIBRARIES}
  ${rviz_DEFAULT_PLUGIN_LIBRARIES}
)

install(TARGETS  intrepid_manipulation_gui_pluginlib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
  TARGETS
    intrepid_deep_grasp_demo_node
    intrepid_manipulation_gui_pluginlib
  ARCHIVE DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  intrepid_manipulation_gui.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
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
