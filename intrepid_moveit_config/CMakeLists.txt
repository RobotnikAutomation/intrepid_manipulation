cmake_minimum_required(VERSION 3.1.3)
project(intrepid_moveit_config)

find_package(catkin REQUIRED roscpp moveit_ros_warehouse)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(generate_path_constraints scripts/generate_path_constraints.cpp)

add_dependencies(generate_path_constraints ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(generate_path_constraints
${catkin_LIBRARIES}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
