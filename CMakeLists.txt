cmake_minimum_required(VERSION 2.8.12)
project(descartes_capability)

# C++ 14
add_compile_options(-std=c++14)

# Warnings
add_definitions(-W -Wall -Wextra
  -Wwrite-strings -Wunreachable-code -Wpointer-arith
  -Winit-self -Wredundant-decls
  -Wno-unused-parameter -Wno-unused-function)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  descartes_core
  descartes_moveit
  descartes_trajectory
  descartes_planner
  moveit_core
  moveit_ros_planning
  moveit_visual_tools
  moveit_ros_move_group
  roscpp
  roslint
  rospy
)

# System dependencies are found with CMake's conventions
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

###################################
## Catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
    moveit_core
    moveit_ros_planning
    moveit_ros_move_group
    moveit_visual_tools
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  DEPENDS
    Boost
    EIGEN3
)

###########
## Build ##
###########

# Specify additional locations of header files
# Your package locations should be listed before other locations
include_directories(
  SYSTEM
    ${Boost_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIRS}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Declare a C++ library with project namespace to avoid naming collision
add_library(
  ${PROJECT_NAME}
  src/descartes_path_service_capability.cpp
)

add_dependencies(
  ${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

# Specify libraries to link a library or executable target against
target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
  TARGETS
    ${PROJECT_NAME}
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

# Mark config files for installation
install(
  DIRECTORY
    config
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Mark python demo script for installation
install(PROGRAMS
  scripts/example_cartesian_command.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  descartes_capability_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(pluginlib REQUIRED)
  find_package(moveit_resources REQUIRED)
  include_directories(${moveit_resources_INCLUDE_DIRS})

  add_rostest_gtest(descartes_path_service_capability_test test/descartes_path_service_capability_test.test test/descartes_path_service_capability_test.cpp)
  target_link_libraries(descartes_path_service_capability_test
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
  )

endif()

## Test for correct C++ source code
roslint_cpp()
