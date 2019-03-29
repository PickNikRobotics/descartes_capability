cmake_minimum_required(VERSION 2.8.3)
project(PACKAGE_NAME)

# C++ 11
add_compile_options(-std=c++11)

# Warnings
add_definitions(-W -Wall -Wextra
  -Wwrite-strings -Wunreachable-code -Wpointer-arith
  -Winit-self -Wredundant-decls
  -Wno-unused-parameter -Wno-unused-function)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslint
  rosparam_shortcuts
  rviz_visual_tools
  std_msgs
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
    std_msgs
    rospy
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  DEPENDS
    Boost
    EIGEN3
)
catkin_python_setup()

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
  src/CPP_CLASS_FILE_NAME.cpp
)

# Specify libraries to link a library or executable target against
target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

# Declare a C++ executable
add_executable(
  ${PROJECT_NAME}_CPP_EXECUTABLE_NAME
  src/CPP_EXECUTABLE_NAME.cpp
)

# Rename C++ executable without namespace
set_target_properties(
  ${PROJECT_NAME}_CPP_EXECUTABLE_NAME
  PROPERTIES
    OUTPUT_NAME CPP_EXECUTABLE_NAME PREFIX ""
)

# Specify libraries to link a library or executable target against
target_link_libraries(
  ${PROJECT_NAME}_CPP_EXECUTABLE_NAME
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
    ${PROJECT_NAME} ${PROJECT_NAME}_CPP_EXECUTABLE_NAME
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

catkin_install_python(PROGRAMS scripts/PYTHON_EXECUTABLE_NAME.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Mark roslaunch files for installation
install(
  DIRECTORY
    launch
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Mark config files for installation
install(
  DIRECTORY
    config
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(CPP_CLASS_FILE_NAME_test test/CPP_CLASS_FILE_NAME_test.test test/CPP_CLASS_FILE_NAME_test.cpp)
  target_link_libraries(CPP_CLASS_FILE_NAME_test
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
  )

endif()

## Test for correct C++ source code
roslint_cpp()
