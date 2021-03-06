cmake_minimum_required(VERSION 2.8.3)
project(teleop_joystick_control)

include ("../cmake_modules/CMakeOpt.cmake")

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  control_msgs
  actionlib
  ros_control_boilerplate
  behaviors
  std_srvs
  talon_state_controller
  robot_visualizer
)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3 REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    roscpp
    control_msgs
    ros_control_boilerplate
  LIBRARIES
  DEPENDS
)

###########
## Build ##
###########

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})

include_directories(
  include/
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

# add_executable(teleop_joystick_offseason src/teleop_joystick_offseason.cpp)
# set_target_properties(teleop_joystick_offseason PROPERTIES OUTPUT_NAME 
# teleop_joystick_offseason PREFIX "")
# target_link_libraries(teleop_joystick_offseason
#   ${catkin_LIBRARIES}
# )

# Add cmake target dependencies of the executable
# same as for the library above. This is needed to force
# talon controller messages to be built before the
# talon controller code tries to use them
# add_dependencies(teleop_joystick_offseason
#   ${${PROJECT_NAME}_EXPORTED_TARGETS}
#   ${catkin_EXPORTED_TARGETS}
# )

## TOOLS ------------------------------------------------------

# Install ------------------------------------------------------------
#Install header files
# install(TARGETS teleop_joystick_offseason 
#    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

#install(DIRECTORY
#launch
#config
#DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)
install(DIRECTORY
	config
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
