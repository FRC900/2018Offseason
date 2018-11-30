cmake_minimum_required(VERSION 2.8.3)
project(ros_control_boilerplate)

include ("../cmake_modules/CMakeOpt.cmake")

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  realtime_tools
  hardware_interface
  controller_manager
  roscpp
  control_msgs
  trajectory_msgs
  actionlib
  urdf
  std_msgs
  sensor_msgs
  rosparam_shortcuts
  talon_interface
  message_generation
  elevator_controller
  robot_controller_interface
  pdp_state_interface
  remote_hardware_interface
  robot_controller_interface
  pcm_state_interface
)

add_message_files (
  FILES
  MatchSpecificData.msg
  JoystickState.msg
  AutoMode.msg
  AutoModeStatus.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
  ros_control_boilerplate
)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)
find_package(Gflags REQUIRED)
find_package(Eigen3 REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
	hardware_interface
	controller_manager
	control_msgs
	trajectory_msgs
	actionlib
	urdf
	std_msgs
	sensor_msgs
	rosparam_shortcuts
	talon_interface
	pdp_state_interface
	elevator_controller
	message_runtime
  LIBRARIES
)

###########
## Build ##
###########

#include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})

include_directories(
  include/
  ${catkin_INCLUDE_DIRS}
  #${Gflags_INCLUDE_DIRS}
)

# Sim version of main control
# executable.  For now this just prints debugging
# info for testing code which doens't actually hit
# RoboRIO or CAN hardware. Eventually expand into
# controlling some sort of sim?
add_executable(frcrobot_sim_main 
	src/frcrobot_sim_main.cpp
	src/frcrobot_sim_interface.cpp
	src/frc_robot_interface.cpp
	src/generic_hw_control_loop.cpp
)

target_link_libraries(frcrobot_sim_main
	${catkin_LIBRARIES}
)

# Forces messages to be generated before
# the target which uses them
add_dependencies(frcrobot_sim_main
	${${PROJECT_NAME}_EXPORTED_TARGETS}
	${catkin_EXPORTED_TARGETS}
)

# Install executable on the off chance you'd ever
# want to run sim on the RoboRIO
install(TARGETS
  frcrobot_sim_main
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Main control executable
set (FRCROBOT_HW_MAIN_SRCS 
	src/frcrobot_hw_main.cpp
	src/frcrobot_hw_interface.cpp
	src/frc_robot_interface.cpp
	src/generic_hw_control_loop.cpp
)

# Base regardless of OS type
set(LIB_BASE $ENV{HOME}/wpilib/user/cpp/lib/linux)
if ("${ARM_PREFIX}" STREQUAL "arm-frc-linux-gnueabi")

	include_directories(
		$ENV{HOME}/wpilib/user/cpp/include
		$ENV{HOME}/wpilib/cpp/current/include
	)
add_definitions(-DHWI_ROBORIO)
	# Rio only
	set (WPI_LIB_DIR        $ENV{HOME}/wpilib/cpp/current/reflib/linux/athena/shared)
	set (WPI_COMMON_LIB_DIR $ENV{HOME}/wpilib/common/current/lib/linux/athena/shared)
	set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -L ${WPI_LIB_DIR}")
	set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -L ${WPI_COMMON_LIB_DIR}")

	# Only need WPI libs for the Rio
	set (CTRE_LIB_DIR ${LIB_BASE}/athena/static)
	set (PLATFORM_SPECIFIC_LIBS
		${LIB_BASE}/../libnavx_frc_cpp.a
		${WPI_LIB_DIR}/libwpi.so
		${WPI_LIB_DIR}/libFRC_NetworkCommunication.so.18.0.0
		${WPI_LIB_DIR}/libRoboRIO_FRC_ChipObject.so.18.0.0 )
elseif ("${CMAKE_LIBRARY_ARCHITECTURE}" STREQUAL "arm-linux-gnueabihf") # Jetson TK1
	set (CTRE_LIB_DIR ${LIB_BASE}/armhf/static)
elseif ("${CMAKE_LIBRARY_ARCHITECTURE}" STREQUAL "aarch64-linux-gnu") # Jetson TX1/TX2
	set (CTRE_LIB_DIR ${LIB_BASE}/aarch64/static)
else() # x86-64, hopefully
	set (CTRE_LIB_DIR ${LIB_BASE}/x86-64/static)
endif()

# All platforms need these, they just live
# in different dirs for different architectures
set (CTRE_USER_LIBS
	${CTRE_LIB_DIR}/libCTRE_Phoenix.a
	${CTRE_LIB_DIR}/libCTRE_PhoenixCCI.a
)

set (ALLWPILIB ../../../allwpilib)
if (NOT "${ARM_PREFIX}" STREQUAL "arm-frc-linux-gnueabi")
	set (CTRE_USER_LIBS 
		${CTRE_USER_LIBS}
		${CTRE_LIB_DIR}/libCTRE_PhoenixPlatform_socketcan.a
		${CTRE_LIB_DIR}/libCTRE_PhoenixCanutils.a
		)

	# For now, use 2019beta includes only for this code to line up with the
	# 2019 beta sources we pull in below. Eventually sync this up when 
	# everything moves to 2019 code
	include_directories(
		$ENV{HOME}/frc2019/roborio/arm-frc2019-linux-gnueabi/include/wpilib
		$ENV{HOME}/frc2019/roborio/arm-frc2019-linux-gnueabi/include
		$ENV{HOME}/frc2019/roborio/arm-frc2019-linux-gnueabi/include/navx
		${ALLWPILIB}/hal/src/main/native/athena
		#${ALLWPILIB}/wpiutil/src/main/native/include
		#${ALLWPILIB}/wpilibc/src/main/native/include
		#${ALLWPILIB}/hal/src/main/native/include
		#${ALLWPILIB}/cameraserver/src/main/native/include
		#${ALLWPILIB}/cscore/src/main/native/include
		#${ALLWPILIB}/ntcore/src/main/native/include
		#${ALLWPILIB}/ni-libraries/include
		#$ENV{HOME}/ni-libraries/src/include
		)
	add_definitions(--std=c++14)
	set (FRCROBOT_HW_MAIN_SRCS ${FRCROBOT_HW_MAIN_SRCS} 
		src/dummy_wpilib.cpp
		${ALLWPILIB}/hal/src/main/native/athena/Compressor.cpp
		${ALLWPILIB}/hal/src/main/native/athena/CANAPI.cpp
		${ALLWPILIB}/hal/src/main/native/athena/HALInitializer.cpp
		${ALLWPILIB}/hal/src/main/native/athena/PCMInternal.cpp
		${ALLWPILIB}/hal/src/main/native/athena/PDP.cpp
		${ALLWPILIB}/hal/src/main/native/athena/Solenoid.cpp
		${ALLWPILIB}/hal/src/main/native/athena/ctre/PCM.cpp
		${ALLWPILIB}/hal/src/main/native/athena/ctre/CtreCanNode.cpp
		${ALLWPILIB}/hal/src/main/native/cpp/handles/HandlesInternal.cpp
		)
else()
	set (FRCROBOT_HW_MAIN_SRCS ${FRCROBOT_HW_MAIN_SRCS} 
		src/rio_wpilib.cpp # Temp until 2019 code ready?
		)
endif()
#include_directories(${ALLWPILIB}/hal/src/main/native/athena)

add_executable(frcrobot_hw_main ${FRCROBOT_HW_MAIN_SRCS})

# Link against wpi user libraries for CAN talon and navX-MXP code and
# wpilib for HAL, driver station, and so on
target_link_libraries(frcrobot_hw_main
	${catkin_LIBRARIES}
	${PLATFORM_SPECIFIC_LIBS}
	${CTRE_USER_LIBS}
)

## Add cmake target dependencies of the executable
## same as for the library above. This is needed to force
## talon controller messages to be built before the
## talon controller code tries to use them
add_dependencies(frcrobot_hw_main
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
## Install ------------------------------------------------------------

# Install executables
install(TARGETS
  frcrobot_hw_main
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if (0)
# Test trajectory generator node
add_executable(${PROJECT_NAME}_test_trajectory src/tools/test_trajectory.cpp)
set_target_properties(${PROJECT_NAME}_test_trajectory PROPERTIES OUTPUT_NAME test_trajectory PREFIX "")
target_link_libraries(${PROJECT_NAME}_test_trajectory
  ${catkin_LIBRARIES}
)

## TOOLS ------------------------------------------------------

# Tool for analyzing controller performance
add_executable(${PROJECT_NAME}_controller_to_csv_main 
	src/tools/controller_to_csv_main.cpp
	src/tools/controller_to_csv.cpp)
set_target_properties(${PROJECT_NAME}_controller_to_csv_main PROPERTIES OUTPUT_NAME controller_to_csv_main PREFIX "")
target_link_libraries(${PROJECT_NAME}_controller_to_csv_main
  ${Gflags_LIBRARIES}
  ${catkin_LIBRARIES}
)

# Tool for analyzing controller performance
add_library(csv_to_controller src/tools/csv_to_controller.cpp)
target_link_libraries(csv_to_controller
  ${catkin_LIBRARIES}
)
add_executable(${PROJECT_NAME}_csv_to_controller_main 
	src/tools/csv_to_controller_main.cpp
	src/tools/csv_to_controller.cpp)
set_target_properties(${PROJECT_NAME}_csv_to_controller_main PROPERTIES OUTPUT_NAME csv_to_controller_main PREFIX "")
target_link_libraries(${PROJECT_NAME}_csv_to_controller_main
  ${Gflags_LIBRARIES}
  ${catkin_LIBRARIES}
)

# Tool for controlling a robot from keyboard
add_executable(${PROJECT_NAME}_keyboard_teleop src/tools/keyboard_teleop.cpp)
set_target_properties(${PROJECT_NAME}_keyboard_teleop PROPERTIES OUTPUT_NAME keyboard_teleop PREFIX "")
target_link_libraries(${PROJECT_NAME}_keyboard_teleop
  ${catkin_LIBRARIES}

install(TARGETS
    ${PROJECT_NAME}_test_trajectory
    ${PROJECT_NAME}_controller_to_csv_main
    ${PROJECT_NAME}_csv_to_controller_main
    ${PROJECT_NAME}_keyboard_teleop
    
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
)
endif()

## Install ------------------------------------------------------------
install(TARGETS
  frcrobot_sim_main
    
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install header files
install(DIRECTORY 
	include/${PROJECT_NAME}
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	PATTERN *~ EXCLUDE
	PATTERN *.sw? EXCLUDE
)

install(DIRECTORY
	launch
	config
	urdf
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
	PATTERN *~ EXCLUDE
	PATTERN *.sw? EXCLUDE
)
