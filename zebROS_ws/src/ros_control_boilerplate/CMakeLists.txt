cmake_minimum_required(VERSION 2.8.3)
project(ros_control_boilerplate)

include ("../cmake_modules/CMakeOpt.cmake")
add_definitions(--std=c++14)

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
  frc_msgs
  frc_interfaces
)

add_message_files (
  FILES
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
#find_package(Gflags REQUIRED)
#find_package(Eigen3 REQUIRED)

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
	frc_msgs
	frc_interfaces
	sensor_msgs
	rosparam_shortcuts
	talon_interface
	message_runtime
  LIBRARIES
)

###########
## Build ##
###########

include_directories(
  include/
  ${catkin_INCLUDE_DIRS}
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

set (WPI_SYSROOT $ENV{HOME}/frc2019/roborio/arm-frc2019-linux-gnueabi)
include_directories(
	${catkin_INCLUDE_DIRS}
	${WPI_SYSROOT}/include/wpilib
	${WPI_SYSROOT}/include
	${WPI_SYSROOT}/include/navx
)

# Base regardless of OS type
set (WPI_BASE_LIB_DIR   ${WPI_SYSROOT}/lib/wpilib/linux)

# Rio-only code
if ("${ARM_PREFIX}" STREQUAL "arm-frc2019-linux-gnueabi")
	set (CTRE_LIB_DIR ${WPI_SYSROOT}/lib/ctre/linux/athena/static)

	# Link against wpi user libraries for CAN talon and navX-MXP code and
	# wpilib for HAL, driver station, and so on
	set (WPI_SHARED_LIB_DIR ${WPI_BASE_LIB_DIR}/athena/shared)
	set (PLATFORM_SPECIFIC_LIBS
		${WPI_SHARED_LIB_DIR}/libcameraserverd.so
		${WPI_SHARED_LIB_DIR}/libntcored.so
		${WPI_SHARED_LIB_DIR}/libvisa.so
		${WPI_SHARED_LIB_DIR}/libwpilibcd.so
		${WPI_SHARED_LIB_DIR}/libcscored.so
		${WPI_SHARED_LIB_DIR}/libFRC_NetworkCommunication.so.19.0.0
		${WPI_SHARED_LIB_DIR}/libNiFpga.so.18.0.0
		${WPI_SHARED_LIB_DIR}/libNiFpgaLv.so.18.0.0
		${WPI_SHARED_LIB_DIR}/libRoboRIO_FRC_ChipObject.so.19.0.0
		${WPI_SHARED_LIB_DIR}/libNiRioSrv.so.18.0.0
		${WPI_SHARED_LIB_DIR}/libniriodevenum.so.18.0.0
		${WPI_SHARED_LIB_DIR}/libniriosession.so.18.0.0
		${WPI_SHARED_LIB_DIR}/libopencv_calib3d.so.3.4
		${WPI_SHARED_LIB_DIR}/libopencv_imgproc.so.3.4
		${WPI_SHARED_LIB_DIR}/libopencv_superres.so.3.4
		${WPI_SHARED_LIB_DIR}/libopencv_core.so.3.4
		${WPI_SHARED_LIB_DIR}/libopencv_ml.so.3.4
		${WPI_SHARED_LIB_DIR}/libopencv_videoio.so.3.4
		${WPI_SHARED_LIB_DIR}/libopencv_features2d.so.3.4
		${WPI_SHARED_LIB_DIR}/libopencv_objdetect.so.3.4
		${WPI_SHARED_LIB_DIR}/libopencv_video.so.3.4
		${WPI_SHARED_LIB_DIR}/libopencv_flann.so.3.4
		${WPI_SHARED_LIB_DIR}/libopencv_photo.so.3.4
		${WPI_SHARED_LIB_DIR}/libopencv_videostab.so.3.4
		${WPI_SHARED_LIB_DIR}/libopencv_highgui.so.3.4
		${WPI_SHARED_LIB_DIR}/libopencv_shape.so.3.4
		${WPI_SHARED_LIB_DIR}/libopencv_imgcodecs.so.3.4
		${WPI_SHARED_LIB_DIR}/libopencv_stitching.so.3.4
		${WPI_SHARED_LIB_DIR}/libwpiHald.so
		${WPI_SHARED_LIB_DIR}/libwpiutild.so
		${WPI_SYSROOT}/lib/navx/linux/athena/static/libnavx_frc.a
	)
elseif ("${CMAKE_LIBRARY_ARCHITECTURE}" STREQUAL "arm-linux-gnueabihf") # Jetson TK1
	set (CTRE_LIB_DIR ${WPI_SYSROOT}/lib/ctre/linux/armhf/static)
elseif ("${CMAKE_LIBRARY_ARCHITECTURE}" STREQUAL "aarch64-linux-gnu") # Jetson TX1/TX2
	set (CTRE_LIB_DIR ${WPI_SYSROOT}/lib/ctre/linux/aarch64/static)
else() # x86-64, hopefully. First person to try and build native on a Mac gets kicked off the team
	set (CTRE_LIB_DIR ${WPI_SYSROOT}/lib/ctre/linux/x86-64/static)
endif()
  
# All platforms need these, they just live
# in different dirs for different architectures
set (CTRE_USER_LIBS
	${CTRE_LIB_DIR}/libCTRE_Phoenix.a
	${CTRE_LIB_DIR}/libCTRE_PhoenixCCI.a
)

set (ALLWPILIB ../../../allwpilib)
# Used to find HALInitializer from wpilib internal repo
include_directories(${ALLWPILIB}/hal/src/main/native/athena)

# Add socketcan library to everything but the Rio
# This adds code to use linux CAN sockets to talk to
# the CAN bus for the Jetson
if (NOT "${ARM_PREFIX}" STREQUAL "arm-frc2019-linux-gnueabi")
	set (CTRE_USER_LIBS 
		${CTRE_USER_LIBS}
		${CTRE_LIB_DIR}/libCTRE_PhoenixPlatform_socketcan.a
		${CTRE_LIB_DIR}/libCTRE_PhoenixCanutils.a
	)

	set (FRCROBOT_HW_MAIN_SRCS ${FRCROBOT_HW_MAIN_SRCS} 
		src/dummy_wpilib_common.cpp
		src/dummy_wpilib_hw.cpp
		${ALLWPILIB}/hal/src/main/native/athena/Compressor.cpp
		${ALLWPILIB}/hal/src/main/native/athena/CANAPI.cpp
		${ALLWPILIB}/hal/src/main/native/athena/HALInitializer.cpp
		${ALLWPILIB}/hal/src/main/native/athena/PCMInternal.cpp
		${ALLWPILIB}/hal/src/main/native/athena/PDP.cpp
		${ALLWPILIB}/hal/src/main/native/athena/Solenoid.cpp
		${ALLWPILIB}/hal/src/main/native/athena/ctre/PCM.cpp
		${ALLWPILIB}/hal/src/main/native/athena/ctre/CtreCanNode.cpp
		${ALLWPILIB}/hal/src/main/native/cpp/handles/HandlesInternal.cpp
		${ALLWPILIB}/wpiutil/src/main/native/cpp/memory.cpp
		${ALLWPILIB}/wpiutil/src/main/native/cpp/llvm/raw_ostream.cpp
		${ALLWPILIB}/wpiutil/src/main/native/cpp/llvm/Hashing.cpp
		${ALLWPILIB}/wpiutil/src/main/native/cpp/llvm/NativeFormatting.cpp
		${ALLWPILIB}/wpiutil/src/main/native/cpp/llvm/Path.cpp
		${ALLWPILIB}/wpiutil/src/main/native/cpp/llvm/SmallVector.cpp
		${ALLWPILIB}/wpiutil/src/main/native/cpp/llvm/StringRef.cpp
		${ALLWPILIB}/wpiutil/src/main/native/cpp/llvm/Twine.cpp
	)
file(GLOB
	ntcore_native_src ${ALLWPILIB}/ntcore/src/main/native/cpp/*.cpp
	ntcore_native_src ${ALLWPILIB}/ntcore/src/main/native/cpp/networktables/*.cpp
	ntcore_native_src ${ALLWPILIB}/ntcore/src/main/native/cpp/tables/*.cpp)

	add_executable(frcrobot_phoenixsim_main
		src/frcrobot_phoenixsim_interface.cpp
		src/frcrobot_hw_main.cpp
		src/frcrobot_hw_interface.cpp
		src/frc_robot_interface.cpp
		src/generic_hw_control_loop.cpp
		src/dummy_wpilib_common.cpp
		src/dummy_wpilib_phoenixsim.cpp
		${ALLWPILIB}/hal/src/main/native/cpp/cpp/fpga_clock.cpp
		${ALLWPILIB}/hal/src/main/native/cpp/handles/HandlesInternal.cpp
		${ALLWPILIB}/hal/src/main/native/sim/AnalogAccumulator.cpp
		${ALLWPILIB}/hal/src/main/native/sim/AnalogInput.cpp
		${ALLWPILIB}/hal/src/main/native/sim/AnalogInternal.cpp
		${ALLWPILIB}/hal/src/main/native/sim/AnalogOutput.cpp
		${ALLWPILIB}/hal/src/main/native/sim/AnalogTrigger.cpp
		${ALLWPILIB}/hal/src/main/native/sim/CAN.cpp
		${ALLWPILIB}/hal/src/main/native/sim/CANAPI.cpp
		${ALLWPILIB}/hal/src/main/native/sim/Compressor.cpp
		${ALLWPILIB}/hal/src/main/native/sim/Constants.cpp
		${ALLWPILIB}/hal/src/main/native/sim/Counter.cpp
		${ALLWPILIB}/hal/src/main/native/sim/DigitalInternal.cpp
		${ALLWPILIB}/hal/src/main/native/sim/DIO.cpp
		${ALLWPILIB}/hal/src/main/native/sim/DriverStation.cpp
		${ALLWPILIB}/hal/src/main/native/sim/Extensions.cpp
		${ALLWPILIB}/hal/src/main/native/sim/HAL.cpp
		${ALLWPILIB}/hal/src/main/native/sim/HALInitializer.cpp
		${ALLWPILIB}/hal/src/main/native/sim/Interrupts.cpp
		${ALLWPILIB}/hal/src/main/native/sim/MockHooks.cpp
		${ALLWPILIB}/hal/src/main/native/sim/Notifier.cpp
		${ALLWPILIB}/hal/src/main/native/sim/PDP.cpp
		${ALLWPILIB}/hal/src/main/native/sim/Ports.cpp
		${ALLWPILIB}/hal/src/main/native/sim/Power.cpp
		${ALLWPILIB}/hal/src/main/native/sim/PWM.cpp
		${ALLWPILIB}/hal/src/main/native/sim/Relay.cpp
		${ALLWPILIB}/hal/src/main/native/sim/Solenoid.cpp
		${ALLWPILIB}/hal/src/main/native/sim/Threads.cpp
		${ALLWPILIB}/hal/src/main/native/sim/mockdata/AnalogInData.cpp
		${ALLWPILIB}/hal/src/main/native/sim/mockdata/AnalogOutData.cpp
		${ALLWPILIB}/hal/src/main/native/sim/mockdata/AnalogTriggerData.cpp
		${ALLWPILIB}/hal/src/main/native/sim/mockdata/CanDataInternal.cpp
		${ALLWPILIB}/hal/src/main/native/sim/mockdata/DigitalPWMData.cpp
		${ALLWPILIB}/hal/src/main/native/sim/mockdata/DIOData.cpp
		${ALLWPILIB}/hal/src/main/native/sim/mockdata/DriverStationData.cpp
		${ALLWPILIB}/hal/src/main/native/sim/mockdata/PCMData.cpp
		${ALLWPILIB}/hal/src/main/native/sim/mockdata/PDPData.cpp
		${ALLWPILIB}/hal/src/main/native/sim/mockdata/PWMData.cpp
		${ALLWPILIB}/hal/src/main/native/sim/mockdata/RelayData.cpp
		${ALLWPILIB}/hal/src/main/native/sim/mockdata/RoboRioData.cpp
		${ALLWPILIB}/wpilibc/src/main/native/cpp/AnalogInput.cpp
		${ALLWPILIB}/wpilibc/src/main/native/cpp/DigitalInput.cpp
		${ALLWPILIB}/wpilibc/src/main/native/cpp/DigitalOutput.cpp
		${ALLWPILIB}/wpilibc/src/main/native/cpp/DriverStation.cpp
		${ALLWPILIB}/wpilibc/src/main/native/cpp/Error.cpp
		${ALLWPILIB}/wpilibc/src/main/native/cpp/ErrorBase.cpp
		${ALLWPILIB}/wpilibc/src/main/native/cpp/IterativeRobotBase.cpp
		${ALLWPILIB}/wpilibc/src/main/native/cpp/RobotController.cpp
		${ALLWPILIB}/wpilibc/src/main/native/cpp/RobotState.cpp
		${ALLWPILIB}/wpilibc/src/main/native/cpp/SensorUtil.cpp
		${ALLWPILIB}/wpilibc/src/main/native/cpp/Timer.cpp
		${ALLWPILIB}/wpilibc/src/main/native/cpp/Watchdog.cpp
		${ALLWPILIB}/wpilibc/src/main/native/cpp/Utility.cpp
		${ALLWPILIB}/wpilibc/src/main/native/cpp/commands/Command.cpp
		${ALLWPILIB}/wpilibc/src/main/native/cpp/commands/Scheduler.cpp
		${ALLWPILIB}/wpilibc/src/main/native/cpp/commands/Subsystem.cpp
		${ALLWPILIB}/wpilibc/src/main/native/cpp/livewindow/LiveWindow.cpp
		${ALLWPILIB}/wpilibc/src/main/native/cpp/smartdashboard/SendableBase.cpp
		${ALLWPILIB}/wpilibc/src/main/native/cpp/smartdashboard/SendableBuilderImpl.cpp
		${ALLWPILIB}/wpilibc/src/main/native/cpp/smartdashboard/SmartDashboard.cpp
		${ALLWPILIB}/wpiutil/src/main/native/cpp/leb128.cpp
		${ALLWPILIB}/wpiutil/src/main/native/cpp/memory.cpp
		${ALLWPILIB}/wpiutil/src/main/native/cpp/raw_istream.cpp
		${ALLWPILIB}/wpiutil/src/main/native/cpp/raw_socket_istream.cpp
		${ALLWPILIB}/wpiutil/src/main/native/cpp/raw_socket_ostream.cpp
		${ALLWPILIB}/wpiutil/src/main/native/cpp/timestamp.cpp
		${ALLWPILIB}/wpiutil/src/main/native/cpp/Base64.cpp
		${ALLWPILIB}/wpiutil/src/main/native/cpp/SafeThread.cpp
		${ALLWPILIB}/wpiutil/src/main/native/cpp/SocketError.cpp
		${ALLWPILIB}/wpiutil/src/main/native/cpp/TCPAcceptor.cpp
		${ALLWPILIB}/wpiutil/src/main/native/cpp/TCPConnector.cpp
		${ALLWPILIB}/wpiutil/src/main/native/cpp/TCPConnector_parallel.cpp
		${ALLWPILIB}/wpiutil/src/main/native/cpp/TCPStream.cpp
		${ALLWPILIB}/wpiutil/src/main/native/cpp/llvm/raw_ostream.cpp
		${ALLWPILIB}/wpiutil/src/main/native/cpp/llvm/Hashing.cpp
		${ALLWPILIB}/wpiutil/src/main/native/cpp/llvm/NativeFormatting.cpp
		${ALLWPILIB}/wpiutil/src/main/native/cpp/llvm/Path.cpp
		${ALLWPILIB}/wpiutil/src/main/native/cpp/llvm/SmallPtrSet.cpp
		${ALLWPILIB}/wpiutil/src/main/native/cpp/llvm/SmallVector.cpp
		${ALLWPILIB}/wpiutil/src/main/native/cpp/llvm/StringMap.cpp
		${ALLWPILIB}/wpiutil/src/main/native/cpp/llvm/StringRef.cpp
		${ALLWPILIB}/wpiutil/src/main/native/cpp/llvm/Twine.cpp
		${ntcore_native_src}
		)

	target_link_libraries(frcrobot_phoenixsim_main
		${catkin_LIBRARIES}
		${CTRE_LIB_DIR}/libCTRE_Phoenix.a
		${CTRE_LIB_DIR}/libCTRE_PhoenixCCI.a
		${CTRE_LIB_DIR}/libCTRE_PhoenixCanutils.a
		${CTRE_LIB_DIR}/libCTRE_PhoenixPlatform_sim.a
		)
	## Add cmake target dependencies of the executable
	## same as for the library above. This is needed to force
	## talon controller messages to be built before the
	## talon controller code tries to use them
	add_dependencies(frcrobot_phoenixsim_main
		${${PROJECT_NAME}_EXPORTED_TARGETS}
		${catkin_EXPORTED_TARGETS}
		)

	# Install executable on the off chance you'd ever
	# want to run sim on the RoboRIO
	install(TARGETS
	  frcrobot_phoenixsim_main
	  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	)

endif()

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
