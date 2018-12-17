#include <ctre/phoenix/platform/Platform.h>
#include <ros/ros.h>
extern "C"
{
	// This is the path for calls going through the new CANAPI.  Accesses
	// via these functions have already been through the CAN status
	// cache and were not found
	void HAL_CAN_SendMessage(uint32_t messageID, const uint8_t *data, uint8_t dataSize, int32_t periodMs, int32_t *status)
	{
		ctre::phoenix::platform::can::CANComm_SendMessage(messageID, data, dataSize, periodMs, status);
	}
	void HAL_CAN_ReceiveMessage(uint32_t *messageID, uint32_t messageIDMask, uint8_t *data, uint8_t *dataSize, uint32_t *timeStamp, int32_t *status)
	{
		ctre::phoenix::platform::can::CANComm_ReceiveMessage(messageID, messageIDMask, data, dataSize, timeStamp, status);
	}
}

#include <hal/Types.h>
#include <HALInitializer.h>
extern "C" {

#include <sys/time.h>
uint64_t HAL_GetFPGATime(int32_t* status)
{
	*status = 0;
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return ((uint64_t)tv.tv_sec * 1000000) + tv.tv_usec;
}

HAL_Bool HAL_Initialize(int32_t timeout, int32_t mode)
{
	hal::init::HAL_IsInitialized.store(true);
	return true;
}

int32_t HAL_GetFPGAVersion(int32_t* status)
{
	ROS_ERROR("Called HAL_GetFPGAVersion() on unsupported platform");
	*status = 0;
	return -900;  // Automatically script this at some point
}

int64_t HAL_GetFPGARevision(int32_t* status)
{
	ROS_ERROR("Called HAL_GetFPGARevision() on unsupported platform");
	*status = 0;
	return -900;  // TODO: Find a better number to return;
}

HAL_Bool HAL_GetFPGAButton(int32_t* status)
{
	ROS_ERROR("Called HAL_GetFPGAButton() on unsupported platform");
	*status = 0;
	return false;
}

HAL_Bool HAL_GetSystemActive(int32_t* status)
{
	ROS_ERROR("Called HAL_GetSystemActive() on unsupported platform");
	*status = 0;
	return true;
}

HAL_Bool HAL_GetBrownedOut(int32_t* status)
{
	ROS_ERROR("Called HAL_GetBrownedOut() on unsupported platform");
	*status = 0;
	return false;
}

double HAL_GetVinVoltage(int32_t* status)
{
	ROS_ERROR("Called HAL_GetVinVoltage() on unsupported platform");
	*status = 0;
	return -1;
}
double HAL_GetVinCurrent(int32_t* status)
{
	ROS_ERROR("Called HAL_GetVinCurrent() on unsupported platform");
	*status = 0;
	return -1;
}
double HAL_GetUserVoltage6V(int32_t* status)
{
	ROS_ERROR("Called HAL_GetUserVoltage6V() on unsupported platform");
	*status = 0;
	return -1;
}
double HAL_GetUserCurrent6V(int32_t* status)
{
	ROS_ERROR("Called HAL_GetUserCurrent6V() on unsupported platform");
	*status = 0;
	return -1;
}
HAL_Bool HAL_GetUserActive6V(int32_t* status)
{
	ROS_ERROR("Called HAL_GetUserActive6V() on unsupported platform");
	*status = 0;
	return false;
}
int32_t HAL_GetUserCurrentFaults6V(int32_t* status)
{
	ROS_ERROR("Called HAL_GetUserCurrentFaults6V() on unsupported platform");
	*status = 0;
	return -1;
}
double HAL_GetUserVoltage5V(int32_t* status)
{
	ROS_ERROR("Called HAL_GetUserVoltage5V() on unsupported platform");
	*status = 0;
	return -1;
}
double HAL_GetUserCurrent5V(int32_t* status)
{
	ROS_ERROR("Called HAL_GetUserCurrent5V() on unsupported platform");
	*status = 0;
	return -1;
}
HAL_Bool HAL_GetUserActive5V(int32_t* status)
{
	ROS_ERROR("Called HAL_GetUserActive5V() on unsupported platform");
	*status = 0;
	return false;
}
int32_t HAL_GetUserCurrentFaults5V(int32_t* status)
{
	ROS_ERROR("Called HAL_GetUserCurrentFaults5V() on unsupported platform");
	*status = 0;
	return -1;
}
double HAL_GetUserVoltage3V3(int32_t* status)
{
	ROS_ERROR("Called HAL_GetUserVoltage3V3() on unsupported platform");
	*status = 0;
	return -1;
}
double HAL_GetUserCurrent3V3(int32_t* status)
{
	ROS_ERROR("Called HAL_GetUserCurrent3V3() on unsupported platform");
	*status = 0;
	return -1;
}
HAL_Bool HAL_GetUserActive3V3(int32_t* status)
{
	ROS_ERROR("Called HAL_GetUserActive3V3() on unsupported platform");
	*status = 0;
	return false;
}
int32_t HAL_GetUserCurrentFaults3V3(int32_t* status)
{
	ROS_ERROR("Called HAL_GetUserCurrentFaults3V3() on unsupported platform");
	*status = 0;
	return -1;
}
void HAL_CAN_GetCANStatus(float* percentBusUtilization, uint32_t* busOffCount,
                          uint32_t* txFullCount, uint32_t* receiveErrorCount,
                          uint32_t* transmitErrorCount, int32_t* status)
{
	ROS_ERROR("Called HAL_CAN_GetCANStatus() on unsupported platform");
	*percentBusUtilization = -1;
	*busOffCount = -1;
	*txFullCount - -1;
	*receiveErrorCount = -1;
	*transmitErrorCount = -1;
	*status = 0;
}

} /// extern "C"

extern "C" {
int64_t HAL_Report(int32_t resource, int32_t instanceNumber,
		int32_t context, const char* feature)
{
	ROS_INFO_STREAM("HAL_Report resource = " << resource << " instanceNumber = " << instanceNumber <<
			" context = " << context << " feature = " << feature);
	return -1;
}
}

#include <hal/DriverStation.h>

void HAL_ObserveUserProgramAutonomous(void)
{
	ROS_ERROR("Called HAL_ObserveUserProgramAutonomous(void) on unsupported platform");
}
void HAL_ObserveUserProgramDisabled(void)
{
	ROS_ERROR("Called HAL_ObserveUserProgramDisabled(void) on unsupported platform");
}
void HAL_ObserveUserProgramStarting(void)
{
	ROS_ERROR("Called HAL_ObserveUserProgramStarting(void) on unsupported platform");
}
void HAL_ObserveUserProgramTeleop(void)
{
	ROS_ERROR("Called HAL_ObserveUserProgramTeleop(void) on unsupported platform");
}
void HAL_ObserveUserProgramTest(void)
{
	ROS_ERROR("Called HAL_ObserveUserProgramTest(void) on unsupported platform");
}
int32_t HAL_SetJoystickOutputs(int32_t joystickNum, int64_t outputs,
                               int32_t leftRumble, int32_t rightRumble)
{
	ROS_ERROR("Called HAL_SetJoystickOutputs(int32_t joystickNum, int64_t outputs, int32_t leftRumble, int32_t rightRumble) on unsupported device");
	return -1;
}

// From wpilib HAL.cpp
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2016-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#include "hal/handles/HandlesInternal.h"
extern "C" {

/**
 * @deprecated Uses module numbers
 */
HAL_PortHandle HAL_GetPortWithModule(int32_t module, int32_t channel) {
  // Dont allow a number that wouldn't fit in a uint8_t
  if (channel < 0 || channel >= 255) return HAL_kInvalidHandle;
  if (module < 0 || module >= 255) return HAL_kInvalidHandle;
  return hal::createPortHandle(channel, module);
}
} // extern "C"


