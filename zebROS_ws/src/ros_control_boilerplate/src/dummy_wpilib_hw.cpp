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

#include <frc/AnalogInput.h>
frc::AnalogInput::AnalogInput(int)
{
	ROS_ERROR("Called AnalogInput::AnalogInput(int) on unsupported platform");
}
frc::AnalogInput::~AnalogInput()
{
	ROS_ERROR("Called ::AnalogInput::~AnalogInput( on unsupported platform");
}

int frc::AnalogInput::GetValue() const
{
	ROS_ERROR("Called frc::AnalogInput::GetValue() const on unsupported platform");
	return -1;
}
int frc::AnalogInput::GetAverageValue() const
{
	ROS_ERROR("Called frc::AnalogInput::GetAverageValue() const on unsupported platform");
	return -1;
}
double frc::AnalogInput::GetVoltage() const
{
	ROS_ERROR("Called frc::AnalogInput::GetVoltage() const on unsupported platform");
	return std::numeric_limits<double>::max();
}
double frc::AnalogInput::GetAverageVoltage() const
{
	ROS_ERROR("Called frc::AnalogInput::GetAverageVoltage() const on unsupported platform");
	return std::numeric_limits<double>::max();
}
int frc::AnalogInput::GetChannel() const
{
	ROS_ERROR("Called frc::AnalogInput::GetChannel() const on unsupported platform");
	return -1;
}
void frc::AnalogInput::SetAverageBits(int bits)
{
	ROS_ERROR("Called frc::AnalogInput::SetAverageBits(int bits) on unsupported platform");
}
int frc::AnalogInput::GetAverageBits() const
{
	ROS_ERROR("Called frc::AnalogInput::GetAverageBits() const on unsupported platform");
	return -1;
}
void frc::AnalogInput::SetOversampleBits(int bits)
{
	ROS_ERROR("Called frc::AnalogInput::SetOversampleBits(int bits) on unsupported platform");
}
int frc::AnalogInput::GetOversampleBits() const
{
	ROS_ERROR("Called frc::AnalogInput::GetOversampleBits() const on unsupported platform");
	return -1;
}
int frc::AnalogInput::GetLSBWeight() const
{
	ROS_ERROR("Called frc::AnalogInput::GetLSBWeight() const on unsupported platform");
	return -1;
}
int frc::AnalogInput::GetOffset() const
{
	ROS_ERROR("Called frc::AnalogInput::GetOffset() const on unsupported platform");
	return -1;
}
bool frc::AnalogInput::IsAccumulatorChannel() const
{
	ROS_ERROR("Called frc::AnalogInput::IsAccumulatorChannel() const on unsupported platform");
	return false;
}
void frc::AnalogInput::InitAccumulator()
{
	ROS_ERROR("Called frc::AnalogInput::InitAccumulator() on unsupported platform");
}
void frc::AnalogInput::SetAccumulatorInitialValue(int64_t value)
{
	ROS_ERROR("Called frc::AnalogInput::SetAccumulatorInitialValue(int64_t value) on unsupported platform");
}
void frc::AnalogInput::ResetAccumulator()
{
	ROS_ERROR("Called frc::AnalogInput::ResetAccumulator() on unsupported platform");
}
void frc::AnalogInput::SetAccumulatorCenter(int center)
{
	ROS_ERROR("Called frc::AnalogInput::SetAccumulatorCenter(int center) on unsupported platform");
}
void frc::AnalogInput::SetAccumulatorDeadband(int deadband)
{
	ROS_ERROR("Called frc::AnalogInput::SetAccumulatorDeadband(int deadband) on unsupported platform");
}
int64_t frc::AnalogInput::GetAccumulatorValue() const
{
	ROS_ERROR("Called frc::AnalogInput::GetAccumulatorValue() const on unsupported platform");
	return -1;
}
int64_t frc::AnalogInput::GetAccumulatorCount() const
{
	ROS_ERROR("Called frc::AnalogInput::GetAccumulatorCount() const on unsupported platform");
	return -1;
}
void frc::AnalogInput::GetAccumulatorOutput(int64_t& value, int64_t& count) const
{
	ROS_ERROR("Called frc::AnalogInput::GetAccumulatorOutput(int64_t& value, int64_t& count) const on unsupported platform");
}
void frc::AnalogInput::SetSampleRate(double samplesPerSecond)
{
	ROS_ERROR("Called frc::AnalogInput::SetSampleRate(double samplesPerSecond) on unsupported platform");
}
double frc::AnalogInput::GetSampleRate()
{
	ROS_ERROR("Called frc::AnalogInput::GetSampleRate() on unsupported platform");
	return std::numeric_limits<double>::max();
}
double frc::AnalogInput::PIDGet()
{
	ROS_ERROR("Called frc::AnalogInput::PIDGet() on unsupported platform");
	return std::numeric_limits<double>::max();
}
void frc::AnalogInput::InitSendable(SendableBuilder& builder)
{
	ROS_ERROR("Called frc::AnalogInput::InitSendable(SendableBuilder& builder) on unsupported platform");
}
#include <frc/DigitalInput.h>
frc::DigitalInput::DigitalInput(int)
{
	ROS_ERROR("Called DigitalInput::DigitalInput(int) on unsupported platform");
}
frc::DigitalInput::~DigitalInput()
{
	ROS_ERROR("Called DigitalInput::~DigitalInput() on unsupported platform");
}
bool frc::DigitalInput::Get() const
{
	ROS_ERROR("Called DigitalInput::Get() const on unsupported platform");
	return false;
}
int frc::DigitalInput::GetChannel() const
{
	ROS_ERROR("Called frc::DigitalInput::GetChannel() const on unsupported platform");
	return std::numeric_limits<int>::max();
}
HAL_Handle frc::DigitalInput::GetPortHandleForRouting() const
{
	ROS_ERROR("Called frc::DigitalInput::GetPortHandleForRouting() const on unsupported platform");
	return HAL_Handle();
}
frc::AnalogTriggerType frc::DigitalInput::GetAnalogTriggerTypeForRouting() const
{
	ROS_ERROR("Called frc::DigitalInput::GetAnalogTriggerTypeForRouting() const on unsupported platform");
	return frc::AnalogTriggerType::kInWindow;
}
bool frc::DigitalInput::IsAnalogTrigger() const
{
	ROS_ERROR("Called frc::DigitalInput::IsAnalogTrigger() const on unsupported platform");
	return false;
}
void frc::DigitalInput::InitSendable(SendableBuilder& builder)
{
	ROS_ERROR("Called frc::DigitalInput::InitSendable(SendableBuilder& builder) on unsupported platform");
}

#include <frc/DigitalOutput.h>
frc::DigitalOutput::DigitalOutput(int)
{
	ROS_ERROR("Called DigitalOutput::DigitalOutput(int) on unsupported platform");
}
frc::DigitalOutput::~DigitalOutput()
{
	ROS_ERROR("Called DigitalOutput::~DigitalOutput() on unsupported platform");
}
void frc::DigitalOutput::Set(bool)
{
	ROS_ERROR("Called DigitalOutput::set(bool) on unsupported platform");
}
  void frc::DigitalOutput::InitSendable(SendableBuilder&)
{
	ROS_ERROR("Called frc::DigitalOutput::InitSendable(SendableBuilder& builder) on unsupported platform");
}

#include <frc/DriverStation.h>
class frc::MatchDataSender {
  MatchDataSender() {
  }
};

frc::DriverStation::DriverStation()
{
	ROS_ERROR("Called DriverStation::DriverStation() on unsupported platform");
}
frc::DriverStation::~DriverStation()
{
	ROS_ERROR("Called DriverStation:~:DriverStation() on unsupported platform");
}
frc::DriverStation & frc::DriverStation::GetInstance()
{
	ROS_ERROR("Called DriverStation::GetInstance() on unsupported platform");
	static frc::DriverStation d;
	return d;
}
bool frc::DriverStation::IsNewControlData() const
{
	ROS_ERROR("Called DriverStation::IsNewControlData() const on unsupported platform");
	return false;
}

#include <frc/ErrorBase.h>
frc::ErrorBase::ErrorBase()
{
	ROS_ERROR("Called ErrorBase::ErrorBase() on unsupported platform");
}

void frc::ErrorBase::ClearError() const
{
	ROS_ERROR("Called ErrorBase::ClearError() const on unsupported platform");
}
void frc::ErrorBase::CloneError(frc::ErrorBase const&) const
{
	ROS_ERROR("Called ErrorBase::CloneError(frc::ErrorBase const&) const on unsupported platform");
}
frc::Error &frc::ErrorBase::GetError()
{
	ROS_ERROR("Called ErrorBase::GetError() on unsupported platform");
	static frc::Error e;
	return e;
}
const frc::Error &frc::ErrorBase::GetError() const
{
	ROS_ERROR("Called ErrorBase::GetError() on unsupported platform");
	static frc::Error e;
	return e;
}
void frc::ErrorBase::SetErrnoError(wpi::Twine const&, wpi::StringRef, wpi::StringRef, int) const
{
	ROS_ERROR("Called ErrorBase::SetErrnoError(wpi::Twine const&, wpi::StringRef, wpi::StringRef, int) const on unsupported platform");
}
void frc::ErrorBase::SetError(int, wpi::Twine const&, wpi::StringRef, wpi::StringRef, int) const
{
	ROS_ERROR("Called ErrorBase::SetError(int, wpi::Twine const&, wpi::StringRef, wpi::StringRef, int) const on unsupported platform");
}
void frc::ErrorBase::SetErrorRange(int, int, int, int, wpi::Twine const&, wpi::StringRef, wpi::StringRef, int) const
{
	ROS_ERROR("Called ErrorBase::SetErrorRange(int, int, int, int, wpi::Twine const&, wpi::StringRef, wpi::StringRef, int) const on unsupported platform");
}
void frc::ErrorBase::SetImaqError(int, wpi::Twine const&, wpi::StringRef, wpi::StringRef, int) const
{
	ROS_ERROR("Called ErrorBase::SetImaqError(int, wpi::Twine const&, wpi::StringRef, wpi::StringRef, int) const on unsupported platform");
}
void frc::ErrorBase::SetWPIError(wpi::Twine const&, int, wpi::Twine const&, wpi::StringRef, wpi::StringRef, int) const
{
	ROS_ERROR("Called ErrorBase::SetWPIError(wpi::Twine const&, int, wpi::Twine const&, wpi::StringRef, wpi::StringRef, int) const on unsupported platform");
}
bool frc::ErrorBase::StatusIsFatal() const
{
	ROS_ERROR("Called ErrorBase::StatusIsFatal() const on unsupported platform");
	return false;
}
#include <frc/IterativeRobotBase.h>
frc::IterativeRobotBase::IterativeRobotBase(double period)
    : m_period(period),
      m_watchdog(period, [this] { } )
{
	ROS_ERROR("Called IterativeRobotBase::IterativeRobotBase(double) on unsupported platform");
}
void frc::IterativeRobotBase::AutonomousInit()
{
	ROS_ERROR("Called IterativeRobotBase::AutonomousInit() on unsupported platform");
}
void frc::IterativeRobotBase::AutonomousPeriodic()
{
	ROS_ERROR("Called IterativeRobotBase::AutonomousPeriodic() on unsupported platform");
}
void frc::IterativeRobotBase::DisabledInit()
{
	ROS_ERROR("Called IterativeRobotBase::DisabledInit() on unsupported platform");
}
void frc::IterativeRobotBase::DisabledPeriodic()
{
	ROS_ERROR("Called IterativeRobotBase::DisabledPeriodic() on unsupported platform");
}
void frc::IterativeRobotBase::RobotInit()
{
	ROS_ERROR("Called IterativeRobotBase::RobotInit() on unsupported platform");
}
void frc::IterativeRobotBase::RobotPeriodic()
{
	ROS_ERROR("Called IterativeRobotBase::RobotPeriodic() on unsupported platform");
}
void frc::IterativeRobotBase::TeleopInit()
{
	ROS_ERROR("Called IterativeRobotBase::TeleopInit() on unsupported platform");
}
void frc::IterativeRobotBase::TeleopPeriodic()
{
	ROS_ERROR("Called IterativeRobotBase::TeleopPeriodic() on unsupported platform");
}
void frc::IterativeRobotBase::TestInit()
{
	ROS_ERROR("Called IterativeRobotBase::TestInit() on unsupported platform");
}
void frc::IterativeRobotBase::TestPeriodic()
{
	ROS_ERROR("Called IterativeRobotBase::TestPeriodic() on unsupported platform");
}

#include <frc/Watchdog.h>

frc::Watchdog::Watchdog(double timeout, std::function<void()> callback)
    : m_timeout(static_cast<int64_t>(timeout * 1.0e6))
    , m_callback(callback)
{
	ROS_ERROR("Called frc::Watchdog::Watchdog(double timeout, std::function<void()> callback) on unsupported platform");
}

frc::Watchdog::~Watchdog()
{
	ROS_ERROR("Called frc::Watchdog::~Watchdog() on unsupported platform");
}

#include <frc/livewindow/LiveWindow.h>
frc::LiveWindow *frc::LiveWindow::GetInstance()
{
	ROS_ERROR("Called LiveWindow::GetInstance() on unsupported platform");
	return nullptr;
}
void frc::LiveWindow::SetEnabled(bool)
{
	ROS_ERROR("Called LiveWindow::SetEnabled(bool) on unsupported platform");
}
void frc::LiveWindow::DisableAllTelemetry()
{
	ROS_ERROR("Called LiveWindow::DisableAllTelemetry() on unsupported platform");
}
void frc::LiveWindow::UpdateValues()
{
	ROS_ERROR("Called LiveWindow::UpdateValues() on unsupported platform");
}


#include <frc/smartdashboard/SendableBase.h>
frc::SendableBase::SendableBase(bool)
{
	ROS_ERROR("Called SendableBase::SendableBase(bool) on unsupported platform");
}
std::string frc::SendableBase::GetName() const
{
	ROS_ERROR("Called string frc::SendableBase::GetName() const on unsupported platform");
	return std::string();
}
void frc::SendableBase::SetName(const wpi::Twine& name)
{
	ROS_ERROR("Called ::SendableBase::SetName(const wpi::Twine& name) on unsupported platform");
}
std::string frc::SendableBase::GetSubsystem() const
{
	ROS_ERROR("Called string frc::SendableBase::GetSubsystem() const on unsupported platform");
	return std::string();
}
void frc::SendableBase::SetSubsystem(const wpi::Twine& subsystem)
{
	ROS_ERROR("Called ::SendableBase::SetSubsystem(const wpi::Twine& subsystem) on unsupported platform");
}

frc::SendableBase::~SendableBase()
{
	ROS_ERROR("Called SendableBase::~SendableBase() on unsupported platform");
}

#include <frc/smartdashboard/SmartDashboard.h>
bool frc::SmartDashboard::PutBoolean(wpi::StringRef, bool)
{
	ROS_ERROR("Called SmartDashboard::PutBoolean(wpi::StringRef, bool) on unsupported platform");
	return false;
}
bool frc::SmartDashboard::PutNumber(wpi::StringRef, double)
{
	ROS_ERROR("Called SmartDashboard::PutNumber(wpi::StringRef, double) on unsupported platform");
	return false;
}

#include <networktables/NetworkTable.h>
bool nt::NetworkTable::GetBoolean(wpi::StringRef, bool) const
{
	ROS_ERROR("Called NetworkTable::GetBoolean(wpi::StringRef, bool) const on unsupported platform");
	return false;
}
double nt::NetworkTable::GetNumber(wpi::StringRef, double) const
{
	ROS_ERROR("Called NetworkTable::GetNumber(wpi::StringRef, double) const on unsupported platform");
	return std::numeric_limits<double>::max();
}
std::shared_ptr<NetworkTable> nt::NetworkTable::GetTable(wpi::StringRef)
{
	ROS_ERROR("Called NetworkTable::GetTable(wpi::StringRef) on unsupported platform");
	return nullptr;
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

int64_t HAL_Report(int32_t resource, int32_t instanceNumber,
		int32_t context, const char* feature)
{
	ROS_INFO_STREAM("HAL_Report resource = " << resource << " instanceNumber = " << instanceNumber <<
			" context = " << context << " feature = " << feature);
	return -1;
}

int32_t HAL_GetControlWord(HAL_ControlWord* controlWord)
{
	ROS_INFO_STREAM("Called HAL_GetControlWord() on unsupported platform");
	return -1;
}

HAL_AllianceStationID HAL_GetAllianceStation(int32_t* status)
{
	ROS_INFO_STREAM("Called HAL_GetAllianceStation() on unsupported platform");
	return static_cast<HAL_AllianceStationID>(-1);
}
double HAL_GetMatchTime(int32_t* status)
{
	ROS_INFO_STREAM("Called HAL_GetMatchTime() on unsupported platform");
	return -1;
}
int32_t HAL_GetMatchInfo(HAL_MatchInfo* info)
{
	ROS_INFO_STREAM("Called HAL_GetMatchInfo() on unsupported platform");
	return -1;
}

} /// extern "C"

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


