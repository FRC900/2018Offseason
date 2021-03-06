#include <ros_control_boilerplate/frcrobot_phoenixsim_interface.h>

/* additional include for sim */
#include <ctre/phoenix/cci/Platform_CCI.h> // c_SimCreate
#include <ctre/phoenix/cci/Unmanaged_CCI.h> // c_FeedEnable

#include <HALInitializer.h>

namespace frcrobot_control
{
FRCRobotPhoenixSimInterface::FRCRobotPhoenixSimInterface(ros::NodeHandle &nh, urdf::Model *urdf_model)
	: FRCRobotHWInterface(nh, urdf_model)
{
}

void FRCRobotPhoenixSimInterface::init(void)
{
	for (size_t i = 0; i < num_can_talon_srxs_; i++)
	{
		c_SimCreate(DeviceType::TalonSRXType,can_talon_srx_can_ids_[i]);
	}

	// for now we need a delay so backend can properly setup device properties
	// before we attempt creating API objects
	// this may not be necessary anymore
	ros::Duration(1.0).sleep();

	hal::init::InitializeHAL();
	FRCRobotHWInterface::init();
}


// Write should grab the motor output and use it to run 1 timestep
// of the underlying motor sim
void FRCRobotPhoenixSimInterface::write(ros::Duration &elapsed_time)
{
	ros::Duration(0.01).sleep();
	c_FeedEnable(500);
	FRCRobotHWInterface::write(elapsed_time);

	// Update the motor connected to each Talon here?
}
} // namespace
