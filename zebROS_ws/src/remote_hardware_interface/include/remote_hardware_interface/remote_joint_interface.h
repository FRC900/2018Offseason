#pragma once

#include <hardware_interface/joint_command_interface.h>

// Create a separate type of joint interface for joints which
// are read from hardware on another controller_manager.  This
// is just an easy way for a controller to get a full list of
// joints which aren't local.
namespace hardware_interface
{
	class RemoteJointInterface : public JointCommandInterface {};
}


