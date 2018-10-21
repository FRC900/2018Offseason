#include "joint_state_listener/state_listener_controller.h"
#include <sensor_msgs/JointState.h>
#include "remote_hardware_interface/remote_joint_interface.h"

#include <pluginlib/class_list_macros.h>

namespace joint_state_listener_controller
{
typedef state_listener_controller::StateListenerController<
							hardware_interface::RemoteJointInterface,
							hardware_interface::JointHandle,
							sensor_msgs::JointState,
							double> JointStateListenerController;
}


PLUGINLIB_EXPORT_CLASS(joint_state_listener_controller::JointStateListenerController,
					   controller_interface::ControllerBase)
