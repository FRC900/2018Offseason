#pragma once

#include <controller_interface/controller.h>
#include <ros/node_handle.h>
#include <realtime_tools/realtime_buffer.h>
#include <sensor_msgs/JointState.h>
#include "remote_hardware_interface/remote_joint_interface.h"

namespace joint_state_listener_controller
{
/**
 * \brief Controller to read from joint state message and
 *        update local copies of the read joint values
 *
 * This controller is initialized with a list of joint names
 * which are remote to the current controller manager.  This
 * controller grabs values from a JointState message and writes
 * those values to the local copy of those remote joints
 *
 * \section ROS interface
 *
 * \param type Must be "RemoteJointInterface".
 *
 * Subscribes to:
 * - \b command (sensor_msgs::JointState) : The joint state topic
 */


// since not all joint names are guaranteed to be found in the
// joint state message, keep track of which ones have using
// this class. Only write values during update if valid end up
// being true.
class ValueValid
{
	public:
		ValueValid() : value_(0), valid_(false) { }
		double value_;
		bool   valid_;
};

class JointStateListenerController :
	public controller_interface::Controller<hardware_interface::RemoteJointInterface>
{
	public:
		JointStateListenerController() {}
		~JointStateListenerController()
		{
			sub_command_.shutdown();
		}

		virtual bool init(hardware_interface::RemoteJointInterface *hw, ros::NodeHandle &n) override
		{
			// Read list of hw, make a list, grab handles for them, plus allocate storage space
			joint_names_ = hw->getNames();
			for (auto j : joint_names_)
			{
				ROS_INFO_STREAM("Got joint " << j);
				ifs_.push_back(hw->getHandle(j));
			}

			std::string topic;

			// get topic to subscribe to
			if (!n.getParam("topic", topic))
			{
				ROS_ERROR("Parameter 'topic' not set");
				return false;
			}

			// Might wantt to make message type a template
			// parameter as well?
			sub_command_ = n.subscribe<sensor_msgs::JointState>(topic, 1, &JointStateListenerController::commandCB, this);
			return true;
		}

		virtual void starting(const ros::Time & /*time*/) override
		{
		}
		virtual void stopping(const ros::Time & /*time*/) override
		{
			//ifs_.release();
		}

		virtual void update(const ros::Time & /*time*/, const ros::Duration & /*period*/) override
		{
			// Take the most recent set of values read from the joint_states
			// topic and write them to the local joints
			std::vector<ValueValid> vals = *command_buffer_.readFromRT();
			for (size_t i = 0; i < vals.size(); i++)
				if (vals[i].valid_)
					ifs_[i].setCommand(vals[i].value_);
		}

	private:
		ros::Subscriber sub_command_;
		std::vector<std::string> joint_names_;
		std::vector<hardware_interface::JointHandle> ifs_;

		// Real-time buffer holds the last command value read from the
		// "command" topic.
		realtime_tools::RealtimeBuffer<std::vector<ValueValid>> command_buffer_;

		// Iterate through each desired joint state.  If it is found in
		// the message, save the value here in the realtime buffer.
		void commandCB(const sensor_msgs::JointStateConstPtr &msg)
		{
			std::vector<ValueValid> ret;
			ret.resize(joint_names_.size());
			for (size_t i = 0; i < joint_names_.size(); i++)
			{
				auto it = std::find(msg->name.cbegin(), msg->name.cend(), joint_names_[i]);
				if (it != msg->name.cend())
				{
					const size_t loc = it - msg->name.cbegin();
					ret[i].value_ = msg->position[loc];
					ret[i].valid_ = true;
				}
			}
			command_buffer_.writeFromNonRT(ret);
		}
};
} // namespace
