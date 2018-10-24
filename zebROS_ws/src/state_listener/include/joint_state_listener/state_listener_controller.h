#pragma once

#include <controller_interface/controller.h>
#include <ros/node_handle.h>
#include <realtime_tools/realtime_buffer.h>
#include <sensor_msgs/JointState.h>
#include "remote_hardware_interface/remote_joint_interface.h"
#include "pdp_state_controller/PDPData.h"

namespace state_listener_controller
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
		ValueValid() : valid_(false) { }
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
				ROS_INFO_STREAM("State listener got joint " << j);
				handles_.push_back(hw->getHandle(j));
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
			//handles_.release();
		}

		virtual void update(const ros::Time & /*time*/, const ros::Duration & /*period*/) override
		{
			// Take the most recent set of values read from the joint_states
			// topic and write them to the local joints
			std::vector<ValueValid> vals = *command_buffer_.readFromRT();
			for (size_t i = 0; i < vals.size(); i++)
				if (vals[i].valid_)
					handles_[i].setCommand(vals[i].value_);
		}

	private:
		ros::Subscriber sub_command_;
		std::vector<std::string> joint_names_;
		std::vector<hardware_interface::JointHandle> handles_;

		// Real-time buffer holds the last command value read from the
		// "command" topic.
		realtime_tools::RealtimeBuffer<std::vector<ValueValid>> command_buffer_;

		// Iterate through each desired joint state.  If it is found in
		// the message, save the value here in the realtime buffer.
		// // TODO : figure out how to hack this to use a ConstPtr type instead
		virtual void commandCB(const sensor_msgs::JointStateConstPtr &msg)
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

class PDPStateListenerController :
	public controller_interface::Controller<hardware_interface::RemotePDPStateInterface>
{
	public:
		PDPStateListenerController() {}
		~PDPStateListenerController()
		{
			sub_command_.shutdown();
		}

		virtual bool init(hardware_interface::RemotePDPStateInterface *hw, ros::NodeHandle &n) override
		{
			// Read list of hw, make a list, grab handles for them, plus allocate storage space
			auto joint_names = hw->getNames();
			if (joint_names.size() == 0)
			{
				ROS_ERROR("PDP State Listener Controller : no remote pdp joints defined");
			}
			ROS_INFO_STREAM("State listener got joint " << joint_names[0]);
			handle_ = hw->getHandle(joint_names[0]);

			std::string topic;

			// get topic to subscribe to
			if (!n.getParam("topic", topic))
			{
				ROS_ERROR("Parameter 'topic' not set");
				return false;
			}

			sub_command_ = n.subscribe<pdp_state_controller::PDPData>(topic, 1, &PDPStateListenerController::commandCB, this);
			return true;
		}

		virtual void starting(const ros::Time & /*time*/) override
		{
		}
		virtual void stopping(const ros::Time & /*time*/) override
		{
			//handles_.release();
		}

		virtual void update(const ros::Time & /*time*/, const ros::Duration & /*period*/) override
		{
			const hardware_interface::PDPHWState data = *command_buffer_.readFromRT();

			handle_->setVoltage(data.getVoltage());
			handle_->setTemperature(data.getTemperature());
			handle_->setTotalCurrent(data.getTotalCurrent());
			handle_->setTotalPower(data.getTotalPower());
			handle_->setTotalEnergy(data.getTotalEnergy());
			for (size_t channel = 0; channel <= 15; channel++)
				handle_->setCurrent(data.getCurrent(channel), channel);
		}

	private:
		ros::Subscriber sub_command_;
		hardware_interface::PDPWritableStateHandle handle_;

		// Real-time buffer holds the last command value read from the
		// "command" topic.
		realtime_tools::RealtimeBuffer<hardware_interface::PDPHWState> command_buffer_;

		// Iterate through each desired joint state.  If it is found in
		// the message, save the value here in the realtime buffer.
		// // TODO : figure out how to hack this to use a ConstPtr type instead
		virtual void commandCB(const pdp_state_controller::PDPDataConstPtr &msg)
		{
			hardware_interface::PDPHWState data;

			data.setVoltage(msg->voltage);
			data.setTemperature(msg->temperature);
			data.setTotalCurrent(msg->totalCurrent);
			data.setTotalPower(msg->totalPower);
			data.setTotalEnergy(msg->totalEnergy);
			for (size_t channel = 0; channel <= 15; channel++)
				data.setCurrent(msg->current[channel], channel);
			std::vector<hardware_interface::PDPHWState> vec;
			vec.push_back(data);
			command_buffer_.writeFromNonRT(data);
		}
};
} // namespace
