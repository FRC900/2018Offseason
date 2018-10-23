#pragma once

#include <controller_interface/controller.h>
#include <ros/node_handle.h>
#include <realtime_tools/realtime_buffer.h>

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

// interface type, message type, value type

// since not all joint names are guaranteed to be found in the
// joint state message, keep track of which ones have using
// this class. Only write values during update if valid end up
// being true.
template <class VAL_TYPE>
class ValueValid
{
	public:
		ValueValid() : valid_(false) { }
		VAL_TYPE value_;
		bool     valid_;
};

template <class IF_TYPE, class HANDLE_TYPE, class MSG_TYPE, class VAL_TYPE>
class StateListenerController :
	public controller_interface::Controller<IF_TYPE>
{
	public:
		StateListenerController() {}
		~StateListenerController()
		{
			sub_command_.shutdown();
		}

		virtual bool init(IF_TYPE *hw, ros::NodeHandle &n) override
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
			sub_command_ = n.subscribe<MSG_TYPE>(topic, 1, &StateListenerController<IF_TYPE, HANDLE_TYPE, MSG_TYPE, VAL_TYPE>::commandCB, this);
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
			std::vector<ValueValid<VAL_TYPE>> vals = *command_buffer_.readFromRT();
			for (size_t i = 0; i < vals.size(); i++)
				if (vals[i].valid_)
					handles_[i].setCommand(vals[i].value_);
		}

	private:
		ros::Subscriber sub_command_;
		std::vector<std::string> joint_names_;
		std::vector<HANDLE_TYPE> handles_;

		// Real-time buffer holds the last command value read from the
		// "command" topic.
		realtime_tools::RealtimeBuffer<std::vector<ValueValid<VAL_TYPE>>> command_buffer_;

		// Iterate through each desired joint state.  If it is found in
		// the message, save the value here in the realtime buffer.
		// // TODO : figure out how to hack this to use a ConstPtr type instead
		void commandCB(MSG_TYPE msg)
		{
			std::vector<ValueValid<VAL_TYPE>> ret;
			ret.resize(joint_names_.size());
			for (size_t i = 0; i < joint_names_.size(); i++)
			{
				auto it = std::find(msg.name.cbegin(), msg.name.cend(), joint_names_[i]);
				if (it != msg.name.cend())
				{
					const size_t loc = it - msg.name.cbegin();
					ret[i].value_ = msg.position[loc];
					ret[i].valid_ = true;
				}
			}
			command_buffer_.writeFromNonRT(ret);
		}
};
} // namespace
