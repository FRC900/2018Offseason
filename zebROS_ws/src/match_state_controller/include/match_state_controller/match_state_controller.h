#ifndef MATCH_STATE_CONTROLLER
#define MATCH_STATE_CONTROLLER

#include <controller_interface/controller.h>
#include <realtime_tools/realtime_publisher.h>
#include <boost/shared_ptr.hpp>
#include <match_state_controller/match_data_interface.h>
#include "match_state_controller/MatchSpecificData.h"
#include <pluginlib/class_list_macros.h>

namespace match_state_controller
{
class MatchStateController: public controller_interface::Controller<hardware_interface::MatchStateInterface>
{
	public:
		MatchStateController() : publish_rate_(15.0) {}

		virtual bool init(hardware_interface::MatchStateInterface *hw,
						ros::NodeHandle					&root_nh,
						ros::NodeHandle					&controller_nh);

		virtual void starting(const ros::Time &time);
		virtual void update(const ros::Time &time, const ros::Duration & );
		virtual void stopping(const ros::Time &time);

	private:
		hardware_interface::MatchStateHandle match_state_;
		boost::shared_ptr<realtime_tools::RealtimePublisher<match_state_controller::MatchSpecificData> > realtime_pub_;
		ros::Time last_publish_time_;
		double publish_rate_;

}; //class

} //namespace

#endif
