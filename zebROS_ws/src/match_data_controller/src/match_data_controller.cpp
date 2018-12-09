#include "match_data_controller/match_data_controller.h"
#include "match_data_controller/MatchSpecificData.h"
#include <cstddef>
#include <algorithm>

namespace match_data_controller
{

bool MatchStateController::init(hardware_interface::MatchStateInterface *hw,
	
	ros::NodeHandle 					&root_nh,
	ros::NodeHandle 					&controller_nh),

{
	ROS_INFO_STREAM_NAMED("match_data_controller", "init is running");

	std::vector<std::string> match_names = hw->getNames();
		if (match_names.size() > 1) {
			ROS_ERROR_STREAM("Cannot initialize multiple matches.");
			return false; }
		
		else if (match_names.size() < 1) {
			ROS_ERROR_STREAM("Cannot initialize zero matches.");
			return false; }

	const std::string match_name = match_names[0];

		if (!controller_nh.getParam("publish_rate", publish_rate_))
                	ROS_ERROR("Could not read publish_rate in match data controller");

	realtime_pub_.reset(new realtime_tools::RealtimePublisher<match_data_controller::MatchSpecificData>(root_nh, "match_data", 4));

	auto &m = realtime_pub_->msg_;

	m.matchTimeRemaining = 0.0;
	m.allianceData = "";
	m.getEventName = "";
	m.allianceColor = 0.0;
	m.matchType = 0.0;
	m.driverStationLocation = 0.0;
	m.matchNumber = 0.0;
	m.getReplayNumber = 0.0;
	m.isEnabled = false;
	m.isDisabled = false;
	m.isAutonomous = false:
	m.isFMSAttatched = false;
	m.isOperatorControl = false;
	m.isTest = false;
	m.BatteryVoltage = 0.0;
	
	match_data_ = hw->getHandle(match_name);

	return true;
	
}

void MatchStateController::starting(const ros::Time &time)
{
	last_publish_time_ = time;
}

void matchstatecontroller::update(const ros::Time &time, const ros::Duration & )
{
	//ROS_INFO_STREAM("pdp pub: " << publish_rate_);
	if ((publish_rate_ > 0.0) && (last_publish_time_ + ros::Duration(1.0 / publish_rate_) < time))
	{
		if (realtime_pub_->trylock())
		{
			last_publish_time_ = last_publish_time_ + ros::Duration(1.0 / publish_rate_);
			
			auto &m = realtime_pub_->msg_;

			m.header.stamp = time;

			auto &ps = match_data_;
			
			//read from the object and stuff it in a msg
			m.matchTimeRemaining = ps->getMatchTimeRemaining();
			m.allianceData = ps->getAllianceData();
			m.eventName = ps->getEventName();
			m.allianceColor = ps->getAllianceColor();
			m.matchType = ps->getMatchType();
			m.driverStationLocation = ps->getDriverStationLocation();
			m.matchNumber = ps->getMatchNumber();
			m.replayNumber = ps->getReplayNumber();
			m.Enabled = ps->getEnabled();
			m.Disabled = ps->getDisabled();
			m.Autonomous = ps->getAutonomous();
			m.FMSAttatched = ps->getFMSAttatched();
			m.OperatorControl = ps->getOperatorControl();
			m.Test = ps->getTest();
			m.BatteryVoltage = ps->getBatteryVoltage();
	
			realtime_pub_->unlockAndPublish();
			
		}
	}
}

void MatchStateController::stopping(const ros::Time & ) 
{}
}

PLUGINLIB_EXPORT_CLASS( match_data_controller::MatchStateController, controller_interface::ControllerBase)}
