///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2012, hiDOF INC.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of hiDOF Inc nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

/*
 * Original joint_state_controller Author: Wim Meeussen
 */

#include <algorithm>
#include <cstddef>

#include "talon_state_controller/talon_state_controller.h"
#include "talon_state_controller/TalonState.h"

namespace talon_state_controller
{

bool TalonStateController::init(hardware_interface::TalonStateInterface *hw,
								ros::NodeHandle                         &root_nh,
								ros::NodeHandle                         &controller_nh)
{
	// get all joint names from the hardware interface
	const std::vector<std::string> &joint_names = hw->getNames();
	num_hw_joints_ = joint_names.size();
	for (unsigned i = 0; i < num_hw_joints_; i++)
		ROS_DEBUG("Got joint %s", joint_names[i].c_str());

	// get publishing period
	if (!controller_nh.getParam("publish_rate", publish_rate_))
	{
		ROS_ERROR("Parameter 'publish_rate' not set");
		return false;
	}

	// realtime publisher
	realtime_pub_.reset(new
						realtime_tools::RealtimePublisher<talon_state_controller::TalonState>(root_nh, "talon_states",
								4));

	// get joints and allocate message
	for (unsigned i = 0; i < num_hw_joints_; i++)
	{
		realtime_pub_->msg_.name.push_back(joint_names[i]);
		realtime_pub_->msg_.talon_mode.push_back("");
		realtime_pub_->msg_.position.push_back(0.0);
		realtime_pub_->msg_.speed.push_back(0.0);
		realtime_pub_->msg_.output_voltage.push_back(0.0);
		realtime_pub_->msg_.output_current.push_back(0.0);
		realtime_pub_->msg_.bus_voltage.push_back(0.0);
		realtime_pub_->msg_.motor_output_percent.push_back(0.0);
		realtime_pub_->msg_.temperature.push_back(0.0);

		realtime_pub_->msg_.feedback_sensor.push_back("");
		realtime_pub_->msg_.encoder_ticks_per_rotation.push_back(0);

		realtime_pub_->msg_.pid_slot.push_back(0);
		realtime_pub_->msg_.pid_p0.push_back(0.0);
		realtime_pub_->msg_.pid_p1.push_back(0.0);

		realtime_pub_->msg_.pid_i0.push_back(0.0);
		realtime_pub_->msg_.pid_i1.push_back(0.0);

		realtime_pub_->msg_.pid_d0.push_back(0.0);
		realtime_pub_->msg_.pid_d1.push_back(0.0);

		realtime_pub_->msg_.pid_f0.push_back(0.0);
		realtime_pub_->msg_.pid_f1.push_back(0.0);

		realtime_pub_->msg_.pid_izone0.push_back(0);
		realtime_pub_->msg_.pid_izone1.push_back(0);
		realtime_pub_->msg_.pid_allowable_closed_loop_error0.push_back(0);
		realtime_pub_->msg_.pid_allowable_closed_loop_error1.push_back(0);
		realtime_pub_->msg_.pid_max_integral_accumulator0.push_back(0);
		realtime_pub_->msg_.pid_max_integral_accumulator1.push_back(0);
		realtime_pub_->msg_.set_point.push_back(0.0);
		realtime_pub_->msg_.can_id.push_back(0);
		realtime_pub_->msg_.closed_loop_error.push_back(0);
		realtime_pub_->msg_.integral_accumulator.push_back(0);
		realtime_pub_->msg_.error_derivative.push_back(0);
		realtime_pub_->msg_.closed_loop_target.push_back(0);
		realtime_pub_->msg_.active_trajectory_position.push_back(0);
		realtime_pub_->msg_.active_trajectory_velocity.push_back(0);
		realtime_pub_->msg_.active_trajectory_heading.push_back(0);
		realtime_pub_->msg_.forward_limit_switch.push_back(false);
		realtime_pub_->msg_.reverse_limit_switch.push_back(false);
		realtime_pub_->msg_.forward_softlimit.push_back(false);
		realtime_pub_->msg_.reverse_softlimit.push_back(false);
		realtime_pub_->msg_.invert.push_back(false);
		realtime_pub_->msg_.sensorPhase.push_back(false);
		realtime_pub_->msg_.neutral_mode.push_back("");
		realtime_pub_->msg_.neutral_output.push_back(false);

		realtime_pub_->msg_.closed_loop_ramp.push_back(0);
		realtime_pub_->msg_.open_loop_ramp.push_back(0);
		realtime_pub_->msg_.peak_output_forward.push_back(0);
		realtime_pub_->msg_.peak_output_reverse.push_back(0);
		realtime_pub_->msg_.nominal_output_forward.push_back(0);
		realtime_pub_->msg_.nominal_output_reverse.push_back(0);
		realtime_pub_->msg_.neutral_deadband.push_back(0);

		realtime_pub_->msg_.voltage_compensation_saturation.push_back(0);
		realtime_pub_->msg_.voltage_measurement_filter.push_back(0);
		realtime_pub_->msg_.voltage_compensation_enable.push_back(false);

		realtime_pub_->msg_.limit_switch_local_forward_source.push_back("");
		realtime_pub_->msg_.limit_switch_local_forward_normal.push_back("");
		realtime_pub_->msg_.limit_switch_local_reverse_source.push_back("");
		realtime_pub_->msg_.limit_switch_local_reverse_normal.push_back("");
		realtime_pub_->msg_.softlimit_forward_threshold.push_back(0);
		realtime_pub_->msg_.softlimit_forward_enable.push_back(false);
		realtime_pub_->msg_.softlimit_reverse_threshold.push_back(0);
		realtime_pub_->msg_.softlimit_reverse_enable.push_back(false);
		realtime_pub_->msg_.softlimits_override_enable.push_back(false);

		realtime_pub_->msg_.current_limit_peak_amps.push_back(0);
		realtime_pub_->msg_.current_limit_peak_msec.push_back(0);
		realtime_pub_->msg_.current_limit_continuous_amps.push_back(0);
		realtime_pub_->msg_.current_limit_enable.push_back(false);

		realtime_pub_->msg_.motion_cruise_velocity.push_back(0);
		realtime_pub_->msg_.motion_acceleration.push_back(0);
		realtime_pub_->msg_.motion_profile_top_level_buffer_count.push_back(0);
		realtime_pub_->msg_.motion_profile_top_level_buffer_full.push_back(false);
		realtime_pub_->msg_.motion_profile_status_top_buffer_rem.push_back(0);
		realtime_pub_->msg_.motion_profile_status_top_buffer_cnt.push_back(0);
		realtime_pub_->msg_.motion_profile_status_btm_buffer_cnt.push_back(0);
		realtime_pub_->msg_.motion_profile_status_has_underrun.push_back(false);
		realtime_pub_->msg_.motion_profile_status_is_underrun.push_back(false);
		realtime_pub_->msg_.motion_profile_status_active_point_valid.push_back(false);
		realtime_pub_->msg_.motion_profile_status_is_last.push_back(false);
		realtime_pub_->msg_.motion_profile_status_profile_slot_select.push_back(0);
		realtime_pub_->msg_.motion_profile_status_output_enable.push_back("");
		realtime_pub_->msg_.faults.push_back("");
		realtime_pub_->msg_.sticky_faults.push_back("");

		talon_state_.push_back(hw->getHandle(joint_names[i]));
	}
	addExtraJoints(controller_nh, realtime_pub_->msg_);

	return true;
}

void TalonStateController::starting(const ros::Time &time)
{
	// initialize time
	last_publish_time_ = time;
}

std::string TalonStateController::limitSwitchSourceToString(const hardware_interface::LimitSwitchSource source)
{
	switch (source)
	{
	case hardware_interface::LimitSwitchSource_Uninitialized:
		return "Uninitialized";
	case hardware_interface::LimitSwitchSource_FeedbackConnector:
		return "FeedbackConnector";
	case hardware_interface::LimitSwitchSource_RemoteTalonSRX:
		return "RemoteTalonSRX";
	case hardware_interface::LimitSwitchSource_RemoteCANifier:
		return "RemoteCANifier";
		break;
	case hardware_interface::LimitSwitchSource_Deactivated:
		return "Deactivated";
	default:
		return "Unknown";
	}
}
std::string TalonStateController::limitSwitchNormalToString(const hardware_interface::LimitSwitchNormal normal)
{
	switch (normal)
	{
	case hardware_interface::LimitSwitchNormal_Uninitialized:
		return "Uninitialized";
	case hardware_interface::LimitSwitchNormal_NormallyOpen:
		return "NormallyOpen";
	case hardware_interface::LimitSwitchNormal_NormallyClosed:
		return "NormallyClosed";
	case hardware_interface::LimitSwitchNormal_Disabled:
		return "Disabled";
	default:
		return "Unknown";

	}
}

void TalonStateController::update(const ros::Time &time, const ros::Duration & /*period*/)
{
	// limit rate of publishing
	if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0 / publish_rate_) < time)
	{

		// try to publish
		if (realtime_pub_->trylock())
		{
			// we're actually publishing, so increment time
			last_publish_time_ = last_publish_time_ + ros::Duration(1.0 / publish_rate_);

			// populate joint state message:
			// - fill only joints that are present in the JointStateInterface, i.e. indices [0, num_hw_joints_)
			// - leave unchanged extra joints, which have static values, i.e. indices from num_hw_joints_ onwards
			realtime_pub_->msg_.header.stamp = time;
			for (unsigned i = 0; i < num_hw_joints_; i++)
			{
				realtime_pub_->msg_.set_point[i] = talon_state_[i]->getSetpoint();
				realtime_pub_->msg_.position[i] = talon_state_[i]->getPosition();
				realtime_pub_->msg_.speed[i] = talon_state_[i]->getSpeed();
				realtime_pub_->msg_.output_voltage[i] = talon_state_[i]->getOutputVoltage();
				realtime_pub_->msg_.can_id[i] = talon_state_[i]->getCANID();
				realtime_pub_->msg_.output_current[i] = talon_state_[i]->getOutputCurrent();
				realtime_pub_->msg_.bus_voltage[i] = talon_state_[i]->getBusVoltage();
				realtime_pub_->msg_.motor_output_percent[i] = talon_state_[i]->getMotorOutputPercent();
				realtime_pub_->msg_.temperature[i] = talon_state_[i]->getTemperature();

				switch (talon_state_[i]->getEncoderFeedback())
				{
				case hardware_interface::FeedbackDevice_Uninitialized:
					realtime_pub_->msg_.feedback_sensor[i] = "Uninitialized";
					break;
				case hardware_interface::FeedbackDevice_QuadEncoder:
					realtime_pub_->msg_.feedback_sensor[i] = "QuadEncoder";
					break;
				case hardware_interface::FeedbackDevice_Analog:
					realtime_pub_->msg_.feedback_sensor[i] = "Analog";
					break;
				case hardware_interface::FeedbackDevice_Tachometer:
					realtime_pub_->msg_.feedback_sensor[i] = "Tachometer";
					break;
				case hardware_interface::FeedbackDevice_PulseWidthEncodedPosition:
					realtime_pub_->msg_.feedback_sensor[i] = "PusleWidthEncodedPosition";
					break;
				case hardware_interface::FeedbackDevice_SensorSum:
					realtime_pub_->msg_.feedback_sensor[i] =  "SensorSum";
					break;
				case hardware_interface::FeedbackDevice_SensorDifference:
					realtime_pub_->msg_.feedback_sensor[i] = "SensorDifference";
					break;
				case hardware_interface::FeedbackDevice_RemoteSensor0:
					realtime_pub_->msg_.feedback_sensor[i] =  "RemoteSensor0";
					break;
				case hardware_interface::FeedbackDevice_RemoteSensor1:
					realtime_pub_->msg_.feedback_sensor[i] =  "RemoteSensor0";
					break;
				case hardware_interface::FeedbackDevice_SoftwareEmulatedSensor:
					realtime_pub_->msg_.feedback_sensor[i] = "SoftwareEmulatedSensor";
					break;
				default:
					realtime_pub_->msg_.feedback_sensor[i] = "Unknown";
					break;
				}
				realtime_pub_->msg_.encoder_ticks_per_rotation[i] = talon_state_[i]->getEncoderTicksPerRotation();

				//publish the array of PIDF values
				realtime_pub_->msg_.pid_slot[i] = talon_state_[i]->getSlot();
				realtime_pub_->msg_.pid_p0[i] = talon_state_[i]->getPidfP(0);
				realtime_pub_->msg_.pid_i0[i] = talon_state_[i]->getPidfI(0);
				realtime_pub_->msg_.pid_d0[i] = talon_state_[i]->getPidfD(0);
				realtime_pub_->msg_.pid_f0[i] = talon_state_[i]->getPidfF(0);
				realtime_pub_->msg_.pid_izone0[i] = talon_state_[i]->getPidfIzone(0);
				realtime_pub_->msg_.pid_allowable_closed_loop_error0[i] = talon_state_[i]->getAllowableClosedLoopError(0);
				realtime_pub_->msg_.pid_max_integral_accumulator0[i] = talon_state_[i]->getMaxIntegralAccumulator(0);

				realtime_pub_->msg_.pid_p1[i] = talon_state_[i]->getPidfP(1);
				realtime_pub_->msg_.pid_i1[i] = talon_state_[i]->getPidfI(1);
				realtime_pub_->msg_.pid_d1[i] = talon_state_[i]->getPidfD(1);
				realtime_pub_->msg_.pid_f1[i] = talon_state_[i]->getPidfF(1);
				realtime_pub_->msg_.pid_izone1[i] = talon_state_[i]->getPidfIzone(1);
				realtime_pub_->msg_.pid_allowable_closed_loop_error1[i] = talon_state_[i]->getAllowableClosedLoopError(1);
				realtime_pub_->msg_.pid_max_integral_accumulator1[i] = talon_state_[i]->getMaxIntegralAccumulator(1);

				realtime_pub_->msg_.closed_loop_error[i] = talon_state_[i]->getClosedLoopError();
				realtime_pub_->msg_.integral_accumulator[i] = talon_state_[i]->getIntegralAccumulator();
				realtime_pub_->msg_.error_derivative[i] = talon_state_[i]->getErrorDerivative();
				realtime_pub_->msg_.closed_loop_error[i] = talon_state_[i]->getClosedLoopError();
				realtime_pub_->msg_.closed_loop_target[i] = talon_state_[i]->getClosedLoopTarget();
				realtime_pub_->msg_.active_trajectory_position[i] = talon_state_[i]->getActiveTrajectoryPosition();
				realtime_pub_->msg_.active_trajectory_velocity[i] = talon_state_[i]->getActiveTrajectoryVelocity();
				realtime_pub_->msg_.active_trajectory_heading[i] = talon_state_[i]->getActiveTrajectoryHeading();
				realtime_pub_->msg_.forward_limit_switch[i] = talon_state_[i]->getForwardLimitSwitch();
				realtime_pub_->msg_.reverse_limit_switch[i] = talon_state_[i]->getReverseLimitSwitch();
				realtime_pub_->msg_.forward_softlimit[i] = talon_state_[i]->getForwardSoftlimitHit();
				realtime_pub_->msg_.reverse_softlimit[i] = talon_state_[i]->getReverseSoftlimitHit();
				realtime_pub_->msg_.invert[i] = talon_state_[i]->getInvert();
				realtime_pub_->msg_.sensorPhase[i] = talon_state_[i]->getSensorPhase();
				int talonMode = talon_state_[i]->getTalonMode();
				switch (talonMode)
				{
				case hardware_interface::TalonMode_Uninitialized:
					realtime_pub_->msg_.talon_mode[i] = "Uninitialized";
					break;
				case hardware_interface::TalonMode_PercentOutput:
					realtime_pub_->msg_.talon_mode[i] = "Percent Output";
					break;
				case hardware_interface::TalonMode_Position:
					realtime_pub_->msg_.talon_mode[i] = "Closed Loop Position";
					break;
				case hardware_interface::TalonMode_Velocity:
					realtime_pub_->msg_.talon_mode[i] = "Closed Loop Velocity";
					break;
				case hardware_interface::TalonMode_Current:
					realtime_pub_->msg_.talon_mode[i] = "Closed Loop Current";
					break;
				case hardware_interface::TalonMode_Follower:
					realtime_pub_->msg_.talon_mode[i] = "Follower";
					break;
				case hardware_interface::TalonMode_MotionProfile:
					realtime_pub_->msg_.talon_mode[i] = "Motion Profile";
					break;
				case hardware_interface::TalonMode_MotionMagic:
					realtime_pub_->msg_.talon_mode[i] = "Motion Magic";
					break;
				case hardware_interface::TalonMode_TimedPercentOutput:
					realtime_pub_->msg_.talon_mode[i] = "Timed Percent Output";
					break;
				case hardware_interface::TalonMode_Disabled:
					realtime_pub_->msg_.talon_mode[i] = "Disabled";
					break;
				case hardware_interface::TalonMode_Last:
					realtime_pub_->msg_.talon_mode[i] = "Last";
					break;
				default:
					realtime_pub_->msg_.talon_mode[i] = "Unknown";
					break;
				}
				switch (talon_state_[i]->getNeutralMode())
				{
				case hardware_interface::NeutralMode_Uninitialized:
					realtime_pub_->msg_.neutral_mode[i] = "Uninitialized";
					break;
				case hardware_interface::NeutralMode_EEPROM_Setting:
					realtime_pub_->msg_.neutral_mode[i] = "EEPROM_Setting";
					break;
				case hardware_interface::NeutralMode_Coast:
					realtime_pub_->msg_.neutral_mode[i] = "Coast";
					break;
				case hardware_interface::NeutralMode_Brake:
					realtime_pub_->msg_.neutral_mode[i] = "Brake";
					break;
				case hardware_interface::NeutralMode_Last:
					realtime_pub_->msg_.neutral_mode[i] = "Last";
					break;
				default:
					realtime_pub_->msg_.neutral_mode[i] = "Unknown";
					break;
				}
				realtime_pub_->msg_.neutral_output[i] = talon_state_[i]->getNeutralOutput();
				realtime_pub_->msg_.closed_loop_ramp[i] = talon_state_[i]->getClosedloopRamp();
				realtime_pub_->msg_.open_loop_ramp[i] = talon_state_[i]->getOpenloopRamp();
				realtime_pub_->msg_.peak_output_forward[i] = talon_state_[i]->getPeakOutputForward();
				realtime_pub_->msg_.peak_output_reverse[i] = talon_state_[i]->getPeakOutputReverse();
				realtime_pub_->msg_.nominal_output_forward[i] = talon_state_[i]->getNominalOutputForward();
				realtime_pub_->msg_.nominal_output_reverse[i] = talon_state_[i]->getNominalOutputReverse();
				realtime_pub_->msg_.neutral_deadband[i] = talon_state_[i]->getNeutralDeadband();

				realtime_pub_->msg_.voltage_compensation_saturation[i] = talon_state_[i]->getVoltageCompensationSaturation();
				realtime_pub_->msg_.voltage_measurement_filter[i] = talon_state_[i]->getVoltageMeasurementFilter();
				realtime_pub_->msg_.voltage_compensation_enable[i] = talon_state_[i]->getVoltageCompensationEnable();
				hardware_interface::LimitSwitchSource ls_source;
				hardware_interface::LimitSwitchNormal ls_normal;
				talon_state_[i]->getForwardLimitSwitchSource(ls_source, ls_normal);


				realtime_pub_->msg_.limit_switch_local_forward_source[i] = limitSwitchSourceToString(ls_source);
				realtime_pub_->msg_.limit_switch_local_forward_normal[i] = limitSwitchNormalToString(ls_normal);

				talon_state_[i]->getReverseLimitSwitchSource(ls_source, ls_normal);
				realtime_pub_->msg_.limit_switch_local_reverse_source[i] = limitSwitchSourceToString(ls_source);
				realtime_pub_->msg_.limit_switch_local_reverse_normal[i] = limitSwitchNormalToString(ls_normal);

				realtime_pub_->msg_.softlimit_forward_threshold[i] = talon_state_[i]->getForwardSoftLimitThreshold();
				realtime_pub_->msg_.softlimit_forward_enable[i] = talon_state_[i]->getForwardSoftLimitEnable();
				realtime_pub_->msg_.softlimit_reverse_threshold[i] = talon_state_[i]->getReverseSoftLimitThreshold();
				realtime_pub_->msg_.softlimit_reverse_enable[i] = talon_state_[i]->getReverseSoftLimitEnable();
				realtime_pub_->msg_.softlimits_override_enable[i] = talon_state_[i]->getOverrideSoftLimitsEnable();

				realtime_pub_->msg_.current_limit_peak_amps[i] = talon_state_[i]->getPeakCurrentLimit();
				realtime_pub_->msg_.current_limit_peak_msec[i] = talon_state_[i]->getPeakCurrentDuration();
				realtime_pub_->msg_.current_limit_continuous_amps[i] = talon_state_[i]->getContinuousCurrentLimit();
				realtime_pub_->msg_.current_limit_enable[i] = talon_state_[i]->getCurrentLimitEnable();

				realtime_pub_->msg_.motion_cruise_velocity[i] = talon_state_[i]->getMotionCruiseVelocity();
				realtime_pub_->msg_.motion_acceleration[i] = talon_state_[i]->getMotionAcceleration();
				realtime_pub_->msg_.motion_profile_top_level_buffer_count[i] = talon_state_[i]->getMotionProfileTopLevelBufferCount();
				realtime_pub_->msg_.motion_profile_top_level_buffer_full[i] = talon_state_[i]->getMotionProfileTopLevelBufferFull();
				hardware_interface::MotionProfileStatus mp_status(talon_state_[i]->getMotionProfileStatus());
				realtime_pub_->msg_.motion_profile_status_top_buffer_rem[i] = mp_status.topBufferRem;
				realtime_pub_->msg_.motion_profile_status_top_buffer_cnt[i] = mp_status.topBufferCnt;
				realtime_pub_->msg_.motion_profile_status_btm_buffer_cnt[i] = mp_status.btmBufferCnt;
				realtime_pub_->msg_.motion_profile_status_has_underrun[i] = mp_status.hasUnderrun;
				realtime_pub_->msg_.motion_profile_status_is_underrun[i] = mp_status.isUnderrun;
				realtime_pub_->msg_.motion_profile_status_active_point_valid[i] = mp_status.activePointValid;
				realtime_pub_->msg_.motion_profile_status_is_last[i] = mp_status.isLast;
				realtime_pub_->msg_.motion_profile_status_profile_slot_select[i] = mp_status.profileSlotSelect;
				switch (mp_status.outputEnable)
				{
				case hardware_interface::Disable:
					realtime_pub_->msg_.motion_profile_status_output_enable[i] = "Disable";
					break;
				case hardware_interface::Enable:
					realtime_pub_->msg_.motion_profile_status_output_enable[i] = "Enable";
					break;
				case hardware_interface::Hold:
					realtime_pub_->msg_.motion_profile_status_output_enable[i] = "Hold";
					break;
				default:
					realtime_pub_->msg_.motion_profile_status_output_enable[i] = "Unknown";
					break;
				}

				{
					unsigned faults = talon_state_[i]->getFaults();
					unsigned int mask = 1;
					std::string str;
					if (faults)
					{
						if (faults & mask) str += "UnderVoltage "; mask <<= 1;
						if (faults & mask) str += "ForwardLimitSwitch "; mask <<= 1;
						if (faults & mask) str += "ReverseLimitSwitch "; mask <<= 1;
						if (faults & mask) str += "ForwardSoftLimit "; mask <<= 1;
						if (faults & mask) str += "ReverseSoftLimit "; mask <<= 1;
						if (faults & mask) str += "HardwareFailure "; mask <<= 1;
						if (faults & mask) str += "ResetDuringEn "; mask <<= 1;
						if (faults & mask) str += "SensorOverflow "; mask <<= 1;
						if (faults & mask) str += "SensorOutOfPhase "; mask <<= 1;
						if (faults & mask) str += "HardwareESDReset "; mask <<= 1;
						if (faults & mask) str += "RemoteLossOfSignal "; mask <<= 1;
					}
					realtime_pub_->msg_.faults[i] = str;
				}

				{
					unsigned faults = talon_state_[i]->getStickyFaults();
					unsigned int mask = 1;
					std::string str;
					if (faults)
					{
						if (faults & mask) str += "UnderVoltage "; mask <<= 1;
						if (faults & mask) str += "ForwardLimitSwitch "; mask <<= 1;
						if (faults & mask) str += "ReverseLimitSwitch "; mask <<= 1;
						if (faults & mask) str += "ForwardSoftLimit "; mask <<= 1;
						if (faults & mask) str += "ReverseSoftLimit "; mask <<= 1;
						if (faults & mask) str += "ResetDuringEn "; mask <<= 1;
						if (faults & mask) str += "SensorOverflow "; mask <<= 1;
						if (faults & mask) str += "SensorOutOfPhase "; mask <<= 1;
						if (faults & mask) str += "HardwareESDReset "; mask <<= 1;
						if (faults & mask) str += "RemoteLossOfSignal "; mask <<= 1;
					}
					realtime_pub_->msg_.sticky_faults[i] = str;
				}
			}
			realtime_pub_->unlockAndPublish();
		}
	}
}

void TalonStateController::stopping(const ros::Time & /*time*/)
{}

void TalonStateController::addExtraJoints(const ros::NodeHandle &nh,
		talon_state_controller::TalonState &msg)
{

	// Preconditions
	XmlRpc::XmlRpcValue list;
	if (!nh.getParam("extra_joints", list))
	{
		ROS_DEBUG("No extra joints specification found.");
		return;
	}

	if (list.getType() != XmlRpc::XmlRpcValue::TypeArray)
	{
		ROS_ERROR("Extra joints specification is not an array. Ignoring.");
		return;
	}

	for (int i = 0; i < list.size(); ++i)
	{
		if (list[i].getType() != XmlRpc::XmlRpcValue::TypeStruct)
		{
			ROS_ERROR_STREAM("Extra joint specification is not a struct, but rather '" << list[i].getType() <<
							 "'. Ignoring.");
			continue;
		}

		if (!list[i].hasMember("name"))
		{
			ROS_ERROR_STREAM("Extra joint does not specify name. Ignoring.");
			continue;
		}

		const std::string name = list[i]["name"];
		if (std::find(msg.name.begin(), msg.name.end(), name) != msg.name.end())
		{
			ROS_WARN_STREAM("Joint state interface already contains specified extra joint '" << name << "'.");
			continue;
		}

		const bool has_pos = list[i].hasMember("position");
		const bool has_vel = list[i].hasMember("velocity");
		const bool has_eff = list[i].hasMember("effort");

		const XmlRpc::XmlRpcValue::Type typeDouble = XmlRpc::XmlRpcValue::TypeDouble;
		if (has_pos && list[i]["position"].getType() != typeDouble)
		{
			ROS_ERROR_STREAM("Extra joint '" << name << "' does not specify a valid default position. Ignoring.");
			continue;
		}
		if (has_vel && list[i]["velocity"].getType() != typeDouble)
		{
			ROS_ERROR_STREAM("Extra joint '" << name << "' does not specify a valid default velocity. Ignoring.");
			continue;
		}
		if (has_eff && list[i]["effort"].getType() != typeDouble)
		{
			ROS_ERROR_STREAM("Extra joint '" << name << "' does not specify a valid default effort. Ignoring.");
			continue;
		}

		// State of extra joint
		const double pos = has_pos ? static_cast<double>(list[i]["position"]) : 0.0;
		const double vel = has_vel ? static_cast<double>(list[i]["velocity"]) : 0.0;
		const double eff = has_eff ? static_cast<double>(list[i]["effort"])   : 0.0;

		// Add extra joints to message
		msg.position.push_back(pos);
		msg.speed.push_back(vel);
		msg.output_voltage.push_back(eff);
		msg.can_id.push_back(0);
	}
}

}

PLUGINLIB_EXPORT_CLASS( talon_state_controller::TalonStateController, controller_interface::ControllerBase)
