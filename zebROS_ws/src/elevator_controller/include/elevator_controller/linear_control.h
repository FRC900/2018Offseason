#pragma once
#include "ros/ros.h"
#include <controller_interface/controller.h>
#include <talon_controllers/talon_controller_interface.h>
#include <pluginlib/class_list_macros.h>
#include "ros/console.h"
#include "std_msgs/Float64.h"
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
//#include <teleop_joystick_control/RobotState.h>
#include <elevator_controller/ElevatorControl.h>
#include <elevator_controller/Intake.h>
#include <elevator_controller/arm_limiting.h>

#include <nav_msgs/Odometry.h>
#include <tf/tfMessage.h>

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>


#include <array>
#include <memory>
#include <Eigen/Dense>


namespace elevator_controller
{

class ElevatorController
        : public controller_interface::Controller<hardware_interface::TalonCommandInterface>
{
        public:
                ElevatorController();
		bool init(hardware_interface::TalonCommandInterface *hw,
			  ros::NodeHandle &root_nh,
			  ros::NodeHandle &controller_nh);

		void update(const ros::Time &time, const ros::Duration &period);
		

		void starting(const ros::Time &time);

	private:
		double lift_position;
		double pivot_angle;

		double lift_velocity;
		double pivot_anglular_velocity;
		
		std::string name_;
		bool if_cube_;
		double clamp_cmd_;

		double max_extension_;
		double min_extension_;
	
		double hook_depth_;
		double hook_min_height_;
		double hook_max_height_;
		
	
		struct IntakeCommand //This struct is highly subject to change
		{			
			double left_command;
			double right_command;
			double spring_left;
			double spring_right;
			double power;
			IntakeCommand() : left_command(0.0),right_command(0.0), spring_left(0.0), spring_right(0.0), power(0.0) {}
	
		};
		//ros::Publisher RobotStatePub;
		//elevator_controller::RobotState RobotStateMsg;

		talon_controllers::TalonMotionMagicCloseLoopControllerInterface lift_joint_;
		talon_controllers::TalonMotionMagicCloseLoopControllerInterface pivot_joint_;
		talon_controllers::TalonPercentOutputControllerInterface intake_joint_;

		struct Commands
                {
                        Eigen::Vector2d lin;
                        bool up_or_down;
                        bool override_pos_limits;
			bool override_sensor_limits;			
			ros::Time stamp;

                        Commands() : lin({0.0, 0.0}), up_or_down(true), stamp(0.0) {}
                };
		realtime_tools::RealtimeBuffer<Commands> command_;
                Commands command_struct_;
		ros::Subscriber sub_command_;
		IntakeCommand intake_struct_;
		ros::Subscriber sub_intake_;
		ros::Subscriber sub_clamp_;
		//TODO: considering adding x offset?
		
		ros::Publisher Clamp; 
		
		ros::Publisher IntakeLeftUp; 
		ros::Publisher IntakeRightUp; 
		ros::Publisher IntakeRightSpring; 
		ros::Publisher IntakeLeftSpring; 

		double arm_length_;
		double pivot_offset_;
		double lift_offset_;
		void cmdPosCallback(const elevator_controller::ElevatorControl& command);
		void intakeCallback(const elevator_controller::Intake& command);
		void clampCallback(const std_msgs::Bool& command); 
		//Add Callback for intake pneumatics, probably needs to be a custom msg
	
		std::shared_ptr<arm_limiting::arm_limits> arm_limiter;
	
		//TODO: add odometry		
		//void compOdometry(const ros::Time& time, const double inv_delta_t);
		void evaluateCubeState();
		//Something for getting the soft limit bounding boxes
		//some function for making limits based on soft limit bounding box


};//Class
PLUGINLIB_EXPORT_CLASS(elevator_controller::ElevatorController, controller_interface::ControllerBase);

}//Namespace
