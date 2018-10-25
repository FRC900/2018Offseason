i#ifndef INTAKE_CONTROLLER
#define INTAKE_CONTROLLER

#include <ros/ros.h>
#include <vector>
#include <hardware_interface/joint_state_interface.h> //other than talon data
#include <realtime_tools/realtime_publisher.h> //code for real-time buffer - stop multple things writing to same variable at same time
#include <boost/shared_ptr.hpp>
#include <controller_interface/controller.h> //for writing controllers
#include <talon_interface/talon_state_interface.h> // "
#include <talon_controllers/talon_controller.h> // "
#include <talon_controllers/talon_controller_interface.h> // "
#include <intake_controller/IntakeSrv.h> //specific to this controller - write these
#include <atomic>
#include <std_msgs/Float64.h>
#include <pluginlib/class_list_macros.h> //to compile as a controller

namespace intake_controller
{
//this is the actual controller, so it stores all of the  update() functions and the actual handle from the joint interface
//if it was only one type, controller_interface::Controller<TalonCommandInterface> here
class IntakeController : public controller_interface::MultiInterfaceController<hardware_interface::TalonCommandInterface, hardware_interface::JointStateInterface>
{
        public:
                IntakeController()
                {
                }

                //should this be hardware_interface::TalonCommandInterface instead? What's the reason to import RobotHW then get CommandInterface from that instead of just importing TalonCommandIface?
                //answer to my question: the TalonCommandInterface is passed in if it's not a multiInterfaceController, and just one kind of joint is made!
                virtual bool init(hardware_interface::RobotHW *hw,
                                                        ros::NodeHandle                                         &root_nh,
                                                        ros::NodeHandle                                         &controller_nh);
                virtual void starting(const ros::Time &time);
                virtual void update(const ros::Time & time, const ros::Duration& period);
                virtual void stopping(const ros::Time &time);

                virtual bool cmdService(intake_controller::IntakeSrv::Request &req, intake_controller::IntakeSrv::Response &/*res*/);

        private:
        std::vector<std::string> joint_names;
                std::vector<talon_controllers::TalonPercentOutputControllerInterface> joints; //interface for the actual joint
                realtime_tools::RealtimeBuffer<intake_controller::IntakeSrv> command_; //this is the buffer for percent output commands to be published. 
                ros::ServiceServer service_command_;
}; //class

} //namespace
#endif

