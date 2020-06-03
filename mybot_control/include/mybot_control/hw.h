#ifndef ARM_HW_INTERFACE
#define ARM_HW_INTERFACE

#include <ros/ros.h>
#include <urdf/model.h>
#include <pthread.h>
#include <time.h>
#include <realtime_tools/realtime_buffer.h>

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Vector3.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <controller_manager/controller_manager.h>
#include <controller_manager_msgs/ListControllers.h>

class MyHWInterface : public hardware_interface::RobotHW
{
public:
    MyHWInterface();
 //   void read(const geometry_msgs::Vector3 &msg);
    void write();
 //   ros::NodeHandle getnode();

    ros::Time getTime() const {return ros::Time::now();}
    ros::Duration getPeriod() const {return ros::Duration(0.01);}

private:
    hardware_interface::JointStateInterface jnt_state_interface;
    hardware_interface::PositionJointInterface jnt_pos_interface;
    double cmd[5];
    double pos[5];
    double vel[5];
    double eff[5];



    ros::NodeHandle n;
    ros::Publisher pub_cmd[5];
    std_msgs::Float64 cmd_msg[5];
   // ros::Time start_time_;
   // ros::Duration start_dur_;
	
   // ros::Subscriber sub_js[5];

};

#endif


