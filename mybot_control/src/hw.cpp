
#include "hw.h"




MyHWInterface::MyHWInterface() 
 { 

   // connect and register the joint state interface
   hardware_interface::JointStateHandle state_handle_0("joint0", &pos[0], &vel[0], &eff[0]);
   jnt_state_interface.registerHandle(state_handle_0);

   hardware_interface::JointStateHandle state_handle_1("joint1", &pos[1], &vel[1], &eff[1]);
   jnt_state_interface.registerHandle(state_handle_1);

   hardware_interface::JointStateHandle state_handle_2("joint2", &pos[2], &vel[2], &eff[2]);
   jnt_state_interface.registerHandle(state_handle_2);

   hardware_interface::JointStateHandle state_handle_3("joint3", &pos[3], &vel[3], &eff[3]);
   jnt_state_interface.registerHandle(state_handle_3);

   hardware_interface::JointStateHandle state_handle_4("joint4", &pos[4], &vel[4], &eff[4]);
   jnt_state_interface.registerHandle(state_handle_4);

   registerInterface(&jnt_state_interface);

   // connect and register the joint position interface
   hardware_interface::JointHandle pos_handle_0(jnt_state_interface.getHandle("joint0"), &cmd[0]);
   jnt_pos_interface.registerHandle(pos_handle_0);

   hardware_interface::JointHandle pos_handle_1(jnt_state_interface.getHandle("joint1"), &cmd[1]);
   jnt_pos_interface.registerHandle(pos_handle_1);

   hardware_interface::JointHandle pos_handle_2(jnt_state_interface.getHandle("joint2"), &cmd[2]);
   jnt_pos_interface.registerHandle(pos_handle_2);

   hardware_interface::JointHandle pos_handle_3(jnt_state_interface.getHandle("joint3"), &cmd[3]);
   jnt_pos_interface.registerHandle(pos_handle_3);

   hardware_interface::JointHandle pos_handle_4(jnt_state_interface.getHandle("joint4"), &cmd[4]);
   jnt_pos_interface.registerHandle(pos_handle_4);

   registerInterface(&jnt_pos_interface);

  for(int i=0; i<5; i++)
  {
        std::string joint_cmd_name="my_joint";
        std::string joint_num=boost::lexical_cast<std::string>(i);
        joint_cmd_name.append(joint_num);
        joint_cmd_name.append("_controller/command");
        pub_cmd[i]=n.advertise<std_msgs::Float64>(joint_cmd_name, 1);
 }
}

void MyHWInterface::write()
{
   for(int i=0; i<4; i++)
	{
		//ROS_INFO("[%f]", cmd[i]);
	    cmd_msg[i].data=cmd[i];
            pub_cmd[i].publish(cmd_msg[i]);
	}
}






int main(int argc, char** argv)
{
	ros::init(argc, argv, "mybot");
	ros::NodeHandle nh;	

	MyHWInterface bot;
	controller_manager::ControllerManager ctrl(&bot, nh);	//class of controller，link to bot

	ros::Rate rate(1.0 / bot.getPeriod().toSec());
	ros::AsyncSpinner spinner(1);
	spinner.start();

	ROS_INFO("HW has been launch!");

	while(ros::ok())
	{
		ctrl.update(bot.getTime(), bot.getPeriod());
		bot.write();	
		rate.sleep();
	}
	spinner.stop();

	return 0;

}
