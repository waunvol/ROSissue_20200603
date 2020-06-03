#include "ros/ros.h"
#include "std_msgs/Float64.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "talker");
	ros::NodeHandle n;

	ros::Publisher chatter_pub = n.advertise<std_msgs::Float64>("/mybot/joint0_position_controller/command", 1000);
	ros::Rate loop_rate(5);

	float k = 0;
	int flag = 0;
	

	while(ros::ok())
	{

		ROS_INFO("begin");
		std_msgs::Float64 msg;



		if(flag == 0)
		{
			ROS_INFO("here flag = 0");
			k = k + 0.1;
			if(k >= 3)
			{
				flag = 1;
			}
		}
		if(flag ==1)
		{
			ROS_INFO("here falg = 1");
			k = k - 0.1;
			if(k <= -3)
			{
				flag = 0;
			}
		}
		msg.data = k;
		ROS_INFO("now k = %f", msg.data);
		chatter_pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}
