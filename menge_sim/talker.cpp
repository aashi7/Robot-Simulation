#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

int main(int argc, char **argv)
{

	// third argument of ros::init() is the name of the node - talker 
	// must call one of the versions of ros::init() before using any part of the ROS system 
	ros::init(argc, argv, "talker");

	// NodeHandle is the main access point to communications with ROS system
	// The first NodeHandle constructed will fully initialize this node, and the last NodeHandle destructed will close down the node 
	ros::NodeHandle n;

	// advertise() functon is how you tell ROS that you want to publish on a given topic name 
	// invokes a call to the ROS master node, which keeps a registry of who is publishing and who is subscribing 
	// After this advertise() call is made, the master node will notify anyone who is trying to subscribe to this topic name,
	// and they will in turn negotiate a peer-to-peer connection with this node.
	// advertise() returns a Publisher object which allows you to publish messages on that topic through a call to publish()

	ros::Publisher vel_pub = n.advertise<std_msgs::String>("chatter", 1000);

	ros::Rate loop_rate(10);

	int count = 0;

	while(ros::ok())
	{
		// This is a message object. You stuff it with data, and then publish it.
		std_msgs::String msg;

		std::stringstream ss;
		ss << "hello world " << count;
		msg.data = ss.str();

		ROS_INFO("%s", msg.data.c_str());

		vel_pub.publish(msg);

		ros::spinOnce();

		loop_rate.sleep();
		++count; 
	}

	return 0;
}