#include <cstdlib>
#include "ros/ros.h"
#include "xbot_talker/request_talker.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "request_talker_client");

	if (argc != 2)
	{
		ROS_INFO("usage: request_talker_client request");
		return 1;
	}

	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<xbot_talker::request_talker>("request_talker");

	xbot_talker::request_talker srv;
	srv.request.request_talker = atoll(argv[1]);

	if (client.call(srv))
	{
		ROS_INFO("response: %ld", (int)srv.response.response);
	}
	else
	{
		ROS_ERROR("Failed to call service request_talker");
		return 1;
	}
	return 0;

}