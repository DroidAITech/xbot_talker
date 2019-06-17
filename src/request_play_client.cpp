#include <cstdlib>
#include "ros/ros.h"
#include "xbot_talker/request_play.h"
#include <string.h>
using namespace std;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "request_talker_client");

	if (argc != 4)
	{
		ROS_INFO("usage: request_play_client choose path text");
		return 1;
	}

	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<xbot_talker::request_play>("request_play");

	xbot_talker::request_play srv;
	//srv.request.request_play = atoll(argv[1]);
        srv.request.choose = atoll(argv[1]);
        srv.request.path = string(argv[2]);
	srv.request.text = string(argv[3]);

	if (client.call(srv))
	{
		ROS_INFO("response: %s", srv.response.feedback.c_str());
	}
	else
	{
		ROS_ERROR("Failed to call service request_play");
		return 1;
	}
	return 0;

}
