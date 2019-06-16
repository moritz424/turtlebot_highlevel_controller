#include <ros/ros.h>
#include "turtlebot_mapping/TurtlebotMapping.hpp"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "turtlebot_mapping_and_server");
	ros::NodeHandle nodeHandleMap = ros::NodeHandle();

	turtlebot_mapping::controllerAction controller_action(nodeHandleMap);
	
	ros::spin();
	return 0;
}
