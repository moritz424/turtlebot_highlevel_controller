#include <ros/ros.h>
#include "turtlebot_mapping/TurtlebotMapping.hpp"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "turtlebot_mapping");
	ros::NodeHandle nodeHandleMap = ros::NodeHandle();

	turtlebot_mapping::TurtlebotMapping turtlebot_mapping(nodeHandleMap);
	
	ros::spin();
	return 0;
}
