#include <ros/ros.h>
#include <turtlebot_mapping/TurtlebotMapping.hpp>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "turtlebot_mapping");
	ros::NodeHandle nodeHandle("~");

	turtlebot_mapping::TurtlebotMapping turtlebot_mapping(nodeHandle);
	
	ros::spin();
	return 0;
}
