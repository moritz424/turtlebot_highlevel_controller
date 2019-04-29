#include <ros/ros.h>
#include <turtlebot_laserscan_range/TurtlebotHighlevelController.hpp>

int main(int argc, char** argv){
	ros::init(argc, argv, "turtlebot_highlevel_controller");
	ros::NodeHandle nodeHandle("~");

	turtlebot_highlevel_controller::TurtlebotHighlevelController turtlebot_highlevel_controller(nodeHandle);
	
	ros::spin();
	return 0;
}
