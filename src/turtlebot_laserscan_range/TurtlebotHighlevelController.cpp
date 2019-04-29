#include "turtlebot_laserscan_range/TurtlebotHighlevelController.hpp"

// STD
#include <string>

namespace turtlebot_highlevel_controller {

TurtlebotHighlevelController::TurtlebotHighlevelController(ros::NodeHandle& nodeHandle): nodeHandle_(nodeHandle) {
  if(!readParameter()) {
	ROS_ERROR("Could not read parameters.");
	ros::requestShutdown();
  }
  subsrciber_ = nodeHandle_.subscriber(subscriberTopic_, 1, &TurtlebotHighlevelController::topicCallback, this);
  serviceServer_ = nodeHandle_.advertiseService("get_average", &RosPackageTemplate::serviceCallback, this);
  ROS_INFO("Successfully launched node.");
};

TurtlebotHighlevelController::~TurtlebotHighlevelController(){

};

bool TurtlebotHighlevelController::readParameters(){
	if (!nodeHandle_.getParam("subscriber_topic", subscriberTopic_)) return false;
	return true;
	if (!nodeHandle_.getParam)
};

void TurtlebotHighlevelController::topicCallback(const sensor_msgs::LaserScan& msg){
	algorithm_.setData(msg.ranges);
};

bool TurtlebotHighlevelController::serviceCallback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response){
	response.success = true;
	response.message =  std::to_string(algorithm_.getValues());
	return true; 
};


} /* namespace */
