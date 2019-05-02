#include "turtlebot_laserscan_range/TurtlebotHighlevelController.hpp"

// STD
#include <string>
#include <sstream>

namespace turtlebot_highlevel_controller 
{
	TurtlebotHighlevelController::TurtlebotHighlevelController(ros::NodeHandle& nodeHandle)
		: nodeHandle_(nodeHandle) 
	{
  		if(!readParameters()) 
  		{
			ROS_ERROR("Could not read parameters.");
			ros::requestShutdown();
  		}
  		
  		//serviceServer_ = nodeHandle_.advertiseService("get_value", &TurtlebotHighlevelController::serviceCallback, this);
  		publisher_ = nodeHandle_.advertise<sensor_msgs::LaserScan>("scan1",queueSize_);
  		subscriber_ = nodeHandle_.subscribe(subscriberTopic_, queueSize_, 
  							&TurtlebotHighlevelController::topicCallback, this);
  		

   		ROS_INFO("Successfully launched node.");
	};

	TurtlebotHighlevelController::~TurtlebotHighlevelController()
	{}

	bool TurtlebotHighlevelController::readParameters()
	{
		if (!(nodeHandle_.getParam("subscriber_topic", subscriberTopic_) && nodeHandle_.getParam("queue_size",queueSize_))) return false;
		return true;
	}

	void TurtlebotHighlevelController::topicCallback(const sensor_msgs::LaserScan& msg)
	{
		//algorithm_.setData(msg.ranges);
		ROS_INFO("ranges min value; %f", algorithm_.findMinLaserScan(msg));
		publisher_.publish(algorithm_.getMsgLaserScan());
		//ROS_INFO("Min value msg: %f", msg.range_min);
	}

	bool TurtlebotHighlevelController::serviceCallback(std_srvs::Trigger::Request& request, 
														std_srvs::Trigger::Response& response)
	{
		//std::stringstream ss;
		//ss.str(algorithm_.getStringValue);
		//std::string st_ = ss.str();

		response.success = true;
		response.message =  std::to_string(algorithm_.getMinValueFloat());
		
		return true; 
	}

} /* namespace */
