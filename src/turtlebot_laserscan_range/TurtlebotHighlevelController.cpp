#include "turtlebot_laserscan_range/TurtlebotHighlevelController.hpp"

// STD
#include <string>
#include <sstream>

typedef actionlib::SimpleActionClient<turtlebot_highlevel_controller::controllerAction> Client;

namespace turtlebot_highlevel_controller 
{
	TurtlebotHighlevelController::TurtlebotHighlevelController(ros::NodeHandle& nodeHandle)
		: nodeHandle_(nodeHandle)
	{
  		if(!readParameters()) 
  		{
  			ROS_ERROR("Could not read parameters thc.");
			ros::requestShutdown();
  		}
  
  		scanPublisher_ = nodeHandle_.advertise<sensor_msgs::LaserScan>("scan1",queueSize_);	

  		targetMsgPublisher_ = nodeHandle_.advertise<turtlebot_highlevel_controller::Target>("target0",queueSize_);

		scanSubscriber_ = nodeHandle_.subscribe(subscriberTopic_, queueSize_, 
  							&TurtlebotHighlevelController::topicCallback, this);

   		ROS_INFO("Successfully launched node TurtlebotHighlevelController");
	};

	TurtlebotHighlevelController::~TurtlebotHighlevelController()
	{}

	bool TurtlebotHighlevelController::readParameters()
	{
		if (!(nodeHandle_.getParam("subscriber_topic", subscriberTopic_) 
			&& nodeHandle_.getParam("queue_size",queueSize_)
			&& nodeHandle_.getParam("kAng",kang_)
			&& nodeHandle_.getParam("kVel",kvel_))) return false;

		return true;
	}

	void TurtlebotHighlevelController::topicCallback(const sensor_msgs::LaserScan& msg)
	{
		minIndex = algorithm_.findMinLaserScan(msg.ranges);
		minDist = msg.ranges[minIndex];
		minScanMsg = msg;
		minScanMsg.angle_min = msg.angle_min+((minIndex-2)*msg.angle_increment);
  		minScanMsg.angle_max = msg.angle_min+(5*msg.angle_increment);
		minScanMsg.ranges = algorithm_.getMsgLaserScan(msg.ranges);
		minScanMsg.intensities = algorithm_.getIntensLaserScan();

		minAngle = msg.angle_min+((minIndex)*msg.angle_increment);
	
		scanPublisher_.publish(minScanMsg);

		// fill and publish target message for turtlebot_mapping node
		targetMsg.header = msg.header;
		targetMsg.distance = minDist;
		targetMsg.angle = minAngle;
		

		ROS_INFO("Message to transmit: %f", targetMsg.distance);

		targetMsgPublisher_.publish(targetMsg);
	}
} 
