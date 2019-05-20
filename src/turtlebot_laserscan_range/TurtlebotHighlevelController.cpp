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
  		
  		/*
  		 * Generate publishers
  		 */
  		//laserPublisher_ 	= nodeHandle_.advertise<sensor_msgs::LaserScan>("mod_scan",queueSize_);
  		twistPublisher_ 	= nodeHandle_.advertise<geometry_msgs::Twist>("teleop",queueSize_);
  		markerPublisher_ 	= nodeHandle_.advertise<visualization_msgs::Marker>("min_marker",queueSize_);

  		/*
  		 * Generate subsribers
  		 */
  		scanSubscriber_ = nodeHandle_.subscribe(subscriberTopic_, queueSize_, 
  											&TurtlebotHighlevelController::topicCallback, this);
  		

   		ROS_INFO("Successfully launched node.");
	};

	TurtlebotHighlevelController::~TurtlebotHighlevelController()
	{}

	bool TurtlebotHighlevelController::readParameters()
	{
		if (!(nodeHandle_.getParam("subscriber_topic", subscriberTopic_) 
			&& nodeHandle_.getParam("queue_size",queueSize_)
			&& nodeHandle_.getParam("k_vel",kVel_)
			&& nodeHandle_.getParam("k_ang",kAng_))) return false;
		return true;
	}

	void TurtlebotHighlevelController::topicCallback(const sensor_msgs::LaserScan& msg)
	{
		//algorithm_.setData(msg.ranges);
		indexMin = algorithm_.findMinIndex(msg.ranges); 
		angleMinLaser = msg.angle_min + indexMin * msg.angle_increment;
		distMinLaser = msg.ranges[indexMin];
		ROS_INFO("Minimal distance: %f",distMinLaser);
		// Marker position in laser coordinates
		pointLaser.header.frame_id = "base_laser_link";
		pointLaser.header.stamp = ros::Time();
		pointLaser.point.x = cos(angleMinLaser) * distMinLaser;
		pointLaser.point.y = sin(angleMinLaser) * distMinLaser;
		pointLaser.point.z = 0; 

		// Init twist message
		twistMsg.linear.y, twistMsg.linear.z = 0;
		twistMsg.angular.x, twistMsg.angular.y = 0; 

		// Controller
			if(distMinLaser == 5)
		{
			//searching for pillar
			twistMsg.linear.x = 0.3;
			twistMsg.angular.z = 0.2;
			ROS_INFO("Searching...");
		}

		if((distMinLaser < 5) && (distMinLaser > 0.3))
		{
			//pillar found -> controller active
			twistMsg.linear.x = kVel_ * distMinLaser;
			twistMsg.angular.z = kAng_ * angleMinLaser;
			ROS_INFO("Controlling...");
		}

		if(distMinLaser <= 0.3)
		{
			//robot at pillar, stop & turn around
			twistMsg.linear.x = 0;
			twistMsg.angular.z = 0.2;
			ROS_INFO("Turning...");
		}
		// publish controller output
		twistPublisher_.publish(twistMsg);

/*

		// Transform point of pillar form laser coordinates to world coordinates
		try
		{
			tfListener.transformPoint("odom",pointLaser,pointOdom);
		}
		catch(tf::TransformException &ex)
		{
			ROS_ERROR("%s", ex.what());
		}
	
		// Init marker message
		markerMsg.header.frame_id = "odom";
		markerMsg.header.stamp = ros::Time();
		markerMsg.ns = "turtlebot_highlevel_controller";
		markerMsg.id = 1;
		markerMsg.type = visualization_msgs::Marker::SPHERE;
		markerMsg.action = visualization_msgs::Marker::ADD;
		markerMsg.pose.orientation.x = 0.0;
		markerMsg.pose.orientation.y = 0.0;
		markerMsg.pose.orientation.z = 0.0;
		markerMsg.pose.orientation.w = 1.0;
		markerMsg.scale.x = 0.1;
		markerMsg.scale.y = 0.1;
		markerMsg.scale.z = 0.1;
		markerMsg.color.a = 1.0;
		markerMsg.color.r = 1.0;
		markerMsg.color.g = 0.0;
		markerMsg.color.b = 0.0;
		markerMsg.pose.position.x = pointOdom.point.x;
		markerMsg.pose.position.y = pointOdom.point.y;
		markerMsg.pose.position.z = pointOdom.point.z;

		markerPublisher_.publish(markerMsg);
*/
	}

	//bool TurtlebotHighlevelController::serviceCallback(std_srvs::Trigger::Request& request, 
	//   												std_srvs::Trigger::Response& response)
	//{
	//	std::stringstream ss;
	//	ss.str(algorithm_.getStringValue);
	//	std::string st_ = ss.str();

	//	response.success = true;
	//	response.message =  std::to_string(algorithm_.getMinValueFloat());
		
	//	return true; 
	//}

} /* namespace */
