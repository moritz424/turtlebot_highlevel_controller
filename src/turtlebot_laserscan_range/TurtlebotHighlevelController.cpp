#include "turtlebot_laserscan_range/TurtlebotHighlevelController.hpp"

// STD
#include <string>
#include <sstream>


namespace turtlebot_highlevel_controller 
{
	TurtlebotHighlevelController::TurtlebotHighlevelController(ros::NodeHandle& nodeHandle)
		: nodeHandle_(nodeHandle), tfListener(tfBuffer)
	{
  		if(!readParameters()) 
  		{
			ROS_ERROR("Could not read parameters.");
			ros::requestShutdown();
  		}
  		//minMarkerMsg.header.frame_id = ros::topic::waitForMessage('odom');
  		
  		scanPublisher_ = nodeHandle_.advertise<sensor_msgs::LaserScan>("scan1",queueSize_);
  		twistPublisher_ = nodeHandle_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop",queueSize_);
  		visPublisher_ = nodeHandle_.advertise<visualization_msgs::Marker>("min_marker", 0);
		
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
		//ROS_INFO("Distance: %f ,  Angle: %f", minDist, minAngle);
		scanPublisher_.publish(minScanMsg);

		//Pfeilerposition aus Robosicht: Point Message
		laserPointStamped.header.frame_id = "base_laser_link";
		laserPointStamped.header.stamp = ros::Time();
		laserPointStamped.point.x = (cos(minAngle)*minDist)+0.1;
		laserPointStamped.point.y = sin(minAngle)*minDist;
		laserPointStamped.point.z = 0;

		controllerTwistMsg.linear.y, controllerTwistMsg.linear.z = 0, controllerTwistMsg.angular.y,controllerTwistMsg.angular.x = 0;

		// Regler für Bewegung auf Pfeiler
		// Pfeiler gefunden und diesen anfahren
		if ((minDist < 5)&&(minDist > 0.4))
		{
			//ROS_INFO("pfeiler");
			controllerTwistMsg.linear.x = kvel_ * minDist;
			controllerTwistMsg.angular.z = kang_ * minAngle; //kp_ = p Verstaerkungsfaktor
		}
		// Vor dem Pfeiler anhalten und drehen
		else if (minDist <= 0.4)
		{
			//ROS_INFO("drehen");
			controllerTwistMsg.linear.x = 0;
			controllerTwistMsg.angular.z = 1;
		}
		// Suchmodus - kein Pfeiler in Sicht
		else if (minDist >= 5)
		{
			//ROS_INFO("suchen");
			controllerTwistMsg.linear.x = 0.65;
			controllerTwistMsg.angular.z = 0.5;
		}
		else
		{
			controllerTwistMsg.linear.x = 0;
			controllerTwistMsg.angular.z = 0;
		}
		//ROS_INFO("x: %f - z: %f",controllerTwistMsg.linear.x, controllerTwistMsg.angular.z);
        twistPublisher_.publish(controllerTwistMsg);

		// TF2 Message (geometry_msgs/TransformStamped) Robo-Odom empfangen
		
		try
		{
			if(ros::ok())
			{
			transformStamped = tfBuffer.lookupTransform("odom", "base_laser_link" , ros::Time(0));
        	}
        }
        catch (tf2::TransformException &ex) 
        {
        	ROS_WARN("%s",ex.what());        	
        }

        // Tranformation
     	tf2::doTransform(laserPointStamped, odomPointStamped, transformStamped);
        
		//Marker1 init: (Marker für die Visualisierung der Säule in RViz)
		minMarkerMsg.header.frame_id = "odom"; //"base_laser_link"; 
		minMarkerMsg.header.stamp = ros::Time();
		minMarkerMsg.ns = "turtlebot_highlevel_controller";
		minMarkerMsg.id = 1;
		minMarkerMsg.type = visualization_msgs::Marker::SPHERE;
		minMarkerMsg.action = visualization_msgs::Marker::ADD;
		minMarkerMsg.pose.orientation.x = 0.0;
		minMarkerMsg.pose.orientation.y = 0.0;
		minMarkerMsg.pose.orientation.z = 0.0;
		minMarkerMsg.pose.orientation.w = 1.0;
		minMarkerMsg.scale.x = 0.1;
		minMarkerMsg.scale.y = 0.1;
		minMarkerMsg.scale.z = 1;
		minMarkerMsg.color.a = 1.0;
		minMarkerMsg.color.r = 1.0;
		minMarkerMsg.color.g = 0.2;
		minMarkerMsg.color.b = 0.0;		
		minMarkerMsg.pose.position.x = odomPointStamped.point.x;
		minMarkerMsg.pose.position.y = odomPointStamped.point.y;
		minMarkerMsg.pose.position.z = odomPointStamped.point.z;
		
		visPublisher_.publish(minMarkerMsg);
		
		//ROS_INFO("x: %f , y: %f,  z: %f", saeule_odom.x, saeule_odom.y, saeule_odom.z);
		//ROS_INFO("tr_x: %f , tr_y: %f,  tr_z: %f", transform_robo_world.getOrigin().x(), 
		//	transform_robo_world.getOrigin().y(), transform_robo_world.getOrigin().z());
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
