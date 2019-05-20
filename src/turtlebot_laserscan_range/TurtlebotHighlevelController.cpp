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
  		

  		publisher_ = nodeHandle_.advertise<sensor_msgs::LaserScan>("scan1",queueSize_);
  		publisher_twist = nodeHandle_.advertise<geometry_msgs::Twist>("teleop",queueSize_);
  		subscriber_ = nodeHandle_.subscribe(subscriberTopic_, queueSize_, 
  							&TurtlebotHighlevelController::topicCallback, this);
  		vis_pub = nodeHandle_.advertise<visualization_msgs::Marker>("Saeule_marker", 0);


   		ROS_INFO("Successfully launched node.");
	};

	TurtlebotHighlevelController::~TurtlebotHighlevelController()
	{}

	bool TurtlebotHighlevelController::readParameters()
	{
		if (!(nodeHandle_.getParam("subscriber_topic", subscriberTopic_) 
			&& nodeHandle_.getParam("queue_size",queueSize_)

			&& nodeHandle_.getParam("kp",kp_))) return false;

		return true;
	}

	void TurtlebotHighlevelController::topicCallback(const sensor_msgs::LaserScan& msg)
	{

		size = msg.ranges.size();
		min_index = algorithm_.findMinLaserScan(msg.ranges, size);
		min_dist = msg.ranges[min_index];
		min_msg = msg;
		min_msg.angle_min = msg.angle_min+((min_index-2)*msg.angle_increment);
  		min_msg.angle_max = min_msg.angle_min+(5*msg.angle_increment);
		min_msg.ranges = algorithm_.getMsgLaserScan(msg.ranges, size, min_index);
		min_msg.intensities = algorithm_.getIntensLaserScan();

		angle_to_min = msg.angle_min+((min_index)*msg.angle_increment);
		ROS_INFO("Distance: %f ,  Angle: %f", min_dist, angle_to_min);
		publisher_.publish(min_msg);

		//Pfeilerposition aus Robosicht: Point Message
		saeule_robo.header.frame_id = "base_laser_link";
		saeule_robo.header.stamp = ros::Time();
		saeule_robo.point.x = (cos(angle_to_min)*min_dist)+0.1;
		saeule_robo.point.y = sin(angle_to_min)*min_dist;
		saeule_robo.point.z = 0;

		// Init Twist Message
		move_msg.linear.z, move_msg.linear.y = 0;
		move_msg.angular.x, move_msg.angular.y = 0;

		// Regler für Bewegung auf Pfeiler
		// Pfeiler gefunden und diesen anfahren
		if ((min_dist < 5)&&(min_dist > 0.4)){
			move_msg.linear.x = 0.5;
			move_msg.angular.z = (angle_to_min * kp_); //kp_ = p Verstaerkungsfaktor
		}
		// Vor dem Pfeiler anhalten und drehen
		if (min_dist <= 0.4){
			move_msg.linear.x = 0;
			move_msg.angular.z = 1;
		}
		// Suchmodus - kein Pfeiler in Sicht
		if (min_dist == 5){
			move_msg.linear.x = 0.65;
			move_msg.angular.z = 0.5;
		}
		publisher_twist.publish(move_msg);

		// TF2 Message (geometry_msgs/TransformStamped) Robo-Odom empfangen
		//tf2_ros::TransformListener tf_listen_robo_world(tfBuffer);
		try{
		 tf_listen_robo_world.transformPoint("odom", saeule_robo, saeule_odom);
         //transform_robo_world = tfBuffer.lookupTransform("base_laser_link", "odom", ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
         ROS_WARN("%s",ex.what());
         ros::Duration(1.0).sleep();
        }

        // Tranformation
        //tf2::doTransform(saeule_robo, saeule_odom, transform_robo_world);
        
		//Marker1 init: (Marker für die Visualisierung der Säule in RViz)
		marker1.header.frame_id = "odom"; //"base_laser_link"; 
		marker1.header.stamp = ros::Time();
		marker1.ns = "turtlebot_highlevel_controller";
		marker1.id = 1;
		marker1.type = visualization_msgs::Marker::SPHERE;
		marker1.action = visualization_msgs::Marker::ADD;
		marker1.pose.orientation.x = 0.0;
		marker1.pose.orientation.y = 0.0;
		marker1.pose.orientation.z = 0.0;
		marker1.pose.orientation.w = 1.0;
		marker1.scale.x = 0.3;
		marker1.scale.y = 0.3;
		marker1.scale.z = 2;
		marker1.color.a = 1.0;
		marker1.color.r = 0.0;
		marker1.color.g = 0.5;
		marker1.color.b = 0.5;
		// Marker 
		marker1.pose.position.x = saeule_odom.point.x;
		marker1.pose.position.y = saeule_odom.point.y;
		marker1.pose.position.z = saeule_odom.point.z;
		//marker1.pose.position.x = saeule_robo.x;
		//marker1.pose.position.y = saeule_robo.y;
		//marker1.pose.position.z = saeule_robo.z;
		vis_pub.publish(marker1);
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
