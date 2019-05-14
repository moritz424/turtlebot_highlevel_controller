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

		//Pfeilerposition aus Robosicht: (Ortsvektor)
		r_pfeiler_robo[0] = cos(angle_to_min)*min_dist+0.1; // x
		r_pfeiler_robo[1] = sin(angle_to_min)*min_dist; // y
		r_pfeiler_robo[2] = -0.29; //z
		r_pfeiler_robo[3] = 1; // 4. Element für Transormationsmatrixmulitplikation 1=Ortsvektor

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

		// TF Message Robo-Odom empfangen
		// try{
        // tf_listen_robo_world.lookupTransform("/base_laser_link", "/odom",  
        //                        ros::Time(0), transform_robo_world);
        // }
        // catch (tf::TransformException ex){
        // ROS_ERROR("%s",ex.what());
        // ros::Duration(1.0).sleep();
        // }
        // // Tranformation
        // //Quaternionen auf Rotation um z rechnen:
        // float x,y = 0;
	    // float w = transform_robo_world.getRotation().w();
	    // float z = transform_robo_world.getRotation().z();
		// double siny_cosp = +2.0 * (w * z + x * y);
		// double cosy_cosp = +1.0 - 2.0 * (y * y + z * z);
		// float yaw = atan2(siny_cosp, cosy_cosp);

        // // Transformationsmatrix erstellen
        // T_ol[0][0] = cos(yaw);
        // T_ol[0][1] = -sin(yaw);
        // T_ol[0][2] = 0;
        // T_ol[0][3] = transform_robo_world.getOrigin().x(); // x distanz
        // T_ol[1][0] = sin(yaw);
        // T_ol[1][1] = cos(yaw);
        // T_ol[1][2] = 0;
        // T_ol[1][3] = transform_robo_world.getOrigin().y(); // y distanz
        // T_ol[2][0] = 0;
        // T_ol[2][1] = 0;
        // T_ol[2][2] = 1;
        // T_ol[2][3] = transform_robo_world.getOrigin().z(); // z distanz
        // T_ol[3][0] = 0;
        // T_ol[3][1] = 0;
        // T_ol[3][2] = 0;
        // T_ol[3][3] = 1;

        // //ROS_INFO("x: %f ,  y: %f", r_pfeiler_robo[0], r_pfeiler_robo[1]);
        // // Neuer Ortsvektor im Odom-System berechnen
        // for(int row=0;row<=3;row++){
        // 	r_pfeiler_odom[row]=((T_ol[row][0]*r_pfeiler_robo[0])
        // 							+(T_ol[row][1]*r_pfeiler_robo[1])
        // 							+(T_ol[row][2]*r_pfeiler_robo[2])
        // 							+(T_ol[row][3]*r_pfeiler_robo[3]));
        // }
        
		// //Marker1 init:
		// marker1.header.frame_id = "base_laser_link"; //"odom"
		// marker1.header.stamp = ros::Time();
		// marker1.ns = "turtlebot_highlevel_controller";
		// marker1.id = 1;
		// marker1.type = visualization_msgs::Marker::SPHERE;
		// marker1.action = visualization_msgs::Marker::ADD;
		// marker1.pose.orientation.x = 0.0;
		// marker1.pose.orientation.y = 0.0;
		// marker1.pose.orientation.z = 0.0;
		// marker1.pose.orientation.w = 1.0;
		// marker1.scale.x = 0.3;
		// marker1.scale.y = 0.3;
		// marker1.scale.z = 2;
		// marker1.color.a = 1.0;
		// marker1.color.r = 0.0;
		// marker1.color.g = 0.5;
		// marker1.color.b = 0.5;
		// // Marker 
		// marker1.pose.position.x = r_pfeiler_robo[0];
		// marker1.pose.position.y = r_pfeiler_robo[1];
		// marker1.pose.position.z = r_pfeiler_robo[2];
		// //marker1.pose.position.x = r_pfeiler_odom[0];
		// //marker1.pose.position.y = r_pfeiler_odom[1];
		// //marker1.pose.position.z = r_pfeiler_odom[2];
		// vis_pub.publish(marker1);

		laser_point.header.frame_id = "base_laser_link";
		laser_point.header.stamp = ros::Time();
		laser_point.point.x = r_pfeiler_robo[0];
		laser_point.point.y = r_pfeiler_robo[1];
		laser_point.point.z = r_pfeiler_robo[2];

		try{
			tf_listen_robo_world.transformPoint("odom", laser_point,odom_point);
		}

		catch(tf::TransformException& ex)
		{
			ROS_ERROR("Received an exception trying to transform a point from \"base_laser_link\" to \"odom\": %s", ex.what());
		}

		//Marker1 init:
		marker1.header.frame_id = "odom"; //"odom"
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
		marker1.pose.position.x = odom_point.point.x;
		marker1.pose.position.y = odom_point.point.y;
		marker1.pose.position.z = odom_point.point.z;

		vis_pub.publish(marker1);


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
