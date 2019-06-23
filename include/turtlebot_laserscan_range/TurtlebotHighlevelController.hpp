#pragma once

#include "turtlebot_laserscan_range/Algorithm.hpp"
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_srvs/Trigger.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <math.h>
#include <visualization_msgs/Marker.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <turtlebot_highlevel_controller/Target.h>
#include <turtlebot_highlevel_controller/controllerAction.h> 
#include <actionlib/client/simple_action_client.h>


namespace turtlebot_highlevel_controller 
{
  class TurtlebotHighlevelController
  {
   public:

    TurtlebotHighlevelController(ros::NodeHandle& nodeHandle);

    virtual ~TurtlebotHighlevelController();

   //! ROS message type
    geometry_msgs::TransformStamped transformStamped;

    // ROS target message
    turtlebot_highlevel_controller::Target targetMsg;

    // ROS topic publisher
    ros::Publisher scanPublisher_;
    ros::Publisher targetMsgPublisher_;
   
   private:


     //! ROS node handle.
    ros::NodeHandle& nodeHandle_;

    //! ROS topic subscriber.
    ros::Subscriber scanSubscriber_;

    sensor_msgs::LaserScan minScanMsg;

    //! ROS topic name to subscribe to.
    std::string subscriberTopic_;

    //! Buffer size of subscriber.
    int queueSize_;
    float kang_;  
    float kvel_;

    bool readParameters();

    void topicCallback(const sensor_msgs::LaserScan& msg);

    
     // Variables for finding minimum of laser scan data array
     
    int minIndex;
    float minAngle;
    float minDist;

    //! Algorithm computation object.
    Algorithm algorithm_;

  };

}
