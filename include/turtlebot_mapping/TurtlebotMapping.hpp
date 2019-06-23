#pragma once
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
#include <actionlib/server/simple_action_server.h>
#include <turtlebot_highlevel_controller/controllerAction.h>


namespace turtlebot_mapping 
{


class controllerAction
{
protected:

 ros::NodeHandle& nodeHandle_;
 ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  actionlib::SimpleActionServer<turtlebot_highlevel_controller::controllerAction> as_;
  std::string action_name_;

  // create messages that are used to published feedback/result
  turtlebot_highlevel_controller::controllerFeedback feedback_;
  turtlebot_highlevel_controller::controllerResult result_;

      //! ROS message type
    turtlebot_highlevel_controller::Target targetMsg_;


    //! ROS topic subscriber.
    ros::Subscriber targetSubscriber_;

    //! ROS topic publisher
    ros::Publisher twistPublisher_;
    

    //! ROS topic name to subscribe to.
    std::string subscriberTopic_;

    //! Buffer size of subscriber.
    
    int queueSize_;
    float kang_;  
    float kvel_;
    geometry_msgs::Twist controllerTwistMsg;

    int minIndex;
    float minAngle;
    float minDist;

public:

  
  controllerAction(ros::NodeHandle& nodeHandle);

  // Destructor
  ~controllerAction(void);

  bool readParameters();

  void topicCallback(const turtlebot_highlevel_controller::Target& target);

  // Execute action callback (passing the goal via reference)
  void executeCB(const turtlebot_highlevel_controller::controllerGoalConstPtr &goal);
  
};
 

} /* namespace */
