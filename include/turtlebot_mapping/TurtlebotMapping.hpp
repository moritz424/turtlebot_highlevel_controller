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
#include <turtlebot_highlevel_controller/FibonacciAction.h>


namespace turtlebot_mapping 
{


class FibonacciAction
{
protected:

  ros::NodeHandle nh_;

  // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  actionlib::SimpleActionServer<turtlebot_highlevel_controller::FibonacciAction> as_;
  std::string action_name_;

  // create messages that are used to published feedback/result
  turtlebot_highlevel_controller::FibonacciFeedback feedback_;
  turtlebot_highlevel_controller::FibonacciResult result_;

public:

  // Our constructor (with a cool initialisation list!)
  // https://stackoverflow.com/questions/2785612/c-what-does-the-colon-after-a-constructor-mean
  FibonacciAction(std::string name);

  // Destructor
  ~FibonacciAction(void);


  // Execute action callback (passing the goal via reference)
  void executeCB(const turtlebot_highlevel_controller::FibonacciGoalConstPtr &goal);
  
};
  /*!
   * Main class for the node to handle the ROS interfacing.
   */
  class TurtlebotMapping
  {
   public:
    /*!
     * Constructor.
     * @param nodeHandle the ROS node handle.
     */
    TurtlebotMapping(ros::NodeHandle& nodeHandle);

    /*!
     * Destructor.
     */
    virtual ~TurtlebotMapping();
      
   
    // Declare buffer and listener
    // tf2_ros::Buffer tfBuffer;
    //tf2_ros::TransformListener tfListener;
    
    
  protected:


    //! ROS node handle.
    ros::NodeHandle& nodeHandle_;

    //! Action server 
    //actionlib::SimpleActionServer<turtlebot_highlevel_controller::controllerAction> as_; 
    //std::string action_name_;
    //turtlebot_highlevel_controller::controllerFeedback feedback_;
    //turtlebot_highlevel_controller::controllerResult result_;

  public:

    //! ROS message type
    turtlebot_highlevel_controller::Target targetMsg;


    //! ROS topic subscriber.
    ros::Subscriber targetSubscriber_;

    //! ROS topic publisher
    ros::Publisher twistPublisher_;
    

    //! ROS topic name to subscribe to.
    std::string subscriberTopic_;

    //! Buffer size of subscriber.
    
    int queueSize_;
    float kang_;  // Verstaerkungsfaktor p-Regler
    float kvel_;
    geometry_msgs::Twist controllerTwistMsg;

    void topicCallback(const turtlebot_highlevel_controller::Target& target);

    /*!
     * Reads and verifies the ROS parameters.
     * @return true if successful.
 */
    bool readParameters();

    /*!
     * Variables for finding minimum of laser scan data array
     */
    int minIndex;
    float minAngle;
    float minDist;

  };

} /* namespace */
