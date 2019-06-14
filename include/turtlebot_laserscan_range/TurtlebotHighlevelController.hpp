#pragma once

#include "turtlebot_laserscan_range/Algorithm.hpp"

// ROS
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


namespace turtlebot_highlevel_controller 
{
  /*!
   * Main class for the node to handle the ROS interfacing.
   */

  class TurtlebotHighlevelController
  {
   public:
    /*!
     * Constructor.
     * @param nodeHandle the ROS node handle.
     */
    TurtlebotHighlevelController(ros::NodeHandle& nodeHandle);

    /*!
     * Destructor.
     */
    virtual ~TurtlebotHighlevelController();
      
   
    // Declare buffer and listener
    //tf2_ros::Buffer tfBuffer;
    //tf2_ros::TransformListener tfListener;
    //geometry_msgs::StampedTransform stampTrans;

   //! ROS message type
    geometry_msgs::TransformStamped transformStamped;

    /*
    geometry_msgs::PointStamped laserPointStamped;
    geometry_msgs::PointStamped odomPointStamped;
     // 
  */

    // ROS target message
    turtlebot_highlevel_controller::Target targetMsg;

    // ROS topic publisher
    ros::Publisher scanPublisher_;
    //ros::Publisher twistPublisher_;
    //ros::Publisher visPublisher_; 
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
    float kang_;  // Verstaerkungsfaktor p-Regler
    float kvel_;

    /*!
     * Reads and verifies the ROS parameters.
     * @return true if successful.
     */
    bool readParameters();

    /*!
     * ROS topic callback method.
     * @param message the received message.
     */
    void topicCallback(const sensor_msgs::LaserScan& msg);

    
     // Variables for finding minimum of laser scan data array
     
    int minIndex;
    float minAngle;
    float minDist;
    /*!
    geometry_msgs::Twist controllerTwistMsg;
    visualization_msgs::Marker minMarkerMsg;
    */
     //! ROS service server.
    //ros::ServiceServer serviceServer_;

    //! Algorithm computation object.
    Algorithm algorithm_;

  };

} /* namespace */
