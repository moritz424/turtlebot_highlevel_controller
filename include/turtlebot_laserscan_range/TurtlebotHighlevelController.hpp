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
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/buffer_core.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/message_filter.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>


namespace turtlebot_highlevel_controller {

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


  tf::TransformListener tfListener;
  
  /*!
   * Matrices for transformation from laser to odom
   */
   // float roundMarkerLaser[3];
   // float roundMarkerOdom[3];
   // float transMatrixLaOd[3][3];

 private:
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

  /*!
   * ROS service server callback.
   * @param request the request of the service.
   * @param response the provided response.
   * @return true if successful, false otherwise.
   */
  //bool serviceCallback(std_srvs::Trigger::Request& request,
  //                     std_srvs::Trigger::Response& response);

  //! ROS node handle.
  ros::NodeHandle& nodeHandle_;

  //! ROS topic subscriber.
  ros::Subscriber scanSubscriber_;

  // ROS topic publisher
  // ros::Publisher laserPublisher_; // currently not neccessary
  ros::Publisher twistPublisher_;
  ros::Publisher markerPublisher_;

  //! ROS topic name to subscribe to.
  std::string subscriberTopic_;

  //! Buffer size of subscriber.
  int queueSize_;

  //! Controller parameters
  float kVel_;
  float kAng_;

  // Variables for minimum of laser scan
  int indexMin;
  float angleMinLaser;
  float distMinLaser;

  // Variables for matrix transformation
  sensor_msgs::LaserScan laserMsg;
  geometry_msgs::Twist twistMsg;
  visualization_msgs::Marker markerMsg;
  
 
  geometry_msgs::PointStamped pointOdom;
  geometry_msgs::PointStamped pointLaser;

  //! ROS service server.
  //ros::ServiceServer serviceServer_;

  //! Algorithm computation object.
  Algorithm algorithm_;
};

} /* namespace */
