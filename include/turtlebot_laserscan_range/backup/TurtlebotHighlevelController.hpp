#pragma once

#include "turtlebot_laserscan_range/Algorithm.hpp"

// ROS
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_srvs/Trigger.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>

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
  float r_pfeiler_robo[3];
  float r_pfeiler_odom[3];
  float T_ol[3][3]; // Transformationsmatrix odom-laser  

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
  long unsigned int size;
  int min_index;
  float angle_to_min;
  float min_dist;
  sensor_msgs::LaserScan min_msg;
  geometry_msgs::Twist move_msg;
  visualization_msgs::Marker marker1;
  tf::TransformListener tf_listen_robo_world;
  tf::StampedTransform transform_robo_world;

   /*!
   * ROS service server callback.
   * @param request the request of the service.
   * @param response the provided response.
   * @return true if successful, false otherwise.
   */
  bool serviceCallback(std_srvs::Trigger::Request& request,
                       std_srvs::Trigger::Response& response);

  //! ROS node handle.
  ros::NodeHandle& nodeHandle_;

  //! ROS topic subscriber.
  ros::Subscriber subscriber_;

  // ROS topic publisher
  ros::Publisher publisher_;
  ros::Publisher publisher_twist;
  ros::Publisher vis_pub; 

  //! ROS topic name to subscribe to.
  std::string subscriberTopic_;

  //! Buffer size of subscriber.
  int queueSize_;
  float kp_;  // Verstaerkungsfaktor p-Regler

  //! ROS service server.
  ros::ServiceServer serviceServer_;

  //! Algorithm computation object.
  Algorithm algorithm_;

};

} /* namespace */
