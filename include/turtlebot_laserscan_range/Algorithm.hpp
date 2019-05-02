#pragma once

#include <vector>
#include <string>
#include <sensor_msgs/LaserScan.h>

namespace turtlebot_highlevel_controller {

/*!
 * Class containing the algorithmic part of the package.
 */
class Algorithm
{
 public:
  /*!
   * Constructor.
   */
  Algorithm();

  /*!
   * Destructor.
   */
  virtual ~Algorithm();

  /*!
   * Set new measurement data.
   * @param data the new data.
   */
  float findMinLaserScan(const sensor_msgs::LaserScan data);

  /*!
   * Get five range values around the minuimum.
   * @return the five range values of the data.
   */
  sensor_msgs::LaserScan getMsgLaserScan();

  float getMinValueFloat();

 private:

  //! Internal variable to hold the current values.
  
  //! Index of minimum value in the data array.
  unsigned int indexOfMin_;
 
  float minValue_;

  sensor_msgs::LaserScan msg;



};

} /* namespace */
