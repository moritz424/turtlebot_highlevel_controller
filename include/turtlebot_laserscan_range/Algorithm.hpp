#pragma once
#include <ros/ros.h>
#include <vector>
#include <string>
#include <sensor_msgs/LaserScan.h>

//using std::vector;

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
  float findMinLaserScan(const std::vector<float> data);

  /*!
   * Get five range values around the minuimum.
   * @return the five range values of the data.
   */
  std::vector<float> getMsgLaserScan(const std::vector<float> data);

  /*!
   * Get five intensity values around the minuimum.
   * @return the five intensity values of the data.
   */
  std::vector<float> getIntensLaserScan();
  
  /*!
   * Declaration of vectors for minimum (ranges 
   * and intensities) of the laser scan data array
   */
  std::vector<float> min_array;
  std::vector<float> intens_array;
  
 private:

   
  /*! 
   * Declare Variables that contain index and value of 
   * the minimum range in the laser scan data array
   */
  float minValue_;
  float indexOfMin_;

};

} /* namespace */
