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
  float findMinLaserScan(const std::vector<float> &data, long unsigned int size);

  /*!
   * Get five range values around the minuimum.
   * @return the five range values of the data.
   */
  std::vector<float> getMsgLaserScan(const std::vector<float> &data, long unsigned int size, float min_index);
  std::vector<float> min_array;
  std::vector<float> intens_array;
  std::vector<float> getIntensLaserScan();
  float getMinValueFloat();


 private:

   
  //! Index of minimum value in the data array.
  float minValue_;


  




};

} /* namespace */
