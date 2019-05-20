#pragma once

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
   *
   */
  
  float findMinValue(const std::vector<float> data);
  unsigned int findMinIndex(const std::vector<float> data);

  

 private:

   
  //! Index of minimum value in the data array.
  float minValue_;
  unsigned int indexOfMin_;
  
};

} /* namespace */
