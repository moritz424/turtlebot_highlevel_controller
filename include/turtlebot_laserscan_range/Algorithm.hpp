#pragma once

#include <vector>
#include <string>

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
  void setData(const std::vector<float> data);

  /*!
   * Get five range values around the minuimum.
   * @return the five range values of the data.
   */
  std::string getStringValue();

  float getFloatValue();

 private:

  //! Internal variable to hold the current values.
  std::vector<float> rawValues_;

  //! Index of minimum value in the data array.
  unsigned int indexOfMin_;

  std::string valuesString_;
  std::string singleValueString_;

  float minValue_;


};

} /* namespace */
