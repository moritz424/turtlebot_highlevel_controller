#pragma once

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
  void setData(const float *data);

  /*!
   * Get five range values around the minuimum.
   * @return the five range values of the data.
   */
  float *getValues() const;

 private:

  //! Internal variable to hold the current values.
  float *values_;

  //! Index of minimum value in the data array.
  unsigned int indexOfMin;
};

} /* namespace */
