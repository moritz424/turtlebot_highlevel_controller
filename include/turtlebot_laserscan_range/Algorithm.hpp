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
  void setData(const std::vector<float>& data);

  /*!
   * Get five range values around the minuimum.
   * @return the five range values of the data.
   */
  std::vector<float> getValues() const;

 private:

  //! Internal variable to hold the current values.
  std::vector<float> values_;

  //! Index of minimum value in the data array.
  unsigned int indexOfMin;
};

} /* namespace */
