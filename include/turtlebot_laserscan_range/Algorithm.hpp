#pragma once
#include <ros/ros.h>
#include <vector>
#include <string>
#include <sensor_msgs/LaserScan.h>

//using std::vector;

namespace turtlebot_highlevel_controller {

class Algorithm
{
 public:

  Algorithm();

  virtual ~Algorithm();

  float findMinLaserScan(const std::vector<float> data);

  std::vector<float> getMsgLaserScan(const std::vector<float> data);

  std::vector<float> getIntensLaserScan();
  
  std::vector<float> min_array;
  std::vector<float> intens_array;
  
 private:

  float minValue_;
  float indexOfMin_;

};

} 
