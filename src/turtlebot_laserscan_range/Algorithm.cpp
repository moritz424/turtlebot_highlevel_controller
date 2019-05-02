#include <turtlebot_laserscan_range/Algorithm.hpp>

namespace turtlebot_highlevel_controller{

Algorithm::Algorithm(): 
indexOfMin_(0), minValue_(5.0){
}

Algorithm::~Algorithm(){
}


float Algorithm::findMinLaserScan(const sensor_msgs::LaserScan data){
	
	for (int i = 0; i < data.ranges.size(); ++i){
		if (data.ranges[i] < minValue_){			
			indexOfMin_ = i;
		}
	}
	msg = data;
	msg.ranges = {data.ranges[indexOfMin_-2], data.ranges[indexOfMin_-1], data.ranges[indexOfMin_], data.ranges[indexOfMin_+1], data.ranges[indexOfMin_+2]};
	
	minValue_ = data.ranges[indexOfMin_];


	return minValue_;
	//singleValueString_ = std::to_string(rawValues_[2]);
	//valuesString_ = std::to_string(rawValues_[0]) + " || " + std::to_string(rawValues_[1]) + " || " + std::to_string(rawValues_[2]) + " || " + std::to_string(rawValues_[3]) + " || " + std::to_string(rawValues_[4]); 
}

sensor_msgs::LaserScan Algorithm::getMsgLaserScan()
{
	return msg;	
}

float Algorithm::getMinValueFloat()
{
	return minValue_;
}

} /* namespace */