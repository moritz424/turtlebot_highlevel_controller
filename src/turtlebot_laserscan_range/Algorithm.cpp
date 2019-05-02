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



	if(1 < indexOfMin_ < data.ranges.size()-1)
	{
		msg.ranges = {data.ranges[(indexOfMin_-2)],data.ranges[(indexOfMin_-1)], data.ranges[(indexOfMin_)], data.ranges[(indexOfMin_+1)], data.ranges[(indexOfMin_+2)]};
		msg.intensities = {data.intensities[(indexOfMin_-2)], data.intensities[(indexOfMin_-1)], data.intensities[(indexOfMin_)], data.intensities[(indexOfMin_+1)], data.intensities[(indexOfMin_+2)]};
			
	} else if(indexOfMin_ == 0)
	{
		msg.ranges = {0.0, 0.0, data.ranges[(indexOfMin_)], data.ranges[(indexOfMin_+1)], data.ranges[(indexOfMin_+2)]};
		msg.intensities = {0.0, 0.0, data.intensities[(indexOfMin_)], data.intensities[(indexOfMin_+1)], data.intensities[(indexOfMin_+2)]};
			
	} else if(indexOfMin_ == 1)
	{
		msg.ranges = {0.0, data.ranges[(indexOfMin_-1)], data.ranges[(indexOfMin_)], data.ranges[(indexOfMin_+1)], data.ranges[(indexOfMin_+2)]};
		msg.intensities = {0.0, data.intensities[(indexOfMin_)-1], data.intensities[(indexOfMin_)], data.intensities[(indexOfMin_+1)], data.intensities[(indexOfMin_+2)]};
			
	} else if(indexOfMin_ == data.ranges.size()-1)
	{
			msg.ranges = {data.ranges[(indexOfMin_-2)],data.ranges[(indexOfMin_-1)], data.ranges[(indexOfMin_)], data.ranges[(indexOfMin_+1)], 0.0};
			msg.intensities = {data.intensities[(indexOfMin_)-2], data.intensities[(indexOfMin_)-1], data.intensities[(indexOfMin_)], data.intensities[(indexOfMin_+1)], 0.0};
				
	} else if(indexOfMin_ == data.ranges.size())
	{
			msg.ranges = {data.ranges[(indexOfMin_-2)],data.ranges[(indexOfMin_-1)], data.ranges[(indexOfMin_)], 0.0, 0.0};
			msg.intensities = {data.intensities[(indexOfMin_-2)], data.intensities[(indexOfMin_-1)], data.intensities[(indexOfMin_)], 0.0, 0.0};
			
	}

	msg.angle_min = (data.angle_min + (indexOfMin_-2)*data.angle_increment);
	msg.angle_max = (data.angle_min + (indexOfMin_+2)*data.angle_increment);


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