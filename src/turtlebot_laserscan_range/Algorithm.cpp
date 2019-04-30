#include <turtlebot_laserscan_range/Algorithm.hpp>

namespace turtlebot_highlevel_controller{

Algorithm::Algorithm(): indexOfMin_(0), rawValues_({0.0,0.0,0.0,0.0,0.0}), minValue_(5.0), valuesString_(""){
}

Algorithm::~Algorithm(){
}


void Algorithm::setData(const std::vector<float> data){
	for (int i = 0; i < data.size(); ++i){
		if (data[i] < minValue_){			
			indexOfMin_ = i;
		}
	}
	rawValues_ = {data[indexOfMin_-2] , data[indexOfMin_-1] ,data[indexOfMin_], data[indexOfMin_+1], data[indexOfMin_+2]};
	valuesString_ = std::to_string(rawValues_[0]) + " || " + std::to_string(rawValues_[1]) + " || " + std::to_string(rawValues_[2]) + " || " + std::to_string(rawValues_[3]) + " || " + std::to_string(rawValues_[4]); 
}

std::string Algorithm::getStringValues(){
	return valuesString_;	
}

} /* namespace */