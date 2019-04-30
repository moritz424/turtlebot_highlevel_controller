#include <turtlebot_laserscan_range/Algorithm.hpp>

namespace turtlebot_highlevel_controller{

Algorithm::Algorithm(): indexOfMin_(0), values_({0,0,0,0,0}), minValue_(5.0){
}

Algorithm::~Algorithm(){
}


void Algorithm::setData(const std::vector<float> data){

	for (int i = 0; i < data.size(); ++i){
		if (data[i] < minValue_){			
			indexOfMin_ = i;
		}
	}

	values_ = {data[indexOfMin_-2] , data[indexOfMin_-1] ,data[indexOfMin_], data[indexOfMin_+1], data[indexOfMin_+2]};
}

std::string Algorithm::getStringValues(){
	
	//std::string s  = (" %2.5f || %2.5f || %2.5f || %2.5f || %2.5f ",values_[0], values_[1], values_[2], values_[3], values_[4]);
	return "Hello";	
}

} /* namespace */