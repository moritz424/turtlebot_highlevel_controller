#include <turtlebot_laserscan_range/Algorithm.hpp>
#include <string>
#include <sstream>

namespace turtlebot_highlevel_controller{

Algorithm::Algorithm(): 
indexOfMin_(0), minValue_(5.0){
}

Algorithm::~Algorithm(){
}


float Algorithm::findMinLaserScan(const std::vector<float> &data, long unsigned int size){
	
	minValue_ = data[0];
	for (int i = 1; i < size; ++i){
		if (data[i] < minValue_){
			minValue_ = data[i];			
			indexOfMin_ = i;
		}
	}

	return indexOfMin_;
	
}

std::vector<float> Algorithm::getMsgLaserScan(const std::vector<float> &data, long unsigned int size, float min_index){
	min_array.resize(5);
	intens_array.resize(5);
	for(int p=0;p<5;p++){
        if (((min_index+3-(5-p)))<0 || (min_index+3-(5-p))>size){
        min_array[p] = 5;
        intens_array[p] = 0;
        }
        else {
        min_array[p] = data[(min_index+3-(5-p))];
        intens_array[p] = 1;
        }
    }
	return min_array;
}

std::vector<float> Algorithm::getIntensLaserScan(){
	return intens_array;
}



float Algorithm::getMinValueFloat()
{
	return minValue_;
}

} /* namespace */