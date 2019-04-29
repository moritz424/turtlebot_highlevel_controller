#include <turtlebot_laserscan_range/Algorithm.h>

namespace turtlebot_highlevel_controller{

void setData(const std::vector<float> data){
	indexOfMin = std::min_element(data.begin(),data.end()-data.begin());
	values_ = {data[indexOfMin-2] , data[indexOfMin-1] ,data[indexOfMin], data[indexOfMin+1], data[indexOfMin+2]};
}

std::vector<float> getValues(){
	return values_;
}

} /* namespace */