#include <turtlebot_laserscan_range/Algorithm.hpp>

namespace turtlebot_highlevel_controller
{

	Algorithm::Algorithm(): 
	indexOfMin_(0), minValue_(5.0){
	}

	Algorithm::~Algorithm(){
	}


	unsigned int Algorithm::findMinIndex(const std::vector<float> data)
	{
		

		for (int i = 0; i < data.size(); ++i)
		{
			if (data[i] < minValue_)
			{			
				indexOfMin_ = i;
			}
		}
		
		return indexOfMin_;
	}

	float Algorithm::findMinValue(const std::vector<float> data)
	{

		for (int i = 0; i < data.size(); ++i)
		{
			if (data[i] < minValue_)
			{			
				minValue_ = data[i];
			}
		}

		return minValue_;
	}


} /* namespace */