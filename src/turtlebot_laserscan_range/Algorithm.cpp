#include <turtlebot_laserscan_range/Algorithm.hpp>
#include <string>
#include <sstream>

namespace turtlebot_highlevel_controller
{

	Algorithm::Algorithm(): 
	indexOfMin_(0), minValue_(5.0)
	{
	}

	Algorithm::~Algorithm()
	{
	}


	float Algorithm::findMinLaserScan(const std::vector<float> data)
	{
		// find minimal value
		minValue_ = data[0];
		for (int i = 1; i < data.size(); ++i)
		{
			if (data[i] < minValue_)
			{
				minValue_ = data[i];			
				indexOfMin_ = i;
			}
		}

		return indexOfMin_;
		
	}

	std::vector<float> Algorithm::getMsgLaserScan(const std::vector<float> data)
	{
		// find minimal value
		minValue_ = data[0];
		for (int i = 1; i < data.size(); ++i)
		{
			if (data[i] < minValue_)
			{
				minValue_ = data[i];
				indexOfMin_ = i;
			}
		}

		// configure return arrays, fill and return them
		min_array.resize(5);
		intens_array.resize(5);
		for(int p=0;p<5;p++)
		{
	        if ((((indexOfMin_+3-(5-p)))<0) || ((indexOfMin_+3-(5-p))> data.size()))
		    {
		        min_array[p] = 5;
		        intens_array[p] = 0;
			}    
	        else 
		    {
		        min_array[p] = data[(indexOfMin_+3-(5-p))];
		        intens_array[p] = 1;
		    }
	    }
	    //ROS_INFO("Range to min: %f \n", min_array[2]);
		return min_array;
	}

	std::vector<float> Algorithm::getIntensLaserScan()
	{
		return intens_array;
	}



} /* namespace */