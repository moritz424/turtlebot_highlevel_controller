#include "turtlebot_mapping/TurtlebotMapping.hpp"

// STD
#include <string>
#include <sstream>


namespace turtlebot_mapping
{
    TurtlebotMapping::TurtlebotMapping(ros::NodeHandle& nodeHandle)
        : nodeHandle_(nodeHandle)
    {
       if(!readParameters()) 
        {
            std::string globalName;
            bool x = nodeHandle_.getParam("/global_name", globalName);
            ROS_ERROR("Could not read parameters. %s",globalName.c_str());
            ros::requestShutdown();
        }
        targetSubscriber_ = nodeHandle_.subscribe(subscriberTopic_, queueSize_, 
                            &TurtlebotMapping::topicCallback, this);

        ROS_INFO("Successfully launched mapping node.");
        
    };

    bool TurtlebotMapping::readParameters()
    {
        if (!(nodeHandle_.getParam("subscriber_topic", subscriberTopic_)
            && nodeHandle_.getParam("queue_size",queueSize_))) return false;

        return true;
    }

    void TurtlebotMapping::topicCallback(const turtlebot_highlevel_controller::Target& target)
    {
        ROS_INFO("Message successfully transmitted %f",target.distance);

    }


}