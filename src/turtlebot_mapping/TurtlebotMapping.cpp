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
            ROS_ERROR("Could not read parameters mapping");
            ros::requestShutdown();
        }
        targetSubscriber_ = nodeHandle_.subscribe(subscriberTopic_, queueSize_, 
                            &TurtlebotMapping::topicCallback, this);

        ROS_INFO("Successfully launched mapping node map.");
        
    };

    TurtlebotMapping::~TurtlebotMapping()
    {}

    bool TurtlebotMapping::readParameters()
    {
        if (!(nodeHandle_.getParam("subscriber_topic_map", subscriberTopic_)
            && nodeHandle_.getParam("queue_size_map",queueSize_))) return false;

        return true;
    }

    void TurtlebotMapping::topicCallback(const turtlebot_highlevel_controller::Target& target)
    {
        ROS_INFO("Message successfully transmitted %f",target.distance);

    }


}