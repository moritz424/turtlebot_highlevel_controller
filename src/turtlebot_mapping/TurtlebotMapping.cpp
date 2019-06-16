#include "turtlebot_mapping/TurtlebotMapping.hpp"

// STD
#include <string>
#include <sstream>

typedef actionlib::SimpleActionServer<turtlebot_highlevel_controller::controllerAction> Server;

void execute(const turtlebot_highlevel_controller::controllerGoalConstPtr& goal, Server* as)
{
    ROS_INFO("HELLO ACTION SERVER");
    as->setSucceeded();

}

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

        Server server(nodeHandle_, "controller", boost::bind(&execute, _1, &server), false);
        server.start();

        twistPublisher_= nodeHandle_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop",queueSize_);

        targetSubscriber_ = nodeHandle_.subscribe(subscriberTopic_, queueSize_, 
                            &TurtlebotMapping::topicCallback, this);

        ROS_WARN("Ich bin Node TurtlebotMapping");
        ROS_INFO("Successfully launched mapping node map.");
        
    };

    TurtlebotMapping::~TurtlebotMapping()
    {}

    bool TurtlebotMapping::readParameters()
    {
        if (!(nodeHandle_.getParam("subscriber_topic_map", subscriberTopic_)
            && nodeHandle_.getParam("queue_size_map",queueSize_)
            && nodeHandle_.getParam("kAng",kang_)
            && nodeHandle_.getParam("kVel",kvel_))) return false;

        return true;
    }

    void TurtlebotMapping::topicCallback(const turtlebot_highlevel_controller::Target& target)
    {
        ROS_INFO("Message successfully transmitted %f",target.distance);

        controllerTwistMsg.linear.y, controllerTwistMsg.linear.z = 0, controllerTwistMsg.angular.y,controllerTwistMsg.angular.x = 0;

        // Regler für Bewegung auf Pfeiler
        // Pfeiler gefunden und diesen anfahren
        if ((target.distance < 5)&&(target.distance > 0.4))
        {
            //ROS_INFO("pfeiler");
            controllerTwistMsg.linear.x = kvel_ * target.distance;
            controllerTwistMsg.angular.z = kang_ * target.angle; //kp_ = p Verstaerkungsfaktor
        }
        // Vor dem Pfeiler anhalten und drehen
        else if (target.distance <= 0.4)
        {
            //ROS_INFO("drehen");
            controllerTwistMsg.linear.x = 0;
            controllerTwistMsg.angular.z = 1;
        }
        // Suchmodus - kein Pfeiler in Sicht
        else if (target.distance >= 5)
        {
            //ROS_INFO("suchen");
            controllerTwistMsg.linear.x = 0.65;
            controllerTwistMsg.angular.z = 0.5;
        }
        else
        {
            controllerTwistMsg.linear.x = 0;
            controllerTwistMsg.angular.z = 0;
        }
        //ROS_INFO("x: %f - z: %f",controllerTwistMsg.linear.x, controllerTwistMsg.angular.z);
        twistPublisher_.publish(controllerTwistMsg);

    }


}