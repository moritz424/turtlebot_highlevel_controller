#include "turtlebot_mapping/TurtlebotMapping.hpp"

// STD
#include <string>
#include <sstream>



namespace turtlebot_mapping
{

 controllerAction::controllerAction(ros::NodeHandle& nodeHandle)
        : nodeHandle_(nodeHandle),as_(nodeHandle_, "controller", boost::bind(&controllerAction::executeCB, this, _1), false),
    action_name_("controller")
    // setup action callback executeCB
    
  {
    
    if(!readParameters()) 
    {
        ROS_ERROR("Could not read parameters mapping");
        ros::requestShutdown();
    }

    // init publisher and subscriber
    twistPublisher_= nodeHandle_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop",queueSize_);
    targetSubscriber_ = nodeHandle_.subscribe(subscriberTopic_, queueSize_, &controllerAction::topicCallback, this);

    // Start the action server
    as_.start();
    ROS_WARN("controller action launched");
  }

  // Destructor
  controllerAction::~controllerAction(void)
  {
  }

  bool controllerAction::readParameters()
    {
        if (!(nodeHandle_.getParam("subscriber_topic_map", subscriberTopic_)
            && nodeHandle_.getParam("queue_size_map",queueSize_)
            && nodeHandle_.getParam("kAng",kang_)
            && nodeHandle_.getParam("kVel",kvel_))) return false;

        return true;
    }

    // callback of own message with target (= distance and angle to piller)
    void controllerAction::topicCallback(const turtlebot_highlevel_controller::Target& target)
    {
    targetMsg_ = target;
    }

  // Execute action callback (passing the goal via reference)
  void controllerAction::executeCB(const turtlebot_highlevel_controller::controllerGoalConstPtr &goal)
  { 
        ros::Rate r(10); // 10 Hz 
        feedback_.sequence.clear();
        feedback_.sequence.push_back(0);

        ROS_INFO("Message successfully transmitted %f",targetMsg_.distance);

        while(true)
        {
            feedback_.sequence.at(0)+=1;
        
            // check that preempt has not been requested by the client
            if (as_.isPreemptRequested() || !ros::ok())
              {
                ROS_WARN("%s: Preempted", action_name_.c_str());
                // set the action state to preempted
                as_.setPreempted();
                break;
              }

            as_.publishFeedback(feedback_);

            if (feedback_.sequence.at(0)==goal->order)
              {
              result_.sequence = feedback_.sequence;
              ROS_WARN("%s: Succeeded", action_name_.c_str());
              // set the action state to succeeded
              as_.setSucceeded(result_);
              break;
              }

            // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
            r.sleep();

        }
    }
  }
