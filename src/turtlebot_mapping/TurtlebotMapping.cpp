#include "turtlebot_mapping/TurtlebotMapping.hpp"

// STD
#include <string>
#include <sstream>



namespace turtlebot_mapping
{

 controllerAction::controllerAction(ros::NodeHandle& nodeHandle)
        : nodeHandle_(nodeHandle),as_(nodeHandle_, "controller", boost::bind(&controllerAction::executeCB, this, _1), false),
    action_name_("controller")
    // Bind the callback to the action server. False is for thread spinning
    
  {
    

    if(!readParameters()) 
    {
        ROS_ERROR("Could not read parameters mapping");
        ros::requestShutdown();
    }

    twistPublisher_= nodeHandle_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop",queueSize_);
    targetSubscriber_ = nodeHandle_.subscribe(subscriberTopic_, queueSize_, &controllerAction::topicCallback, this);

    // Start the action server
    as_.start();
    ROS_WARN("Ich bin im Constructor controllerAction");
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

 void controllerAction::topicCallback(const turtlebot_highlevel_controller::Target& target)
 {
    targetMsg_ = target;
 }

  // Execute action callback (passing the goal via reference)
  void controllerAction::executeCB(const turtlebot_highlevel_controller::controllerGoalConstPtr &goal)
  { 
        ros::Rate r(10); // 10 Hz schleifengeschwindigkeit
        bool success = false;   // ziel nicht erreicht
        feedback_.sequence.clear();
        feedback_.sequence.push_back(0);

        ROS_INFO("Message successfully transmitted %f",targetMsg_.distance);

        //for(int i=1; i>=goal->order; i++)
        while(true)
    {feedback_.sequence.at(0)+=1;
        
        
        // check that preempt has not been requested by the client !!!!!!!!!ABBRUCHHH!!!!!!!!!!!!!!!!
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_WARN("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
        break;
      }

    as_.publishFeedback(feedback_);

          if (feedback_.sequence.at(0)==goal->order)
      {
        success = true;
      result_.sequence = feedback_.sequence;
      ROS_WARN("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_); // ENDE Bedingung
      break;
    }

     // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
      r.sleep();

    }
}
    /*// helper variables
    ros::Rate r(1);
    bool success = true;

    // push_back the seeds for the fibonacci sequence
    feedback_.sequence.clear();
    feedback_.sequence.push_back(0);
    feedback_.sequence.push_back(1);

    // publish info to the console for the user
    ROS_INFO("%s: Executing, creating controller sequence of order %i with seeds %i, %i", action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);

    // start executing the action (i <= goal->order, as goal is a pointer)
    for(int i=1; i<=goal->order; i++)
    {
      // check that preempt has not been requested by the client !!!!!!!!!ABBRUCHHH!!!!!!!!!!!!!!!!
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
        break;
      }

      // Add the number to the feedback to be fed back
      feedback_.sequence.push_back(feedback_.sequence[i] + feedback_.sequence[i-1]);
      // publish the feedback
      as_.publishFeedback(feedback_);
      // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
      r.sleep();
    }

    if(success)
    {
      result_.sequence = feedback_.sequence;
      ROS_WARN("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_); // ENDE Bedingung
    }*/
  }
