#include "turtlebot_mapping/TurtlebotMapping.hpp"

// STD
#include <string>
#include <sstream>



namespace turtlebot_mapping
{

 FibonacciAction::FibonacciAction(std::string name) :
    // Bind the callback to the action server. False is for thread spinning
    as_(nh_, name, boost::bind(&FibonacciAction::executeCB, this, _1), false),
    action_name_(name)
  {
    // Start the action server
    as_.start();
    ROS_WARN("Ich bin im Constructor FibonacciAction");
  }

  // Destructor
  FibonacciAction::~FibonacciAction(void)
  {
  }

  // Execute action callback (passing the goal via reference)
  void FibonacciAction::executeCB(const turtlebot_highlevel_controller::FibonacciGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(1);
    bool success = true;

    // push_back the seeds for the fibonacci sequence
    feedback_.sequence.clear();
    feedback_.sequence.push_back(0);
    feedback_.sequence.push_back(1);

    // publish info to the console for the user
    ROS_INFO("%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i", action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);

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
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_); // ENDE Bedingung
    }
  }

    TurtlebotMapping::TurtlebotMapping(ros::NodeHandle& nodeHandle)
        : nodeHandle_(nodeHandle)   
    {

        if(!readParameters()) 
        {
            ROS_ERROR("Could not read parameters mapping");
            ros::requestShutdown();
        }

        twistPublisher_= nodeHandle_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop",queueSize_);

        targetSubscriber_ = nodeHandle_.subscribe(subscriberTopic_, queueSize_, 
                            &TurtlebotMapping::topicCallback, this);

        ROS_WARN("Ich bin im Constructor TurtlebotMapping");
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