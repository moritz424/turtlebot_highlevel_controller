// Action Client - 

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <turtlebot_highlevel_controller/controllerAction.h>
#include <termios.h>
#include "std_msgs/Int32.h"


// Called once when the goal completes 
void doneCb(const actionlib::SimpleClientGoalState& state, const turtlebot_highlevel_controller::controllerResultConstPtr &result)
{
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
  ROS_INFO("Answer: %i", result->sequence.back());
  ros::shutdown();
}

// Called once when the goal becomes active
void activeCb()
{
  ROS_INFO("Goal just went active");
}

// Called every time feedback is received for the goal
void feedbackCb(const turtlebot_highlevel_controller::controllerFeedbackConstPtr& feedback)
{
  ROS_INFO("Distance Piller = %i", feedback->sequence.at(0));
}

// receives msg to cancel the action
int cancel_int;
void topicCallback(const std_msgs::Int32::ConstPtr& cancel)
{
  cancel_int = cancel->data;
 
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "controller_client");
  ros::NodeHandle nodeHandle = ros::NodeHandle();
  // Subscriber for cancel-msg
  ros::Subscriber cancelSubscriber = nodeHandle.subscribe("cancel_Goal", 1, topicCallback);
  // action client declaration  
  actionlib::SimpleActionClient<turtlebot_highlevel_controller::controllerAction> ac("controller", true);

  ROS_WARN("Waiting for action server to start.");
  ac.waitForServer(); //will wait for infinite time

  ROS_WARN("Action server started, sending goal.");
  turtlebot_highlevel_controller::controllerGoal goal;
  goal.order = 0.4;             // Goal = distance to piller
  ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb); // setup callbacks
  ROS_WARN("Goal sent");

  ros::Rate r(1);
  
  while(true)
  {
    actionlib::SimpleClientGoalState state = ac.getState();   // get current state of action
    ROS_WARN("Action_Server State: %s",state.toString().c_str());

    // abort the action if cancel-msg received
    if (cancel_int==1)
    {
      ROS_WARN("cancel goal");
      ac.cancelGoal();
      break;
    }

    ros::spinOnce();
    r.sleep();
  }

  // timeout action server
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

    if (finished_before_timeout)
    {
      actionlib::SimpleClientGoalState state = ac.getState();
      ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else
      {ROS_INFO("Action did not finish before the time out.");}

  //exit
  ros::spin();
  return 0;
}
