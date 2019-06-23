#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <turtlebot_highlevel_controller/controllerAction.h>
#include <termios.h>
#include "std_msgs/Bool.h"





// Called once when the goal completes 
void doneCb(const actionlib::SimpleClientGoalState& state, const turtlebot_highlevel_controller::controllerResultConstPtr &result)
{
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
  ROS_INFO("Answer: %f", result->sequence.back());
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
  ROS_INFO("Distance Piller = %f", feedback->sequence.at(0));
}


// For non-blocking keyboard inputs
int getch(void)
{
  int ch;
  struct termios oldt;
  struct termios newt;

  // Store old settings, and copy to new settings
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;

  // Make required changes and apply the settings
  newt.c_lflag &= ~(ICANON | ECHO);
  newt.c_iflag |= IGNBRK;
  newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
  newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
  newt.c_cc[VMIN] = 1;
  newt.c_cc[VTIME] = 0;
  tcsetattr(fileno(stdin), TCSANOW, &newt);

  // Get the current character
  ch = getchar();

  // Reapply old settings
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

  return ch;
}

/*bool cancel;
topicCallback(const std_msgs::Bool& cancel_bool)
{
  cancel = cancel_bool.data;
}*/



int main (int argc, char **argv)
{
  // Init ROS node called test_fibonacci
  ros::init(argc, argv, "controller_client");
  ros::NodeHandle nodeHandle = ros::NodeHandle();
  ROS_INFO("Jetzt gehe ich in den client construktor");
  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<turtlebot_highlevel_controller::controllerAction> ac("controller", true);

  ROS_WARN("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_WARN("Action server started, sending goal.");
  // send a goal to the action
  turtlebot_highlevel_controller::controllerGoal goal;
  goal.order = 0.4;
  //ac.sendGoal(goal);
  ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
  ROS_WARN("Goal gesendet");

  //ros::Subscriber cancelSubscriber = nodeHandle.subscribe("cancel_Goal", 10, &topicCallback, this);

  //wait for the action to return
  
  ros::Rate r(1);
  
  while(true)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_WARN("Action_Server State: %s",state.toString().c_str());


    /*char key = getch();

    if(key  == 'a')
    {
      ROS_INFO("cancel");
      ac.cancelGoal();
      break;
    }*/

   r.sleep();
  }

   actionlib::SimpleClientGoalState state = ac.getState();
  ROS_WARN("Action_Server State: %s",state.toString().c_str());
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
