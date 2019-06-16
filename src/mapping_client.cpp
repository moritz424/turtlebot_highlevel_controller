#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <turtlebot_highlevel_controller/controllerAction.h>

int main (int argc, char **argv)
{
  // Init ROS node called test_fibonacci
  ros::init(argc, argv, "controller_client");
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
  goal.order = 10;
  ac.sendGoal(goal);








  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  //exit
  return 0;
}
