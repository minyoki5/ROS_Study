#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <mid_project/Action1Action.h>



int main(int argc, char **argv)
{
  ros::init(argc, argv, "Leader_Vehicle");
  ros::NodeHandle nh;

  actionlib::SimpleActionClient<mid_project::Action1Action> ac("V2X",true);
  ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();

    ROS_INFO("Action server started, sending goal.");
    mid_project::Action1Goal goal;
    goal.order=true;
    ac.sendGoal(goal);

  bool finished_before_timeout=ac.waitForResult(ros::Duration(30.0));

    if(finished_before_timeout){
      actionlib::SimpleClientGoalState state=ac.getState();
      ROS_INFO("Action finished: %s", state.toString().c_str());
    }else
      ROS_INFO("Action did not finish before the time out.");


    ros::spin();

  return 0;
}
