#include "ros/ros.h"
#include "mid_project/rear_judge.h"
#include "mid_project/Action1Action.h"
#include "mid_project/emergency.h"
#include "mid_project/waypoint.h"
#include <actionlib/server/simple_action_server.h>
#include <iostream>


class V2X_Action{
protected:
  ros::NodeHandle nh;
  ros::Subscriber judge_sub = nh.subscribe("rear_judgement", 1000, &V2X_Action::judgeCallback,this);

  ros::ServiceClient control_client = nh.serviceClient<mid_project::emergency>("emergency");
  ros::ServiceClient client = nh.serviceClient<mid_project::waypoint>("waypoint_check");

  actionlib::SimpleActionServer<mid_project::Action1Action> as_;
  std::string action_name_;
  mid_project::Action1Feedback feedback_;
  mid_project::Action1Result result_;
  mid_project::emergency srv;
 mid_project::waypoint srv1;
  mid_project::rear_judge msg;

  int num=1;
  int point=1;
  int angle=45;
  bool sig=false;


public:

  V2X_Action(std::string name)
      :as_(nh,name,boost::bind(&V2X_Action::executeCB,this,_1),false),
      action_name_(name){
        as_.start();
      }

    ~V2X_Action(void) {}

 void judgeCallback(const mid_project::rear_judge::ConstPtr& msg)
    {
      if(int(msg->judge)==1){
         ROS_INFO_STREAM("!!Emergency Situation at rear!!");
         sig=true;
          }
          else {
            ROS_INFO_STREAM("Nothing special..");
            sig=false;
          }
        }

    void executeCB(const mid_project::Action1GoalConstPtr& goal){
      ros::Rate r(1);
      while(!as_.isPreemptRequested()|| ros::ok())
          {

        if(sig==goal->order){
          srv.request.a=true;
          as_.setSucceeded(result_);
          result_.sequence=angle;
          control_client.call(srv);

          ROS_INFO_STREAM("CHANGE OUR LANE, Turn your handle angle: "<<result_.sequence);
          break;

        }
        else {
          srv.request.a=false;
          feedback_.sequence = num;
          srv1.request.a=point;
          srv1.request.b=point+1;
          client.call(srv1);
          as_.publishFeedback(feedback_);
          ROS_INFO_STREAM("KEEP OUR LANE "<<feedback_.sequence);

        }
            r.sleep();
            ros::spinOnce();

             point++;
          }

}

};




int main(int argc, char **argv)
{
  ros::init(argc, argv, "V2X");
  V2X_Action V2X_Act("V2X");
  ros::spin();

  return 0;
}
