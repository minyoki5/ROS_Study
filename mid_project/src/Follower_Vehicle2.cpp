#include "ros/ros.h"
#include "mid_project/rear_cam.h"
#include "mid_project/rear_judge.h"
#include <iostream>


void CamCallback(const mid_project::rear_cam::ConstPtr& msg)
{
  if(msg->data=="Well"){
     ROS_INFO_STREAM("Signal Checking "<< (msg->data));
 }
  else {
    ROS_INFO_STREAM("Signal "<< (msg->data));
  }

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Follower_Vehicle2");
  ros::NodeHandle nh;
  ros::Rate loop_rate(1);
  mid_project::rear_judge msg;

  ros::Subscriber sub = nh.subscribe("rear_info", 1000, CamCallback);
  ros::Publisher judge_pub = nh.advertise<mid_project::rear_judge>("rear_judgement", 1000);
  int value=0;
  int count=1;


  while(ros::ok()){

    if(count%20==0){
     ROS_INFO_STREAM("Police Car is coming.. ");
     value=3;
    }
    else if (count%30==0) {
      ROS_INFO_STREAM("Ambulance is coming.. ");
      value=1;
    }
    else if (count%50==0) {
      ROS_INFO_STREAM("Accident at rear occurred..");
      value=0;
    }
    else {
      ROS_INFO_STREAM("Nothing at rear. ");
      value=0;
    }
    msg.judge=value;

    judge_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
    count++;

  }

  return 0;
}
