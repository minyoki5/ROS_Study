#include "mid_project/emergency.h"
#include "mid_project/traffic_info.h"
#include "ros/ros.h"

bool emergency(mid_project::emergency::Request &req, mid_project::emergency::Response &res)
{   res.b = true;
    ROS_INFO_STREAM("Emergency recognized, Need to change line.");

}

void trafficCallback(const mid_project::traffic_info::ConstPtr& msg){
  if(int(msg->data)%30==0){
    ROS_INFO("Estimating traffic situation to change line");
  }


  }

int main(int argc, char **argv){
ros::init(argc, argv, "Platoon_Control");
ros::NodeHandle nh;

ros::Subscriber traffic_sub = nh.subscribe("traffic_information", 1000, trafficCallback);
ros::ServiceServer control_service = nh.advertiseService("emergency", emergency );
mid_project::traffic_info msg;

ROS_INFO("Emergency Detecting at rear.");

ros::spin();

return 0;

}
