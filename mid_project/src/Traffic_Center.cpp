#include "ros/ros.h"
#include "mid_project/traffic_info.h"
#include <cstdlib>

int main(int argc, char **argv){

    ros::init(argc, argv, "Traffic_Center");
    ros::NodeHandle nh;
    ros::Publisher traffic_pub = nh.advertise<mid_project::traffic_info>("traffic_information",1000);


    ros::Rate loop_rate(1);
    mid_project::traffic_info msg;
    int count=1;


    while (ros::ok()) {

      msg.data=count;

      if(count%30==0){
        ROS_INFO_STREAM("Predicting rear emergency situation!  ");
      }
      if(count%11==0){
        ROS_INFO_STREAM("Traffic information no."<<msg.data);
      }


      traffic_pub.publish(msg);
      ros::spinOnce();
      loop_rate.sleep();
      count++;

    }
    return 0;
}
