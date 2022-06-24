#include "ros/ros.h"
#include "mid_project/rear_cam.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Rear_Cam");
  ros::NodeHandle nh;

  ros::Publisher cam_pub = nh.advertise<mid_project::rear_cam>("rear_info", 1000);

  ros::Rate loop_rate(1);
  mid_project::rear_cam msg;
  std::string signal="Well";
  std::string signal1="Delay";
  int count=0;

  while (ros::ok())
  {

    if(count%10==0){
      msg.data=signal1;
      ROS_INFO_STREAM("Doing Signal Processing " <<msg.data);
    }
    else {
      msg.data=signal;
      ROS_INFO_STREAM("Signal Process " <<msg.data);
    }

    cam_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    count++;
  }

  return 0;
}
