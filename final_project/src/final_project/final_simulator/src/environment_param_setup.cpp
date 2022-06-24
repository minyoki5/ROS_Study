/**
 * @file environment_param_setup.cpp
 * @author Junhee Lee (998jun@gmail.com)
 * @brief 주행환경 변화에 대한 클래스
 * @details
 * @version 0.1
 * @date 2021-11-09
 *
 * @copyright Copyright (c) 2021
 *
 */

#include "environment_param_setup.h"
#include "autonomous_msg/EnvironmentParam.h"
#include "autonomous_msg/EnvironmentMode.h"
#include "autonomous_msg/EnvironmentDetected.h"
#include "autonomous_msg/EnvironmentModeArea.h"
#include "autonomous_msg/VehicleOutput.h"
#include <tf/transform_listener.h>
#include "geometry_msgs/PointStamped.h"
#include <ros/ros.h>
#include <vector>
#include <math.h>

#define PLAIN = 0
#define UP_HILL = 1
#define DOWN_HILL = 2

EnvironmentSetup::EnvironmentSetup(){
   m_rosPubEnvironmentParam_output =
         m_rosNodeHandler.advertise<autonomous_msg::EnvironmentParam>(
            "environment_param", 10);

   m_rosPubEnvironmentModeArea =
         m_rosNodeHandler.advertise<autonomous_msg::EnvironmentModeArea>(
            "/environment_area", 10);

   m_rosPubEnvironmentMode =
         m_rosNodeHandler.advertise<autonomous_msg::EnvironmentMode>(
            "/environment_mode", 10);
   
   m_rosPubEnvironmentDetected =
         m_rosNodeHandler.advertise<autonomous_msg::EnvironmentDetected>(
            "/environment_detected", 10);

   m_rosSubVehicle_Output = m_rosNodeHandler.subscribe(
        "/back/vehicle_output", 10, &EnvironmentSetup::vehicleOutputCallback, this);

   m_rosNodeHandler.param("initFyfMax", this->initFyfMax, 6200.0);
   m_rosNodeHandler.param("initFyrMax", this->initFyrMax, 6200.0);
   m_rosNodeHandler.param("lane_detection/ROIFront", m_ROIFront_param, 20.0);
   m_rosNodeHandler.param("lane_detection/ROIRear", m_ROIRear_param, 10.0);
   m_rosNodeHandler.param("lane_detection/ROILeft", m_ROILeft_param, 3.0);
   m_rosNodeHandler.param("lane_detection/ROIRight", m_ROIRight_param, 3.0);
   m_rosNodeHandler.param("vehicle/ns", m_vehicle_namespace_param,
                           std::string(""));

   m_iceParam.push_back(IceParam(113.5144, -113.7806, 10.0, 2000.0, 2000.0));
   m_iceParam.push_back(IceParam(163.503, 106.617, 15.0, 4000.0, 4000.0));
   m_iceParam.push_back(IceParam(215.8292, -25.0859, 20.0, 3500.0, 3500.0));
   m_iceParam.push_back(IceParam(386.0347, -62.5237, 15.0, 4000.0, 4000.0));
   m_iceParam.push_back(IceParam(469.8816, -33.0636, 25.0, 4000.0, 4000.0));
   m_iceParam.push_back(IceParam(406.0232, 174.2993, 20.0, 1000.0, 1000.0));
   m_iceParam.push_back(IceParam(615.4918, -16.3427, 20.0, 1000.0, 1000.0));
   m_iceParam.push_back(IceParam(608.0437, 116.8927, 10.0, 1000.0, 1000.0));
   m_iceParam.push_back(IceParam(572.07, 304.448, 8.0, 1000.0, 1000.0));
   m_iceParam.push_back(IceParam(535.5730, 495.2167, 10.0, 1000.0, 1000.0));
   m_iceParam.push_back(IceParam(444.5648, 553.4633, 20.0, 1000.0, 1000.0));
   


   m_hillParam.push_back(HillParam(100.4, -105.55 , 10.0));
   m_hillParam.push_back(HillParam(161.98,-20.58 ,0.0 ));
   m_hillParam.push_back(HillParam(200.138, 242.51, -5.0 ));
   m_hillParam.push_back(HillParam(388.23,-82.81 ,15.0 ));
   m_hillParam.push_back(HillParam(336.73, 59.72,-10.0 ));
   m_hillParam.push_back(HillParam(478.28, -38.9308 ,0.0 ));
   m_hillParam.push_back(HillParam(416.94,278.98 , -12.0 ));
   m_hillParam.push_back(HillParam(596.87,-19.03 ,5.0 ));
   m_hillParam.push_back(HillParam(604.85,175.6 ,0.0 ));
  
   
}

EnvironmentSetup::~EnvironmentSetup(){}

void EnvironmentSetup::vehicleOutputCallback(const autonomous_msg::VehicleOutput::ConstPtr &msg){
   m_vehicleState = *msg;
   isSimulatorOn = true;
}

void EnvironmentSetup::masterVehicleCallback(const autonomous_msg::VehicleOutput::ConstPtr &msg) {
   m_masterVehicleState = *msg;
   isMasterDataExist = true;
}

void EnvironmentSetup::checkIce() {
   for (auto i = 0; i < m_iceParam.size(); i++) {
      double dx = m_vehicleState.x - m_iceParam[i].x;
      double dy = m_vehicleState.y - m_iceParam[i].y;
      double distance_sq = dx * dx + dy * dy;

      if (distance_sq <= m_iceParam[i].radius*m_iceParam[i].radius) {
         this->m_dFyf_max = m_iceParam[i].fyf_max;
         this->m_dFyr_max = m_iceParam[i].fyr_max;
         this->m_ice_mode = "Ice";
         break;
      }
      else{
         this->m_dFyf_max = this->initFyfMax;
         this->m_dFyr_max = this->initFyrMax;
         this->m_ice_mode = "Asphalt";
      }
   }
}

void EnvironmentSetup::checkHill() {
   for (auto i = 0; i < m_hillParam.size(); i++) {
      double dx = m_vehicleState.x - m_hillParam[i].x;
      double dy = m_vehicleState.y - m_hillParam[i].y;

      double distance_sq = dx * dx + dy * dy;
      if (distance_sq <= 5.0 * 5.0) {
         this->m_dSlopeAngle = m_hillParam[i].slope_angle;
         if (this->m_dSlopeAngle > 0.0){
            this->m_hill_mode = "Up Hill";
         }
         else if(this->m_dSlopeAngle < 0.0){
            this->m_hill_mode = "Down Hill";
         }
         else{
            this->m_hill_mode = "Plain";         
         }
      }
   }
}

void EnvironmentSetup::environmentAreaPublish(){
   std::vector<double> ice_x;  std::vector<double> ice_y;  std::vector<double> ice_radius;
   std::vector<double> hill_x; std::vector<double> hill_y; std::vector<double> hill_slope_angle;
   for (auto i = 0; i < m_iceParam.size(); i++) {
      ice_x.push_back(m_iceParam[i].x);
      ice_y.push_back(m_iceParam[i].y);
      ice_radius.push_back(m_iceParam[i].radius);
   }

   for(int i=0; i< m_hillParam.size(); i++){
      hill_x.push_back(m_hillParam[i].x);
      hill_y.push_back(m_hillParam[i].y);
      hill_slope_angle.push_back(m_hillParam[i].slope_angle);
   }

   autonomous_msg::EnvironmentModeArea env_mode_xy;
   env_mode_xy.ice_x = ice_x;   env_mode_xy.ice_y = ice_y;   env_mode_xy.ice_radius =  ice_radius;
   env_mode_xy.hill_x = hill_x; env_mode_xy.hill_y = hill_y; env_mode_xy.hill_slope_angle = hill_slope_angle;

   m_rosPubEnvironmentModeArea.publish(env_mode_xy);
}

void EnvironmentSetup::environmentParamPublish(){
   if(isSimulatorOn==true){ 
      this->checkHill();
      this->checkIce();
      this->environmentAreaPublish();     

      autonomous_msg::EnvironmentParam env_param_msg;
      env_param_msg.fyr_max = this->m_dFyr_max;
      env_param_msg.fyf_max = this->m_dFyf_max;
      env_param_msg.slope_angle = (this->m_dSlopeAngle)/ 180.0 * 3.141592;
      m_rosPubEnvironmentParam_output.publish(env_param_msg);

      autonomous_msg::EnvironmentMode env_mode_msg;
      env_mode_msg.ice_mode = this->m_ice_mode;
      env_mode_msg.hill_mode = this->m_hill_mode;
      m_rosPubEnvironmentMode.publish(env_mode_msg);

      autonomous_msg::EnvironmentDetected env_giving_msg;
      env_giving_msg.ice_mode = this->m_ice_mode;
      env_giving_msg.ice_param = this->m_dFyf_max;
      m_rosPubEnvironmentDetected.publish(env_giving_msg);

   }
}



// void EnvironmentSetup::giveIceInfo(){
//    for (auto i = 0; i < m_iceParam.size(); i++) {
//       geometry_msgs::PointStamped icePoint_world;
//       icePoint_world.header.frame_id = "/world";
//       icePoint_world.header.stamp = ros::Time(0);
//       icePoint_world.point.x = m_iceParam[i].x;
//       icePoint_world.point.y = m_iceParam[i].y;
//       // (m_iceParam[i].radius);
//       geometry_msgs::PointStamped icePoint_body;
//       try {
//          m_rosTfListenr.transformPoint( m_vehicle_namespace_param + "/body",
//                                        icePoint_world, icePoint_body);
         
         
//          double theta = atan2(icePoint_body.point.y,icePoint_body.point.x);
//          double hypo = sqrt(pow(icePoint_body.point.x,2)+pow(icePoint_body.point.y,2)) - m_iceParam[i].radius;
//          if(hypo < 0.0){
//             hypo = 0.0;
//          }
//          double arc_x = cos(theta)*hypo;
//          double arc_y = sin(theta)*hypo;
         
//          if ((arc_x <= m_ROIFront_param) &&
//             (arc_x >= -1 * m_ROIRear_param) &&
//             (arc_y <= m_ROILeft_param) &&
//             (arc_y >= -1 * m_ROIRight_param)) {
            
//             ROS_INFO_STREAM("--------\n" << arc_x <<" \n"<< arc_y <<"\n");
     
//             autonomous_msg::EnvironmentDetected env_detected;
//             env_detected.ice_x_body = icePoint_body.point.x;
//             env_detected.ice_y_body = icePoint_body.point.y;
//             env_detected.ice_radius = m_iceParam[i].radius;
//             m_rosPubEnvironmentDetected.publish(env_detected);
//          }

//       } catch (tf::TransformException &ex) {
//          // ROS_ERROR(ex.what());
//       }
//    }
// }


int main(int argc, char **argv) {

   ros::init(argc, argv, "environment_adjustment");
   EnvironmentSetup env_param;

   ros::Rate loop_rate(100);
   while (ros::ok()) {
      env_param.environmentParamPublish();
      // env_param.giveIceInfo();
      ros::spinOnce();
      loop_rate.sleep();
   }

   return 0;
}