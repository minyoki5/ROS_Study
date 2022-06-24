#ifndef ENVIRONMENT_PARAM_SETUP_H
#define ENVIRONMENT_PARAM_SETUP_H

#include <ros/ros.h>
#include <string>
#include "autonomous_msg/VehicleOutput.h"
#include <tf/transform_listener.h>


class IceParam {
public:
   double x = 0.0;
   double y = 0.0;
   double radius = 0.0;
   double fyf_max = 0.0;
   double fyr_max = 0.0;

public:
   IceParam() {}
   IceParam(double x, double y, double r, double fyf_max, double fyr_max) {
      this->x = x;
      this->y = y;
      this->radius = r;
      this->fyf_max = fyf_max;
      this->fyr_max = fyr_max;
   }
   ~IceParam() {}
};

class HillParam {
public:
   double x = 0.0;
   double y = 0.0;
   double slope_angle = 0.0;

public:
   HillParam() {}
   HillParam(double x, double y, double slope_angle) {
      this->x = x;
      this->y = y;
      this->slope_angle = slope_angle;
   }
   ~HillParam() {}
};

class EnvironmentSetup {
protected:
   ros::NodeHandle m_rosNodeHandler;
   ros::Publisher m_rosPubEnvironmentParam_output;
   ros::Publisher m_rosPubEnvironmentModeArea;
   ros::Publisher m_rosPubEnvironmentMode;
   ros::Publisher m_rosPubEnvironmentDetected;
   ros::Subscriber m_rosSubVehicle_Output;
   ros::Subscriber m_rosSubMasterVehicle_Output;
   
   tf::TransformListener m_rosTfListenr;

   autonomous_msg::VehicleOutput m_vehicleState;
   autonomous_msg::VehicleOutput m_masterVehicleState;
   std::vector<IceParam> m_iceParam;
   std::vector<HillParam> m_hillParam;
   bool isSimulatorOn = false;
   bool isMasterDataExist = false;
   
   double initFyfMax = 6200.0;
   double initFyrMax = 6200.0;
   double m_dFyf_max = 6200.0; // N/rad
   double m_dFyr_max = 6200.0; // N/rad
   double m_dSlopeAngle = 0.0;
   std::string m_ice_mode = "Asphalt";
   std::string m_hill_mode = "Plain";

   double m_ROIFront_param = 0.0;
   double m_ROIRear_param = 0.0;
   double m_ROILeft_param = 0.0;
   double m_ROIRight_param = 0.0;
   std::string m_vehicle_namespace_param;

public:
   EnvironmentSetup();
   ~EnvironmentSetup();
   void vehicleOutputCallback(const autonomous_msg::VehicleOutput::ConstPtr &msg);
   void masterVehicleCallback(const autonomous_msg::VehicleOutput::ConstPtr &msg);
   void checkIce();
   void checkHill();
   void environmentAreaPublish();
   void environmentParamPublish();
   void giveIceInfo();
};

#endif