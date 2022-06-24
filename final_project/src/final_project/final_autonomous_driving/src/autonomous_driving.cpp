#include <math.h>
#include <ros/ros.h>
#include <string>
#include "autonomous_msg/PolyfitLaneDataArray.h"
#include "autonomous_msg/VehicleInput.h"
#include "autonomous_msg/VehicleOutput.h"
#include "autonomous_msg/EnvironmentDetected.h"
#include "std_msgs/Float64.h"

class AutonomousDriving {
 protected:
  ros::NodeHandle m_rosNodeHandler;
  ros::Publisher m_rosPubVehicleInput;

  ros::Subscriber m_rosSubVehicleState;
  ros::Subscriber m_rosSubPolyLanes;
  ros::Subscriber m_rosSubDrivingInput;
  ros::Subscriber m_rosLimitSpeed;
  ros::Subscriber m_rosSubLidar;
  ros::Subscriber m_rosSubEnvironmentMode;

  ros::Publisher m_rosPubDrivingWay;

  std::string m_vehicle_namespace_param;
  double m_lookAhead_param = 0.0;

 public:
  AutonomousDriving() {
    m_rosSubVehicleState = m_rosNodeHandler.subscribe(
        "vehicle_output", 10, &AutonomousDriving::vehicleStateCallback, this);

    m_rosPubVehicleInput =
        m_rosNodeHandler.advertise<autonomous_msg::VehicleInput>(
            "vehicle_input", 10);

    m_rosSubPolyLanes = m_rosNodeHandler.subscribe(
        "polyfit_lanes", 10, &AutonomousDriving::polyLanesCallback, this);

    m_rosLimitSpeed = m_rosNodeHandler.subscribe(
        "/limit_speed", 10, &AutonomousDriving::limitSpeedCallback, this);

    m_rosSubDrivingInput = m_rosNodeHandler.subscribe(
        "/driving_input", 10, &AutonomousDriving::drivingInputCallback, this);

    m_rosSubLidar = m_rosNodeHandler.subscribe(
        "/lidar", 10, &AutonomousDriving::lidarSensorCallback, this);

    m_rosSubEnvironmentMode = m_rosNodeHandler.subscribe(
        "/environment_detected", 10, &AutonomousDriving::environmentModeCallback, this);

    m_rosPubDrivingWay =
        m_rosNodeHandler.advertise<autonomous_msg::PolyfitLaneData>(
            "driving_way", 10);

    m_rosNodeHandler.param("autonomous_driving/ns", m_vehicle_namespace_param,
                           std::string(""));
    m_rosNodeHandler.param("autonomous_driving/lookAhead", m_lookAhead_param,
                           5.0);
  }

  ~AutonomousDriving() {}

 protected:
  autonomous_msg::PolyfitLaneDataArray m_polyLanes;
  autonomous_msg::VehicleInput m_drivingInput;
  autonomous_msg::PolyfitLaneData m_midPolyLane;
  autonomous_msg::VehicleOutput m_vehicleState;
  autonomous_msg::VehicleOutput m_lidarOutput;
  double m_limitSpeed = 0.0;
  bool isLidarDataExist = false;
  std::string m_iceMode = "Asphalt";
  double m_iceParam = 6200.0;

  autonomous_msg::PolyfitLaneData prev_leftlane;
  autonomous_msg::PolyfitLaneData prev_rightlane;

  double e_Kp=0.0; double e_Kd; double e_Ki=0.0; double pre_err=0; double e_integral=0;
  double lane_width=0.0; double prev_steering=0.0; double target_speed=10.0;


 public:
  /**
   * @brief Temporary functions for debugging pure pursuit
   *
   * @param VehicleInput accel and brake
   */
  void drivingInputCallback(const autonomous_msg::VehicleInput::ConstPtr &msg) {
    m_drivingInput.accel = msg->accel;
    m_drivingInput.brake = msg->brake;
    // m_drivingInput.steering = msg->steering;
  }

  void vehicleStateCallback(
      const autonomous_msg::VehicleOutput::ConstPtr &msg) {
    m_vehicleState = *msg;
  }

  void limitSpeedCallback(const std_msgs::Float64::ConstPtr &msg) {
    m_limitSpeed = msg->data;
  }

  void polyLanesCallback(
      const autonomous_msg::PolyfitLaneDataArray::ConstPtr &msg) {
    m_polyLanes = *msg;
  }
  void lidarSensorCallback(const autonomous_msg::VehicleOutput::ConstPtr &msg) {
    m_lidarOutput = *msg;
    isLidarDataExist = true;
  }

  void environmentModeCallback(const autonomous_msg::EnvironmentDetected::ConstPtr &msg) {
    m_iceMode = msg->ice_mode;
    m_iceParam = msg->ice_param;
  }

  void controlVehicleSpeed() {
    // TODO

    double vehicle_move=sqrt(pow(m_vehicleState.x,2)+pow(m_vehicleState.y,2));
    double err=target_speed-m_vehicleState.velocity;

    if(vehicle_move<10.0){
           m_drivingInput.accel=0.1;
           m_drivingInput.brake=0.0;
         }
        else if(m_vehicleState.velocity<5.0&&vehicle_move>10.0){
          m_drivingInput.accel=1.0;
          m_drivingInput.brake=0.0;
        }



    else if(m_iceMode=="Ice"){
      if(m_vehicleState.velocity<8){
        m_drivingInput.accel=0.0;
        m_drivingInput.brake+=m_drivingInput.steering*m_limitSpeed*0.005;
      }
      else {
        m_drivingInput.accel=0.0;
        m_drivingInput.brake+=m_drivingInput.steering*m_limitSpeed*0.01;
      }

             }

    else if(m_limitSpeed*0.8>m_vehicleState.velocity){
      if(m_drivingInput.steering<0.01){

        target_speed=0.8*m_limitSpeed;
        if(target_speed>20){
          target_speed=12.0;
        }
        if(m_vehicleState.velocity>target_speed){
          m_drivingInput.accel=0.0;
          m_drivingInput.brake+=1.0;
        }
        else{
          m_drivingInput.accel+=err*0.5;
          m_drivingInput.brake=0.0;
        }
      }
      else if(m_drivingInput.steering<0.02){
        target_speed=0.8*m_limitSpeed;
        if(target_speed>11){
          target_speed=11.0;
        }
        if(m_vehicleState.velocity>target_speed){
          m_drivingInput.accel=0.0;
          m_drivingInput.brake+=1.0;
        }
        else{
          m_drivingInput.accel+=err*0.5;
          m_drivingInput.brake=0.0;
        }
      }
      else if(m_drivingInput.steering<0.05){
        target_speed=0.8*m_limitSpeed;
        if(target_speed>8){
          target_speed=8.0;
        }
        if(m_vehicleState.velocity>target_speed){
          m_drivingInput.accel=0.0;
          m_drivingInput.brake+=1.0;
        }
        else{
          m_drivingInput.accel+=err*0.5;
          m_drivingInput.brake=0.0;
        }
      }

      else {
        target_speed=0.8*m_limitSpeed;
        if(target_speed>8){
          target_speed=8.0;
        }
        if(m_vehicleState.velocity>target_speed){
          m_drivingInput.accel=0.0;
          m_drivingInput.brake+=1.0;
        }
        else{
          m_drivingInput.accel+=err*0.5;
          m_drivingInput.brake=0.0;
        }
      }
      }
    else{
      if(m_drivingInput.steering<0.01){
        target_speed=0.8*m_limitSpeed;
        if(target_speed>20){
          target_speed=12.0;
        }
        if(m_vehicleState.velocity>target_speed){
          m_drivingInput.accel=0.0;
          m_drivingInput.brake+=1.0;
        }
        else{
          m_drivingInput.accel+=err*0.5;
          m_drivingInput.brake=0.0;
        }
      }
      else if(m_drivingInput.steering<0.02){
        target_speed=0.8*m_limitSpeed;
        if(target_speed>11){
          target_speed=11;
        }
        if(m_vehicleState.velocity>target_speed){
          m_drivingInput.accel=0.0;
          m_drivingInput.brake+=1.0;
        }
        else{
          m_drivingInput.accel+=err*0.5;
          m_drivingInput.brake=0.0;
        }
      }
     else if(m_drivingInput.steering<0.05){
        target_speed=0.8*m_limitSpeed;
        if(target_speed>8){
          target_speed=8;
        }
        if(m_vehicleState.velocity>target_speed){
          m_drivingInput.accel=0.0;
          m_drivingInput.brake+=1.0;
        }
        else{
          m_drivingInput.accel+=err*0.5;
          m_drivingInput.brake=0.0;
        }
      }

      else {
        target_speed=0.8*m_limitSpeed;
        if(target_speed>6){
          target_speed=6.0;
        }
        if(m_vehicleState.velocity>target_speed){
          m_drivingInput.accel=0.0;
          m_drivingInput.brake+=1.0;
        }
        else{
          m_drivingInput.accel+=err*0.5;
          m_drivingInput.brake=0.0;
        }
      }
    }






  }

  /**
   * brief: Find the left lane and the right lane, then change to the actual
   * driving lane. input: m_polyLanes output: m_midPolyLane
   */
  void findDrivingWay() {
    // TODO
    autonomous_msg::PolyfitLaneData leftlane;
    autonomous_msg::PolyfitLaneData rightlane;

    leftlane.a0=100; rightlane.a0=100;
    double vehicle_move=sqrt(pow(m_vehicleState.x,2)+pow(m_vehicleState.y,2));

    if(m_polyLanes.polyfitLanes.size()==2){
      if(m_polyLanes.polyfitLanes[0].a0*m_polyLanes.polyfitLanes[1].a0<0){


        if(vehicle_move<1.0){
          for (auto i_lane = 0;i_lane < m_polyLanes.polyfitLanes.size(); i_lane++) {
              if(m_polyLanes.polyfitLanes[i_lane].a0 > 0 && m_polyLanes.polyfitLanes[i_lane].a0 < rightlane.a0){
                rightlane.a0 = m_polyLanes.polyfitLanes[i_lane].a0;
                rightlane.a1 = m_polyLanes.polyfitLanes[i_lane].a1;
                rightlane.a2 = m_polyLanes.polyfitLanes[i_lane].a2;
                rightlane.a3 = m_polyLanes.polyfitLanes[i_lane].a3;
                }
               else if(m_polyLanes.polyfitLanes[i_lane].a0<0 && m_polyLanes.polyfitLanes[i_lane].a0 < leftlane.a0) {
                leftlane.a0 = m_polyLanes.polyfitLanes[i_lane].a0;
                leftlane.a1 = m_polyLanes.polyfitLanes[i_lane].a1;
                leftlane.a2 = m_polyLanes.polyfitLanes[i_lane].a2;
                leftlane.a3 = m_polyLanes.polyfitLanes[i_lane].a3;
                }

                lane_width=abs(m_polyLanes.polyfitLanes[1].a0-m_polyLanes.polyfitLanes[0].a0);


          }
        }
      else if(abs(m_polyLanes.polyfitLanes[0].a0-m_polyLanes.polyfitLanes[1].a0)<lane_width*1.01){//정상적인식

          for (auto i_lane = 0;i_lane < m_polyLanes.polyfitLanes.size(); i_lane++) {
              if(m_polyLanes.polyfitLanes[i_lane].a0 > 0 && m_polyLanes.polyfitLanes[i_lane].a0 < rightlane.a0){
                rightlane.a0 = m_polyLanes.polyfitLanes[i_lane].a0;
                rightlane.a1 = m_polyLanes.polyfitLanes[i_lane].a1;
                rightlane.a2 = m_polyLanes.polyfitLanes[i_lane].a2;
                rightlane.a3 = m_polyLanes.polyfitLanes[i_lane].a3;
                }
               else if(m_polyLanes.polyfitLanes[i_lane].a0<0 && m_polyLanes.polyfitLanes[i_lane].a0 < leftlane.a0) {
                leftlane.a0 = m_polyLanes.polyfitLanes[i_lane].a0;
                leftlane.a1 = m_polyLanes.polyfitLanes[i_lane].a1;
                leftlane.a2 = m_polyLanes.polyfitLanes[i_lane].a2;
                leftlane.a3 = m_polyLanes.polyfitLanes[i_lane].a3;
                }

          }


          }



      else if(abs(m_polyLanes.polyfitLanes[0].a0-m_polyLanes.polyfitLanes[1].a0)>lane_width*1.01){//차선은 제대로 두개이지만 먼 차선을 읽어버렸을때
          if(m_polyLanes.polyfitLanes[0].a0<0 &&abs(m_polyLanes.polyfitLanes[0].a0)> abs(m_polyLanes.polyfitLanes[1].a0)){//왼쪽에 먼차선
            rightlane.a0 = m_polyLanes.polyfitLanes[1].a0;
            rightlane.a1 = m_polyLanes.polyfitLanes[1].a1;
            rightlane.a2 = m_polyLanes.polyfitLanes[1].a2;
            rightlane.a3 = m_polyLanes.polyfitLanes[1].a3;
            leftlane.a0 = rightlane.a0-lane_width;
            leftlane.a1 = rightlane.a1;
            leftlane.a2 = rightlane.a2;
            leftlane.a3 = rightlane.a3;
          }
          else if(m_polyLanes.polyfitLanes[0].a0<0 &&abs(m_polyLanes.polyfitLanes[0].a0)< abs(m_polyLanes.polyfitLanes[1].a0)){//오른쪽에 먼차선
            leftlane.a0 = m_polyLanes.polyfitLanes[0].a0;
            leftlane.a1 = m_polyLanes.polyfitLanes[0].a1;
            leftlane.a2 = m_polyLanes.polyfitLanes[0].a2;
            leftlane.a3 = m_polyLanes.polyfitLanes[0].a3;
            rightlane.a0 = leftlane.a0+lane_width;
            rightlane.a1 = leftlane.a1;
            rightlane.a2 = leftlane.a2;
            rightlane.a3 = leftlane.a3;
          }
          else if(m_polyLanes.polyfitLanes[0].a0>0 &&abs(m_polyLanes.polyfitLanes[0].a0)> abs(m_polyLanes.polyfitLanes[1].a0)){//오른쪽ㄱ에 먼차선
            leftlane.a0 = m_polyLanes.polyfitLanes[1].a0;
            leftlane.a1 = m_polyLanes.polyfitLanes[1].a1;
            leftlane.a2 = m_polyLanes.polyfitLanes[1].a2;
            leftlane.a3 = m_polyLanes.polyfitLanes[1].a3;
            rightlane.a0 = leftlane.a0+lane_width;
            rightlane.a1 = leftlane.a1;
            rightlane.a2 = leftlane.a2;
            rightlane.a3 = leftlane.a3;


          }
          else if(m_polyLanes.polyfitLanes[0].a0>0 &&abs(m_polyLanes.polyfitLanes[0].a0)< abs(m_polyLanes.polyfitLanes[1].a0)){//왼쪽 먼차선
            rightlane.a0 = m_polyLanes.polyfitLanes[0].a0;
            rightlane.a1 = m_polyLanes.polyfitLanes[0].a1;
            rightlane.a2 = m_polyLanes.polyfitLanes[0].a2;
            rightlane.a3 = m_polyLanes.polyfitLanes[0].a3;
            leftlane.a0 = rightlane.a0-lane_width;
            leftlane.a1 = rightlane.a1;
            leftlane.a2 = rightlane.a2;
            leftlane.a3 = rightlane.a3;
          }

        }

      }


      else if (m_polyLanes.polyfitLanes[0].a0*m_polyLanes.polyfitLanes[1].a0>0) {//같은방향차선 두개 인식

        if(m_polyLanes.polyfitLanes[0].a0 > 0){
          if(m_polyLanes.polyfitLanes[0].a0 < m_polyLanes.polyfitLanes[1].a0){//오른쪽 두개 차선인식
            rightlane.a0 = m_polyLanes.polyfitLanes[0].a0;
            rightlane.a1 = m_polyLanes.polyfitLanes[0].a1;
            rightlane.a2 = m_polyLanes.polyfitLanes[0].a2;
            rightlane.a3 = m_polyLanes.polyfitLanes[0].a3;
            leftlane.a0 = rightlane.a0-lane_width;
            leftlane.a1 = rightlane.a1;
            leftlane.a2 = rightlane.a2;
            leftlane.a3 = rightlane.a3;
          }
          else if(m_polyLanes.polyfitLanes[0].a0 > m_polyLanes.polyfitLanes[1].a0){
            rightlane.a0 = m_polyLanes.polyfitLanes[1].a0;
            rightlane.a1 = m_polyLanes.polyfitLanes[1].a1;
            rightlane.a2 = m_polyLanes.polyfitLanes[1].a2;
            rightlane.a3 = m_polyLanes.polyfitLanes[1].a3;
            leftlane.a0 = rightlane.a0-lane_width;
            leftlane.a1 = rightlane.a1;
            leftlane.a2 = rightlane.a2;
            leftlane.a3 = rightlane.a3;
          }




        }
        else if(m_polyLanes.polyfitLanes[0].a0 < 0){//왼쪽 두개 차선인식

          if(abs(m_polyLanes.polyfitLanes[0].a0) > abs(m_polyLanes.polyfitLanes[1].a0)){
            leftlane.a0 = m_polyLanes.polyfitLanes[1].a0;
            leftlane.a1 = m_polyLanes.polyfitLanes[1].a1;
            leftlane.a2 = m_polyLanes.polyfitLanes[1].a2;
            leftlane.a3 = m_polyLanes.polyfitLanes[1].a3;
            rightlane.a0 = leftlane.a0+lane_width;
            rightlane.a1 = leftlane.a1;
            rightlane.a2 = leftlane.a2;
            rightlane.a3 = leftlane.a3;
          }
          else if(abs(m_polyLanes.polyfitLanes[0].a0) < abs(m_polyLanes.polyfitLanes[1].a0)){
            leftlane.a0 = m_polyLanes.polyfitLanes[0].a0;
            leftlane.a1 = m_polyLanes.polyfitLanes[0].a1;
            leftlane.a2 = m_polyLanes.polyfitLanes[0].a2;
            leftlane.a3 = m_polyLanes.polyfitLanes[0].a3;
            rightlane.a0 = leftlane.a0+lane_width;
            rightlane.a1 = leftlane.a1;
            rightlane.a2 = leftlane.a2;
            rightlane.a3 = leftlane.a3;
          }
        }




      }
      else{
        leftlane.a0=prev_leftlane.a0;
        leftlane.a1=prev_leftlane.a1;
        leftlane.a2=prev_leftlane.a2;
        leftlane.a3=prev_leftlane.a3;
        rightlane.a0=prev_rightlane.a0;
        rightlane.a1=prev_rightlane.a1;
        rightlane.a2=prev_rightlane.a2;
        rightlane.a3=prev_rightlane.a3;



      }
      m_midPolyLane.a0=0.5*(rightlane.a0+leftlane.a0);
      m_midPolyLane.a1=0.5*(rightlane.a1+leftlane.a1);
      m_midPolyLane.a2=0.5*(rightlane.a2+leftlane.a2);
      m_midPolyLane.a3=0.5*(rightlane.a3+leftlane.a3);

      prev_leftlane=leftlane; prev_rightlane=rightlane;






}


     else if(m_polyLanes.polyfitLanes.size()==1){


        if(m_polyLanes.polyfitLanes[0].a0 > 0 && m_polyLanes.polyfitLanes[0].a0 < rightlane.a0){//오른차선인식

          rightlane.a0 = m_polyLanes.polyfitLanes[0].a0;
          rightlane.a1 = m_polyLanes.polyfitLanes[0].a1;
          rightlane.a2 = m_polyLanes.polyfitLanes[0].a2;
          rightlane.a3 = m_polyLanes.polyfitLanes[0].a3;
          leftlane.a0 = rightlane.a0-lane_width;
          leftlane.a1 = rightlane.a1;
          leftlane.a2 = rightlane.a2;
          leftlane.a3 = rightlane.a3;
          }
         else if(m_polyLanes.polyfitLanes[0].a0<0 && m_polyLanes.polyfitLanes[0].a0 < leftlane.a0) {//왼차선인식￣

          leftlane.a0 = m_polyLanes.polyfitLanes[0].a0;
          leftlane.a1 = m_polyLanes.polyfitLanes[0].a1;
          leftlane.a2 = m_polyLanes.polyfitLanes[0].a2;
          leftlane.a3 = m_polyLanes.polyfitLanes[0].a3;
          rightlane.a0 = leftlane.a0+lane_width;
          rightlane.a1 = leftlane.a1;
          rightlane.a2 = leftlane.a2;
          rightlane.a3 = leftlane.a3;
          }
          m_midPolyLane.a0=0.5*(rightlane.a0+leftlane.a0);
          m_midPolyLane.a1=0.5*(rightlane.a1+leftlane.a1);
          m_midPolyLane.a2=0.5*(rightlane.a2+leftlane.a2);
          m_midPolyLane.a3=0.5*(rightlane.a3+leftlane.a3);


          prev_leftlane=leftlane; prev_rightlane=rightlane;


        }



     else if(m_polyLanes.polyfitLanes.size()==3){//lane이 3개 인식 될때는 가장 가까운 두놈만

        float max= abs(m_polyLanes.polyfitLanes[0].a0);
        int idx=0;
         for (auto i_lane = 0;i_lane < m_polyLanes.polyfitLanes.size(); i_lane++) {
           if(abs(m_polyLanes.polyfitLanes[i_lane].a0)>max){
             max=abs(m_polyLanes.polyfitLanes[i_lane].a0);
             idx=i_lane;
           }
         }
         for (auto i_lane = 0;i_lane < m_polyLanes.polyfitLanes.size(); i_lane++) {
           if(i_lane==idx){
             continue;
           }
           if(m_polyLanes.polyfitLanes[i_lane].a0 > 0 && m_polyLanes.polyfitLanes[i_lane].a0 < rightlane.a0){
             rightlane.a0 = m_polyLanes.polyfitLanes[i_lane].a0;
             rightlane.a1 = m_polyLanes.polyfitLanes[i_lane].a1;
             rightlane.a2 = m_polyLanes.polyfitLanes[i_lane].a2;
             rightlane.a3 = m_polyLanes.polyfitLanes[i_lane].a3;
             }
            else if(m_polyLanes.polyfitLanes[i_lane].a0<0 && m_polyLanes.polyfitLanes[i_lane].a0 < leftlane.a0) {
             leftlane.a0 = m_polyLanes.polyfitLanes[i_lane].a0;
             leftlane.a1 = m_polyLanes.polyfitLanes[i_lane].a1;
             leftlane.a2 = m_polyLanes.polyfitLanes[i_lane].a2;
             leftlane.a3 = m_polyLanes.polyfitLanes[i_lane].a3;
             }
       }
             m_midPolyLane.a0=0.5*(rightlane.a0+leftlane.a0);
             m_midPolyLane.a1=0.5*(rightlane.a1+leftlane.a1);
             m_midPolyLane.a2=0.5*(rightlane.a2+leftlane.a2)  ;
             m_midPolyLane.a3=0.5*(rightlane.a3+leftlane.a3);
             prev_leftlane=leftlane; prev_rightlane=rightlane;
             if(vehicle_move<1.0){lane_width=abs(m_polyLanes.polyfitLanes[1].a0-m_polyLanes.polyfitLanes[0].a0);
}


           }




     else if(m_polyLanes.polyfitLanes.size()==0){

    /*  if(abs(prev_leftlane.a0)>abs(prev_rightlane.a0)){//직전에 왼쪽 레인과 가까웟을 경우
        leftlane.a0=prev_leftlane.a0;
        leftlane.a1=prev_leftlane.a1;
        leftlane.a2=prev_leftlane.a2;
        leftlane.a3=prev_leftlane.a3;
        rightlane.a0 = leftlane.a0+lane_width;
        rightlane.a1 = leftlane.a1;
        rightlane.a2 = leftlane.a2;
        rightlane.a3 = leftlane.a3;

      }
      else if(abs(prev_leftlane.a0)<abs(prev_rightlane.a0)){//직전에 오른쪽 레인과 가까웟을 경우
        rightlane.a0=prev_rightlane.a0;
        rightlane.a1=prev_rightlane.a1;
        rightlane.a2=prev_rightlane.a2;
        rightlane.a3=prev_rightlane.a3;
        leftlane.a0 = rightlane.a0-lane_width;
        leftlane.a1 = rightlane.a1;
        leftlane.a2 = rightlane.a2;
        leftlane.a3 = rightlane.a3;

      }*/

           leftlane.a0=prev_leftlane.a0;
            leftlane.a1=prev_leftlane.a1;
            leftlane.a2=prev_leftlane.a2;
            leftlane.a3=prev_leftlane.a3;
            rightlane.a0=prev_rightlane.a0;
            rightlane.a1=prev_rightlane.a1;
            rightlane.a2=prev_rightlane.a2;
            rightlane.a3=prev_rightlane.a3;

            m_midPolyLane.a0=0.5*(rightlane.a0+leftlane.a0);
            m_midPolyLane.a1=0.5*(rightlane.a1+leftlane.a1);
            m_midPolyLane.a2=0.5*(rightlane.a2+leftlane.a2);
            m_midPolyLane.a3=0.5*(rightlane.a3+leftlane.a3);

            prev_leftlane=leftlane; prev_rightlane=rightlane;

            //lane_width=rightlane.a0-leftlane.a0;


          }
    else if(m_polyLanes.polyfitLanes.size()>3) {
         leftlane.a0=prev_leftlane.a0;
         leftlane.a1=prev_leftlane.a1;
         leftlane.a2=prev_leftlane.a2;
         leftlane.a3=prev_leftlane.a3;
         rightlane.a0=prev_rightlane.a0;
         rightlane.a1=prev_rightlane.a1;
             rightlane.a2=prev_rightlane.a2;
             rightlane.a3=prev_rightlane.a3;

             m_midPolyLane.a0=0.5*(rightlane.a0+leftlane.a0);
             m_midPolyLane.a1=0.5*(rightlane.a1+leftlane.a1);
             m_midPolyLane.a2=0.5*(rightlane.a2+leftlane.a2);
             m_midPolyLane.a3=0.5*(rightlane.a3+leftlane.a3);


             prev_leftlane=leftlane; prev_rightlane=rightlane;

    }

   //ROS_INFO_STREAM("position"<<m_vehicleState.x<<m_vehicleState.y);
    //ROS_INFO_STREAM("lane heading: "<<lane_width);
    m_midPolyLane.frame_id = m_vehicle_namespace_param + "/body";
    m_rosPubDrivingWay.publish(m_midPolyLane);
  }


  /**
   * brief: Find the steering angle for driving in the driving lane.
   * input: m_midPolyLane
   * output: m_drivingInput.steering
   */
  void calcSteeringAngle() {

    // TODO
    double m_lookAhead_param_x=5;
    double g_x=m_lookAhead_param_x;
    double g_y=m_midPolyLane.a3*pow(g_x,3)+m_midPolyLane.a2*pow(g_x,2)+m_midPolyLane.a1*g_x+m_midPolyLane.a0;
    double e_ld=abs(g_y);
    m_lookAhead_param=sqrt(g_x*g_x+g_y*g_y);

/*    if(m_polyLanes.polyfitLanes.size()==0&&(abs(prev_leftlane.a1)+abs(prev_rightlane.a1))/2>0.002){
          m_lookAhead_param_x=15;
        }

    else if(m_polyLanes.polyfitLanes.size()==0&&(abs(prev_leftlane.a1)+abs(prev_rightlane.a1))/2<0.002){
      m_drivingInput.steering=0;
    }
   else if(m_polyLanes.polyfitLanes.size()==2 ||m_polyLanes.polyfitLanes.size()==3 ){
      if(m_midPolyLane.a1>0.02){
        m_lookAhead_param_x=7;
      }
      else{
         m_lookAhead_param_x=10;
      }

    }
    else if(m_polyLanes.polyfitLanes.size()==1){
      if(m_midPolyLane.a1<0.02){
        m_lookAhead_param_x=7;
      }
      else{
         m_lookAhead_param_x=15;
      }

    }


   else{
     m_lookAhead_param_x=8;

   }

   if(m_lookAhead_param==m_lookAhead_param_x){
                       m_drivingInput.steering=0.0;
                                         }
         else if(g_y<0){
            m_drivingInput.steering=-atan((2*2.7*e_ld)/(m_lookAhead_param*m_lookAhead_param));
                                     }
          else if(g_y>0){
            m_drivingInput.steering=atan((2*2.7*e_ld)/(m_lookAhead_param*m_lookAhead_param));
                                    }
                                    */

if(m_polyLanes.polyfitLanes.size()==2||m_polyLanes.polyfitLanes.size()==3){
    if((m_midPolyLane.a1)<0.04){
      m_lookAhead_param_x=10;
    }
    else if((m_midPolyLane.a1)>0.04&&(m_midPolyLane.a1)<0.1){
      m_lookAhead_param_x=8;
    }

    else if((m_midPolyLane.a1)>0.1){
      m_lookAhead_param_x=5;
    }

    if(m_lookAhead_param==m_lookAhead_param_x){
                  m_drivingInput.steering=0.0;
                                    }
    else if(g_y<0){
       m_drivingInput.steering=-atan((2*2.7*e_ld)/(m_lookAhead_param*m_lookAhead_param));
                                }
     else if(g_y>0){
       m_drivingInput.steering=atan((2*2.7*e_ld)/(m_lookAhead_param*m_lookAhead_param));
                               }
}


else if(m_polyLanes.polyfitLanes.size()==1){

     if((m_midPolyLane.a1)<0.02){
      m_lookAhead_param_x=8;
    }

    else if((m_midPolyLane.a1)<0.06){
      m_lookAhead_param_x=6;
    }

    else {
      m_lookAhead_param_x=5;
    }
    if(m_lookAhead_param==m_lookAhead_param_x){
                  m_drivingInput.steering=0.0;
                                    }
    else if(g_y<0){
       m_drivingInput.steering=-atan((2*2.7*e_ld)/(m_lookAhead_param*m_lookAhead_param));
                                }
     else if(g_y>0){
       m_drivingInput.steering=atan((2*2.7*e_ld)/(m_lookAhead_param*m_lookAhead_param));
                               }
}
else if(m_polyLanes.polyfitLanes.size()==0){
  m_lookAhead_param_x=7;
  if((abs(prev_leftlane.a1)+abs(prev_rightlane.a1))/2>0.01){
    if(m_lookAhead_param==m_lookAhead_param_x){
                        m_drivingInput.steering=0.0;
                                          }
          else if(g_y<0){
             m_drivingInput.steering=-atan((2*2.7*e_ld)/(m_lookAhead_param*m_lookAhead_param));
                                      }
           else if(g_y>0){
             m_drivingInput.steering=atan((2*2.7*e_ld)/(m_lookAhead_param*m_lookAhead_param));
                                     }
  }
  else{
     m_drivingInput.steering=0;
  }



}
else{

  m_lookAhead_param_x=10;
  if(m_lookAhead_param==m_lookAhead_param_x){
                m_drivingInput.steering=0.0;
                                  }
  else if(g_y<0){
     m_drivingInput.steering=-atan((2*2.7*e_ld)/(m_lookAhead_param*m_lookAhead_param));
                              }
   else if(g_y>0){
     m_drivingInput.steering=atan((2*2.7*e_ld)/(m_lookAhead_param*m_lookAhead_param));
                             }
}







   // m_drivingInput.steering = 0.0;

}


  void publishVehicleInput() { m_rosPubVehicleInput.publish(m_drivingInput); }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "autonomous_driving");

  AutonomousDriving autonomousDriving;

  double prev_csvLaneMarkTime = ros::Time::now().toSec();
  // The approximate control time is 100 Hz
  ros::Rate loop_rate(100);
  while (ros::ok()) {
    autonomousDriving.controlVehicleSpeed();
    autonomousDriving.findDrivingWay();
    autonomousDriving.calcSteeringAngle();
    autonomousDriving.publishVehicleInput();



    if ((ros::Time::now().toSec() - prev_csvLaneMarkTime) > 1.0) {
      prev_csvLaneMarkTime = ros::Time::now().toSec();
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
