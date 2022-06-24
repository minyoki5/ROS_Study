/**
 * @file evaluation.cpp
 * @author Eunsan Jo (eunsan.mountain@gmail.com)
 * @brief
 * @version 0.1
 * @date 2018-11-28
 *
 * @copyright Copyright (c) 2018
 *
 */
#include "KusvLane.hpp"
#include "autonomous_msg/LanePointDataArray.h"
#include "autonomous_msg/VehicleOutput.h"
#include "jsk_rviz_plugins/OverlayText.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include <math.h>
#include <ros/ros.h>
#include <string>
#include <tf/tf.h>

#define PI 3.1415926579
#define LOOPRATE (50.0)

class TargetSpeedPoint {
public:
  double x = 0.0;
  double y = 0.0;
  double targetSpeed = 0.0;

public:
  TargetSpeedPoint() {}
  TargetSpeedPoint(double x, double y, double targetSpeed) {
    this->x = x;
    this->y = y;
    this->targetSpeed = targetSpeed;
  }
  ~TargetSpeedPoint() {}
};

class Evaluation {
protected:
  ros::NodeHandle m_rosNodeHandler;

  ros::Publisher m_rosPubLimitSpeed;
  ros::Publisher m_rosPubMasterVehicleInitPoint;
  ros::Publisher m_rosPubMasterTargetSpeed;
  ros::Publisher m_rosPubLidar;

  ros::Subscriber m_rosSubVehicle_Output;
  ros::Subscriber m_rosSubRefPointArray;
  ros::Subscriber m_rosSubMasterVehicle_Output;

  //for visualization
  ros::Publisher m_rosPubPlotLimitSpeed;
  ros::Publisher m_rosPubTextLimitSpeed;
  ros::Publisher m_rosPubTextCrossTrackError;
  ros::Publisher m_rosPubPlotCrossTrackError;
  ros::Publisher m_rosPubPlotZero;

  std::string m_ref_path_param;
  std::string m_laneId_param;

  double prev_sec;

  double m_limitSpeed_ms;
  std::vector<TargetSpeedPoint> m_limitSpeedSigns;

  double m_targetSpeed_master_ms;
  geometry_msgs::Point m_masterInitPoint;
  std::vector<TargetSpeedPoint> m_targetSpeedMasterSigns;

  double m_start_time = 0.0;
  double m_prev_evaluationTimie = 0.0;
  double m_dt_evaluation_sec = 0.0;
  double m_drivingTime = 0.0;

  double m_goalX = 490.98;
  double m_goalY = 504.48;

  double m_timeGap = 1.0;
  double m_minDistance = 10.0;

public:
  Evaluation() {

    m_rosPubLimitSpeed =
        m_rosNodeHandler.advertise<std_msgs::Float64>("/limit_speed", 10);

    m_rosPubMasterVehicleInitPoint =
        m_rosNodeHandler.advertise<geometry_msgs::Point>("/master_init_point",
                                                         10);
    m_rosPubMasterTargetSpeed = m_rosNodeHandler.advertise<std_msgs::Float64>(
        "/master_target_speed", 10);
    m_rosPubLidar =
        m_rosNodeHandler.advertise<autonomous_msg::VehicleOutput>("/lidar", 10);

    m_rosSubVehicle_Output = m_rosNodeHandler.subscribe(
        "/back/vehicle_output", 10, &Evaluation::vehicleOutputCallback, this);

    m_rosSubMasterVehicle_Output = m_rosNodeHandler.subscribe(
        "/front/vehicle_output", 10, &Evaluation::masterVehicleCallback, this);

    m_rosPubPlotLimitSpeed = 
        m_rosNodeHandler.advertise<std_msgs::Float32>("/plot_limit_speed",10);

    m_rosPubTextLimitSpeed = 
        m_rosNodeHandler.advertise<jsk_rviz_plugins::OverlayText>("/text_limit_speed",10);

    m_rosPubTextCrossTrackError = 
        m_rosNodeHandler.advertise<jsk_rviz_plugins::OverlayText>("/text_crossTrack_error",10);

    m_rosPubPlotCrossTrackError = 
        m_rosNodeHandler.advertise<std_msgs::Float32>("/plot_crossTrack_error",10);

    m_rosPubPlotZero = 
        m_rosNodeHandler.advertise<std_msgs::Float32>("/plot_zero_axis",10);

    double init_x, init_y;
    m_rosNodeHandler.param("master_vehicle/init_x", init_x, 0.0);
    m_rosNodeHandler.param("master_vehicle/init_y", init_y, 0.0);

    m_rosNodeHandler.param("evaluation/refPath", m_ref_path_param,
                           std::string(""));

    m_rosNodeHandler.param("evaluation/laneId", m_laneId_param,
                           std::string(""));

    if (m_ref_path_param == std::string("")) {
      ROS_ERROR_STREAM("Empty path!!");
    }

    m_limitSpeed_ms = 100.0;
    m_limitSpeedSigns.push_back(TargetSpeedPoint(0.0, 0.0, 14.0));
    m_limitSpeedSigns.push_back(TargetSpeedPoint(188.82, -1.0841, 20.0));
    m_limitSpeedSigns.push_back(TargetSpeedPoint(159.25, 386.93, 12.0));
    m_limitSpeedSigns.push_back(TargetSpeedPoint(315.24, 416.1, 50.0));
    m_limitSpeedSigns.push_back(TargetSpeedPoint(320.55, -75.078, 17.0));
    m_limitSpeedSigns.push_back(TargetSpeedPoint(397.91, 20.55, 30.0));
    m_limitSpeedSigns.push_back(TargetSpeedPoint(541.72, 293.32, 15.0));
    m_limitSpeedSigns.push_back(TargetSpeedPoint(425.78, 413.75, 17.0));
    m_limitSpeedSigns.push_back(TargetSpeedPoint(587.51, 461.75, 12.0));


    m_targetSpeed_master_ms = 20.0;
    m_masterInitPoint.x = init_x;
    m_masterInitPoint.y = init_y;
    m_targetSpeedMasterSigns.push_back(TargetSpeedPoint(
        m_masterInitPoint.x, m_masterInitPoint.y, m_targetSpeed_master_ms));
    m_targetSpeedMasterSigns.push_back(TargetSpeedPoint(699.1, 8.2, 10.0));
    
    m_targetSpeedMasterSigns.push_back(TargetSpeedPoint(m_goalX, m_goalY, 0.0));

    m_prev_evaluationTimie = ros::Time::now().toSec();
  }

  ~Evaluation() {}

public:
  void evaluation() {
    double curr_evaluationTimie = ros::Time::now().toSec();
    m_dt_evaluation_sec = curr_evaluationTimie - m_prev_evaluationTimie;
    m_prev_evaluationTimie = curr_evaluationTimie;

    // if (isMasterCreated == false) { //앞차량(마스터)를 생성할지 검사한다.
    //   this->createMasterVehicle();
    // }

    this->checkAndSendLimitSpeedSign(); //뒷차량의 최대 주행 속도를 정한다.
    this->checkAndSendTargetSpeedMasterSign(); //앞차량의 acc 속도를 정한다.

    if (isSimulatorOn == true && isFinish == false) {
      if (isMasterCreated == true) { // acc
        this->calcSpacingError();
        m_rosPubLidar.publish(
            m_masterVehicleState); //마스터가 생성될때만 Lidar를 출력한다.
      } else {                     // speed control zone
        this->calcOverSpeedPenalty();
      }
      this->calcOffsetError();

    } else if (isSimulatorOn == false) {
      m_start_time = ros::Time::now().toSec();
    }

    // 마스터(앞차량)이 생성되면 마스터의 위치와 속도를 lidar 데이터로 출력한다.
    if (isMasterCreated == true) {
      m_rosPubLidar.publish(m_masterVehicleState);
    }

    this->measureDrivingTime();
    this->printEvaluationValue();
  }

protected:
  double m_LADSpacingError = 0.0;
  double m_currSpacingError = 0.0;
  bool m_isCollision = false;

public:
  double calcDistance(double front_x, double front_y, double back_x,
                      double back_y) {
    //거리를 구한다.
    //앞차량의 포인트 인덱스를 구한다.
    double min_distance_sq = std::numeric_limits<double>::max();
    int frontIndex = 0;
    for (int i = 0; i < m_refLane.point.size(); i++) {
      double dx = front_x - m_refLane.point[i].x;
      double dy = front_y - m_refLane.point[i].y;
      double distance_sq = dx * dx + dy * dy;

      if (distance_sq < min_distance_sq) {
        min_distance_sq = distance_sq;
        frontIndex = i;
      }
    }
    //뒷 차량의 인덱스를 구한다.
    min_distance_sq = std::numeric_limits<double>::max();
    int backIndex = 0;
    for (int i = 0; i < m_refLane.point.size(); i++) {
      double dx = back_x - m_refLane.point[i].x;
      double dy = back_y - m_refLane.point[i].y;
      double distance_sq = dx * dx + dy * dy;

      if (distance_sq < min_distance_sq) {
        min_distance_sq = distance_sq;
        backIndex = i;
      }
    }

    //인덱스 사이의 거리를 적분한다.
    if (frontIndex < backIndex) {
      frontIndex += m_refLane.point.size();
    }
    double distance = 0.0;
    for (int i = backIndex + 1; i < frontIndex; i++) {
      int back_i = (i - 1) % m_refLane.point.size();
      int front_i = i % m_refLane.point.size();
      double dx = m_refLane.point[front_i].x - m_refLane.point[back_i].x;
      double dy = m_refLane.point[front_i].y - m_refLane.point[back_i].y;
      double ds = pow(dx * dx + dy * dy, 0.5);
      distance += ds;
    }
    return distance;
  }

  void calcSpacingError() {
    if (isMasterDataExist == true) {
      double distance =
          calcDistance(m_masterVehicleState.x, m_masterVehicleState.y,
                       m_vehicleState.x, m_vehicleState.y);
      if (distance <= 2.0) {
        m_isCollision = true;
      }
      m_currSpacingError =
          distance - (m_timeGap * m_vehicleState.velocity + m_minDistance);
    } else {
      m_currSpacingError = 0.0;
    }

    double abs_SpacingError = m_currSpacingError;
    if (abs_SpacingError < 0.0) {
      abs_SpacingError *= -1.0;
    }
    abs_SpacingError -= m_masterVehicleState.velocity * 1.0 / LOOPRATE;
    if (abs_SpacingError < 0.0) {
      abs_SpacingError = 0.0;
    }

    m_LADSpacingError += m_dt_evaluation_sec * abs_SpacingError;
  }

protected:
  double m_overSpeedPenalty = 0.0;

public:
  void calcOverSpeedPenalty() {
    double gain = 5.0;

    if (m_limitSpeed_ms > 0.1) {
      if (m_vehicleState.velocity > m_limitSpeed_ms) {
        m_overSpeedPenalty += gain * m_dt_evaluation_sec *
                              (m_vehicleState.velocity - m_limitSpeed_ms) /
                              m_limitSpeed_ms;
      }
    }
  }

protected:
  double m_maxOffsetError = 0.0;
  double m_currOffsetError = 0.0;
  double m_LADOffsetError = 0.0;

public:
  void calcOffsetError() {
    double min1_distance_sq = std::numeric_limits<double>::max();
    double min2_distance_sq = std::numeric_limits<double>::max();
    geometry_msgs::Point min1_point;
    geometry_msgs::Point min2_point;
    for (auto i = 0; i < m_refLane.point.size(); i++) {
      double dx = m_vehicleState.x - m_refLane.point[i].x;
      double dy = m_vehicleState.y - m_refLane.point[i].y;
      double distance_sq = dx * dx + dy * dy;

      if (distance_sq < min1_distance_sq) {
        min2_distance_sq = min1_distance_sq;
        min2_point = min1_point;

        min1_distance_sq = distance_sq;
        min1_point = m_refLane.point[i];
      } else if (distance_sq < min2_distance_sq) {
        min2_distance_sq = distance_sq;
        min2_point = m_refLane.point[i];
      }
    }

    double a = (min2_point.y - min1_point.y) / (min2_point.x - min1_point.x);
    double b = -1.0;
    double c = -1.0 * a * min2_point.x + min2_point.y;

    m_currOffsetError = (a * m_vehicleState.x + b * m_vehicleState.y + c) /
                        (pow(a * a + b * b, 0.5));
    text_crossTrackError();
    std_msgs::Float32 crossTrackError;
    std_msgs::Float32 zero;
    crossTrackError.data = m_currOffsetError;
    zero.data = 0;
    m_rosPubPlotCrossTrackError.publish(crossTrackError);
    m_rosPubPlotZero.publish(zero);

    if (m_currOffsetError < 0.0) {
      m_currOffsetError *= -1.0;
    }
    if (m_currOffsetError > m_maxOffsetError) {
      m_maxOffsetError = m_currOffsetError;
      // ROS_INFO_STREAM("\n Max Offset Error: " << m_maxOffsetError);
    }
    m_LADOffsetError += m_dt_evaluation_sec * m_currOffsetError;
    
  }

  void text_crossTrackError(){
    jsk_rviz_plugins::OverlayText modeText;
    double backGround_rgb[] = {0.0f,0.0f,0.0f,0.0f};
    double text_rgb[] = {0.9f,0.9f,0.9f,0.7f};
    //Bounding Box
    modeText.width = 128;
    modeText.height = 128;
    modeText.left = 128;
    modeText.top = 128;
    modeText.bg_color.r = backGround_rgb[0];
    modeText.bg_color.g = backGround_rgb[1];
    modeText.bg_color.b = backGround_rgb[2];
    modeText.bg_color.a = backGround_rgb[3];
    //Text
    modeText.line_width = 1;
    modeText.text_size = 9.0;
    modeText.fg_color.r = text_rgb[0];
    modeText.fg_color.r = text_rgb[1];
    modeText.fg_color.r = text_rgb[2];
    modeText.fg_color.r = text_rgb[3];

    modeText.text = 
        " Cross Track Error : \n" + std::to_string(m_currOffsetError) + " \n LAD Error : \n" + std::to_string(m_LADOffsetError);

    m_rosPubTextCrossTrackError.publish(modeText);
  }


  void checkAndSendLimitSpeedSign() {
    for (auto i = 0; i < m_limitSpeedSigns.size(); i++) {
      double dx = m_vehicleState.x - m_limitSpeedSigns[i].x;
      double dy = m_vehicleState.y - m_limitSpeedSigns[i].y;

      double distance_sq = dx * dx + dy * dy;
      if (distance_sq <= 5.0 * 5.0) {
        m_limitSpeed_ms = m_limitSpeedSigns[i].targetSpeed;
        // ROS_INFO_STREAM("m_limitSpeed_ms: " << m_limitSpeed_ms);
      }
    }
    std_msgs::Float64 std_double_data;
    std_msgs::Float32 plotLimitVel;
    std_double_data.data = m_limitSpeed_ms;
    plotLimitVel.data = m_limitSpeed_ms;
    m_rosPubLimitSpeed.publish(std_double_data);
    m_rosPubPlotLimitSpeed.publish(plotLimitVel);
    text_limit();
  }

  void text_limit(){
    jsk_rviz_plugins::OverlayText modeText;
    double backGround_rgb[] = {0.0f,0.0f,0.0f,0.0f};
    double text_rgb[] = {0.9f,0.9f,0.9f,0.7f};
    //Bounding Box
    modeText.width = 128;
    modeText.height = 128;
    modeText.left = 128;
    modeText.top = 128;
    modeText.bg_color.r = backGround_rgb[0];
    modeText.bg_color.g = backGround_rgb[1];
    modeText.bg_color.b = backGround_rgb[2];
    modeText.bg_color.a = backGround_rgb[3];
    //Text
    modeText.line_width = 1;
    modeText.text_size = 9.0;
    modeText.fg_color.r = text_rgb[0];
    modeText.fg_color.r = text_rgb[1];
    modeText.fg_color.r = text_rgb[2];
    modeText.fg_color.r = text_rgb[3];

    modeText.text = " Limit Velocity : \n" + std::to_string(m_limitSpeed_ms);

    m_rosPubTextLimitSpeed.publish(modeText);
  }

protected:
  bool isMasterCreated = false;

public:
  // void createMasterVehicle() {

  //   double distance = calcDistance(m_masterInitPoint.x, m_masterInitPoint.y,
  //                                  m_vehicleState.x, m_vehicleState.y);
  //   double spacing_Error =
  //       distance - (m_timeGap * m_vehicleState.velocity + m_minDistance);

  //   if (spacing_Error < 0.0) {
  //     spacing_Error *= -1.0;
  //   }

  //   if (spacing_Error < 5.0) {
  //     isMasterCreated = true;
  //     m_rosPubMasterVehicleInitPoint.publish(m_masterInitPoint);
  //   }
  // }
  void checkAndSendTargetSpeedMasterSign() {
    for (auto i = 0; i < m_targetSpeedMasterSigns.size(); i++) {
      double dx = m_masterVehicleState.x - m_targetSpeedMasterSigns[i].x;
      double dy = m_masterVehicleState.y - m_targetSpeedMasterSigns[i].y;

      double distance_sq = dx * dx + dy * dy;
      if (distance_sq <= 5.0 * 5.0) {
        m_targetSpeed_master_ms = m_targetSpeedMasterSigns[i].targetSpeed;
      }
    }
    std_msgs::Float64 targetSpeed;
    targetSpeed.data = m_targetSpeed_master_ms;

    m_rosPubMasterTargetSpeed.publish(targetSpeed);
  }

protected:
  autonomous_msg::VehicleOutput m_vehicleState;
  autonomous_msg::VehicleOutput m_masterVehicleState;
  bool isSimulatorOn = false;
  bool isMasterDataExist = false;

public:
  void
  vehicleOutputCallback(const autonomous_msg::VehicleOutput::ConstPtr &msg) {
    m_vehicleState = *msg;
    isSimulatorOn = true;
  }
  void
  masterVehicleCallback(const autonomous_msg::VehicleOutput::ConstPtr &msg) {
    m_masterVehicleState = *msg;
    isMasterDataExist = true;
  }

protected:
  bool isFinish = false;

public:
  void measureDrivingTime() {

    double dx = m_vehicleState.x - m_goalX;
    double dy = m_vehicleState.y - m_goalY;
    double distance_sq = dx * dx + dy * dy;

    if (distance_sq <= 5.0 * 5.0) {
      isFinish = true;
    }

    if (isFinish == false) {
      m_drivingTime = ros::Time::now().toSec() - m_start_time;
    }
  }

protected:
  autonomous_msg::LanePointData m_refLane;

public:
  void loadLaneData() {

    SKusvLanes csvRefLaneImport;
    csvRefLaneImport.ImportKusvLaneCsvFile(m_ref_path_param);

    m_refLane.frame_id = "/world";
    m_refLane.point.clear();
    m_refLane.id = m_laneId_param;

    for (auto i_lane = 0; i_lane < csvRefLaneImport.m_vecKusvLanes.size();
         i_lane++) {

      if (m_laneId_param ==
          std::to_string(csvRefLaneImport.m_vecKusvLanes[i_lane].m_nLaneID)) {

        for (auto i_point = 0;
             i_point <
             csvRefLaneImport.m_vecKusvLanes[i_lane].m_vecKusvLanePoint.size();
             i_point++) {
          geometry_msgs::Point point;
          point.x = csvRefLaneImport.m_vecKusvLanes[i_lane]
                        .m_vecKusvLanePoint[i_point]
                        .m_dPtX_m;
          point.y = csvRefLaneImport.m_vecKusvLanes[i_lane]
                        .m_vecKusvLanePoint[i_point]
                        .m_dPtY_m;
          m_refLane.point.push_back(point);
        }
      }
    }
  }

  void printEvaluationValue() {
    ROS_INFO_STREAM(
        "\n"
        << "1. limit Speed: " << m_limitSpeed_ms << " m/s"
        << "\n"
        << "   Vehicle Speed: " << m_vehicleState.velocity << " m/s"
        << "\n"
        << "   ------------------------------------------------"
        << "\n"
        << "   Over Speed Penalty: " << m_overSpeedPenalty << " sec"
        << "\n"
        << "\n"
        << "==================================================="
        << "\n"
        << "2. Offset Error: " << m_currOffsetError << " m"
        << "\n"
        << "   LAD Offset Error: " << m_LADOffsetError << " m"
        << "\n"
        << "   ------------------------------------------------"
        << "\n"
        << "   Max Offset Error: " << m_maxOffsetError << " m"
        << "\n"
        << "\n"
        << "==================================================="
        << "\n"
        // << "3. Spacing Error: " << m_currSpacingError << " m"
        // << "\n"
        // << "   LAD Spacing Error: " << m_LADSpacingError << " m"
        // << "\n"
        // << "   Collision: " << (m_isCollision ? "TRUE" : "FALSE") << "\n"
        // << "\n"
        // << "==================================================="
        // << "\n"
        << "3. Driving Time: " << m_drivingTime << "\n"
        << "\n"
        << "------------------- [Result] ----------------------"
        << "\n"
        << "Time Score:\t" << (m_drivingTime + m_overSpeedPenalty) << "\n"
        << "LK Score:\t" << (m_maxOffsetError * 5.0 + m_LADOffsetError) 
        //<< "\n" << "ACC Score:\t" << (m_LADSpacingError)
        );
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "evaluation");
  Evaluation evaluation;
  evaluation.loadLaneData();

  ros::Rate loop_rate(LOOPRATE);
  while (ros::ok()) {
    evaluation.evaluation();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}