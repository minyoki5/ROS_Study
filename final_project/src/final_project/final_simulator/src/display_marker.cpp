#include "autonomous_msg/LanePointDataArray.h"
#include "autonomous_msg/PolyfitLaneDataArray.h"
#include "autonomous_msg/VehicleOutput.h"
#include "autonomous_msg/EnvironmentMode.h"
#include "autonomous_msg/EnvironmentModeArea.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "jsk_rviz_plugins/OverlayText.h"
#include "std_msgs/ColorRGBA.h"
#include "std_msgs/Float32.h"
#include <math.h>
#include <ros/ros.h>
#include <string>
#include <tf/tf.h>

#define PI 3.1415926579

#define PLAIN 0
#define UP_HILL 1
#define DOWN_HILL 2

class Display {
protected:
  ros::NodeHandle m_rosNodeHandler;

  ros::Publisher m_rosPubVehiclesMarker;
  ros::Subscriber m_rosSubVehicle_Output;

  ros::Publisher m_rosPubLanesMarkerArray;
  ros::Subscriber m_rosSubLanePointArray;

  ros::Publisher m_rosPubPolyMarkerArray;
  ros::Subscriber m_rosSubPolyLaneArray;

  ros::Publisher m_rosPubDrivingWay;
  ros::Subscriber m_rosSubDrivingWay;

  ros::Publisher m_rosPubCsvMarkerArray;
  ros::Subscriber m_rosSubCsvLaneArray;

  ros::Publisher m_rosPubEnvirnmentArea;
  ros::Subscriber m_rosSubEnvirnmentArea;

  ros::Publisher m_rosPubEnvironmentMode;
  ros::Subscriber m_rosSubEnvironmentMode;

  ros::Publisher m_rosPubVehicleVelocity;
  ros::Publisher m_rosPubPlotVehicleVelocity;

public:
  Display() {
    m_rosPubVehiclesMarker =
        m_rosNodeHandler.advertise<visualization_msgs::Marker>("vehicle_marker",
                                                               10);
    m_rosSubVehicle_Output = m_rosNodeHandler.subscribe(
        "vehicle_output", 10, &Display::vehicleOutputCallback, this);

    m_rosPubCsvMarkerArray =
        m_rosNodeHandler.advertise<visualization_msgs::MarkerArray>(
            "csv_lanes_marker", 10);
    m_rosSubCsvLaneArray = m_rosNodeHandler.subscribe(
        "csv_lanes", 10, &Display::csvLanesCallback, this);

    m_rosPubLanesMarkerArray =
        m_rosNodeHandler.advertise<visualization_msgs::MarkerArray>(
            "ROI_lanes_marker", 10);
    m_rosSubLanePointArray = m_rosNodeHandler.subscribe(
        "ROI_lanes", 10, &Display::lanesCallback, this);

    m_rosPubPolyMarkerArray =
        m_rosNodeHandler.advertise<visualization_msgs::MarkerArray>(
            "polyfit_lanes_marker", 10);
    m_rosSubPolyLaneArray = m_rosNodeHandler.subscribe(
        "polyfit_lanes", 10, &Display::polyLanesCallback, this);

    m_rosPubDrivingWay =
        m_rosNodeHandler.advertise<visualization_msgs::MarkerArray>(
            "driving_way_marker", 10);
    m_rosSubDrivingWay = m_rosNodeHandler.subscribe(
        "driving_way", 10, &Display::drivingWayCallback, this);

    m_rosPubEnvirnmentArea =
        m_rosNodeHandler.advertise<visualization_msgs::MarkerArray>(
            "environment_area_marker", 10);
    m_rosSubEnvirnmentArea = m_rosNodeHandler.subscribe(
        "/environment_area", 10, &Display::environmentAreaCallback, this);

    m_rosPubEnvironmentMode = m_rosNodeHandler.advertise<jsk_rviz_plugins::OverlayText>(
          "jsk_environment_Mode",1000);

    m_rosSubEnvironmentMode = m_rosNodeHandler.subscribe(
          "/environment_mode", 10, &Display::environmentModeCallback, this);

    m_rosPubVehicleVelocity = m_rosNodeHandler.advertise<std_msgs::Float32>(
          "/ego_vehicle_velocity",10);
    
    m_rosPubPlotVehicleVelocity = m_rosNodeHandler.advertise<jsk_rviz_plugins::OverlayText>(
          "/text_velocity",10);
  }

  ~Display() {}

protected:
  autonomous_msg::LanePointDataArray m_csvLanes;

public:
  void
  csvLanesCallback(const autonomous_msg::LanePointDataArray::ConstPtr &msg) {
    // std::string id = msg->id;
    m_csvLanes.frame_id = msg->frame_id;
    m_csvLanes.id = msg->id;
    m_csvLanes.lane = msg->lane;
  }

  // Map Lane visualization
  void mark_csvLanes() {

    visualization_msgs::MarkerArray markerArray;
    visualization_msgs::MarkerArray hillMarkerArray;

    int id = 0;
    double marker_rgb[] = {0.9f,0.9f,0.9f,0.7f};

    for (auto i_lane = 0; i_lane < m_csvLanes.lane.size(); i_lane++) {
      visualization_msgs::Marker marker;
      visualization_msgs::Marker hillMarker;

      marker.header.frame_id = m_csvLanes.frame_id;
      marker.header.stamp = ros::Time::now();
      marker.ns = m_csvLanes.lane[i_lane].id;
      marker.id = id++;

      hillMarker.header.frame_id = m_csvLanes.frame_id;
      hillMarker.header.stamp = ros::Time::now();
      hillMarker.ns = m_csvLanes.lane[i_lane].id;
      hillMarker.id = id++;

      marker.type = visualization_msgs::Marker::LINE_STRIP;
      marker.action = visualization_msgs::Marker::ADD;

      hillMarker.type = visualization_msgs::Marker::LINE_STRIP;
      hillMarker.action = visualization_msgs::Marker::ADD;

      marker.color.r = marker_rgb[0];
      marker.color.g = marker_rgb[1];
      marker.color.b = marker_rgb[2];
      marker.color.a = 0.7;
      marker.scale.x = 0.2;
      marker.lifetime = ros::Duration();
      
      geometry_msgs::Point prevPoint;
      int hillMode = PLAIN;
      bool first = true;
      for (auto i_point = 0; i_point < m_csvLanes.lane[i_lane].point.size(); i_point++) {
        geometry_msgs::Point currPoint;
        currPoint.x = m_csvLanes.lane[i_lane].point[i_point].x;
        currPoint.y = m_csvLanes.lane[i_lane].point[i_point].y;
        currPoint.z = 0.0;

        for (auto i = 0; i < m_environmentArea.hill_x.size(); i++) {
          double dx = currPoint.x - m_environmentArea.hill_x[i];
          double dy = currPoint.y - m_environmentArea.hill_y[i];
          double distance_sq = dx * dx + dy * dy;
          double slopeAngle;
          if (distance_sq <= 10.0 * 10.0) {
            slopeAngle = m_environmentArea.hill_slope_angle[i];
            if(slopeAngle > 0.0){
              hillMode = UP_HILL;
              break;
            }else if(slopeAngle < 0.0){
              hillMode = DOWN_HILL;
              break;
            }else{
              hillMode = PLAIN;
            }     
          }
        }

        std_msgs::ColorRGBA color;
        if(hillMode == UP_HILL){
          color.r = 1.0f;
          color.g = 0.11f;
          color.b = 0.11f;
          color.a = 0.7 ;
        }
        else if(hillMode == DOWN_HILL){
          color.r = 0.11f;
          color.g = 0.11f;
          color.b = 1.0f;
          color.a = 0.7 ;
        }
        else{
          color.r = 0.9f;
          color.g = 0.9f;
          color.b = 0.9f;
          color.a = 0.7;
        }
        



        if (first == true) {
          first = false;
        } else {
          double dx = currPoint.x - prevPoint.x;
          double dy = currPoint.y - prevPoint.y;
          if ((dx * dx + dy * dy) <= 2.0 * 2.0) {
            marker.points.push_back(prevPoint);
            marker.points.push_back(currPoint);
            marker.colors.push_back(color);
            marker.colors.push_back(color);
            }
            
          else {
            markerArray.markers.push_back(marker);
            marker.points.clear();
            marker.colors.clear();
            marker.id = id++;
          }
        }
        prevPoint = currPoint;
      }
      markerArray.markers.push_back(marker);
    }
    m_rosPubCsvMarkerArray.publish(markerArray);
  }

protected:
  std::string m_sVehicle_id = "";
  double m_dVehicleX = 0.0;
  double m_dVehicleY = 0.0;
  double m_dVehicleYaw = 0.0;
  double m_dVehicleVel = 0.0;
  bool m_isVehicleExist = false;

public:
  void
  vehicleOutputCallback(const autonomous_msg::VehicleOutput::ConstPtr &msg) {
    m_sVehicle_id = msg->id;
    m_dVehicleX = msg->x;
    m_dVehicleY = msg->y;
    m_dVehicleYaw = msg->yaw;
    m_dVehicleVel = msg->velocity;
    std_msgs::Float32 vehicleVelocity;
    vehicleVelocity.data = m_dVehicleVel;
    m_rosPubVehicleVelocity.publish(vehicleVelocity);
    text_vehicle();
    m_isVehicleExist = true;
  }

  void text_vehicle(){
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

    modeText.text = "Vehicle Velocity : \n" + std::to_string(m_dVehicleVel);

    m_rosPubPlotVehicleVelocity.publish(modeText);
  }

  void mark_vehicle() {
    if (m_isVehicleExist == true) {
      tf::Quaternion q_temp;
      tf::Matrix3x3 m(q_temp);
      q_temp.setRPY(0.0 / 180.0 * PI, 0, 90.0 / 180.0 * PI + m_dVehicleYaw);
      tf::Quaternion q(q_temp.getX(), q_temp.getY(), q_temp.getZ(),
                       q_temp.getW());

      visualization_msgs::Marker vehicle_marker;
      vehicle_marker.header.frame_id = "/world";
      vehicle_marker.header.stamp = ros::Time::now();
      vehicle_marker.ns = m_sVehicle_id;
      vehicle_marker.id = 0;
      // Set the marker type
      vehicle_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
      vehicle_marker.mesh_resource =
          "package://final_simulator/meshes/evoque_new.dae";
          //"package://final_simulator/meshes/BMW_X5_4.dae";
      vehicle_marker.mesh_use_embedded_materials = false;

      vehicle_marker.pose.position.x = m_dVehicleX;
      vehicle_marker.pose.position.y = m_dVehicleY;
      vehicle_marker.pose.position.z = 0.0;
      vehicle_marker.pose.orientation.x = q.getX();
      vehicle_marker.pose.orientation.y = q.getY();
      vehicle_marker.pose.orientation.z = q.getZ();
      vehicle_marker.pose.orientation.w = q.getW();
      // Set the scale of the marker
      vehicle_marker.scale.x = 1.0;
      vehicle_marker.scale.y = 1.0;
      vehicle_marker.scale.z = 1.0;

      vehicle_marker.color.r = 1.0;
      vehicle_marker.color.g = 1.0;
      vehicle_marker.color.b = 1.0;
      vehicle_marker.color.a = 1.0;

      vehicle_marker.lifetime = ros::Duration(0.1);

      m_rosPubVehiclesMarker.publish(vehicle_marker);
    }
  }

protected:
  autonomous_msg::LanePointDataArray m_lanes;

public:
  void lanesCallback(const autonomous_msg::LanePointDataArray::ConstPtr &msg) {
    // std::string id = msg->id;
    m_lanes.frame_id = msg->frame_id;
    m_lanes.id = msg->id;
    m_lanes.lane = msg->lane;
  }

  void mark_ROILanes() {

    visualization_msgs::MarkerArray markerArray;
    int id = 0;
    for (auto i_lane = 0; i_lane < m_lanes.lane.size(); i_lane++) {
      visualization_msgs::Marker marker;
      marker.header.frame_id = m_lanes.lane[i_lane].frame_id;
      marker.header.stamp = ros::Time::now();

      marker.ns = m_lanes.lane[i_lane].id;
      marker.id = id++;

      marker.type = visualization_msgs::Marker::LINE_STRIP;
      marker.action = visualization_msgs::Marker::ADD;

      marker.color.r = 1.0f;
      marker.color.g = 0.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0;
      marker.scale.x = 0.25;
      marker.lifetime = ros::Duration(0.2);

      geometry_msgs::Point prevPoint;
      bool first = true;
      for (auto i_point = 0; i_point < m_lanes.lane[i_lane].point.size();
           i_point++) {
        geometry_msgs::Point currPoint;
        currPoint.x = m_lanes.lane[i_lane].point[i_point].x;
        currPoint.y = m_lanes.lane[i_lane].point[i_point].y;
        currPoint.z = 0.0;

        if (first == true) {
          first = false;
        } else {
          double dx = currPoint.x - prevPoint.x;
          double dy = currPoint.y - prevPoint.y;
          if ((dx * dx + dy * dy) <= 2.0 * 2.0) {
            marker.points.push_back(prevPoint);
            marker.points.push_back(currPoint);
            markerArray.markers.push_back(marker);
          } else {
            markerArray.markers.push_back(marker);
            marker.points.clear();
            marker.id = id++;
          }
        }
        prevPoint = currPoint;
      }
      markerArray.markers.push_back(marker);
    }
    m_rosPubLanesMarkerArray.publish(markerArray);
  }

protected:
  autonomous_msg::PolyfitLaneDataArray m_polyLanes;

public:
  void
  polyLanesCallback(const autonomous_msg::PolyfitLaneDataArray::ConstPtr &msg) {
    // std::string id = msg->id;
    m_polyLanes.frame_id = msg->frame_id;
    m_polyLanes.polyfitLanes = msg->polyfitLanes;
  }

  void mark_ROIpolyLanes(double interval = 0.1, double ROILength = 30.0) {

    visualization_msgs::MarkerArray markerArray;
    for (auto i_lane = 0; i_lane < m_polyLanes.polyfitLanes.size(); i_lane++) {

      double x = 0.0;
      double y = m_polyLanes.polyfitLanes[i_lane].a0;

      double distance_square = x * x + y * y;
      ;
      int id = 0;
      while (distance_square < ROILength * ROILength) {
        double a0 = m_polyLanes.polyfitLanes[i_lane].a0;
        double a1 = m_polyLanes.polyfitLanes[i_lane].a1;
        double a2 = m_polyLanes.polyfitLanes[i_lane].a2;
        double a3 = m_polyLanes.polyfitLanes[i_lane].a3;

        y = a0 + a1 * x + a2 * x * x + a3 * x * x * x;
        distance_square = x * x + y * y;

        visualization_msgs::Marker marker;
        marker.header.frame_id = m_polyLanes.polyfitLanes[i_lane].frame_id;
        marker.header.stamp = ros::Time::now();

        marker.ns = m_polyLanes.polyfitLanes[i_lane].id;
        marker.id = id;

        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.1;
        marker.pose.orientation.w = 1.0;
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.lifetime = ros::Duration(0.2);

        markerArray.markers.push_back(marker);
        x += interval;
        id++;
      }
    }
    m_rosPubPolyMarkerArray.publish(markerArray);
  }

protected:
  autonomous_msg::PolyfitLaneData m_drivingWay;

public:
  void
  drivingWayCallback(const autonomous_msg::PolyfitLaneData::ConstPtr &msg) {
    m_drivingWay = *msg;
  }

  void mark_drivingWay(double interval = 0.1, double ROILength = 30.0) {

    double a0 = m_drivingWay.a0;
    double a1 = m_drivingWay.a1;
    double a2 = m_drivingWay.a2;
    double a3 = m_drivingWay.a3;

    double x = 0.0;
    double y = a0;

    double distance_square = x * x + y * y;
    int id = 0;
    visualization_msgs::MarkerArray markerArray;
    while (distance_square < ROILength * ROILength) {

      y = a0 + a1 * x + a2 * x * x + a3 * x * x * x;
      distance_square = x * x + y * y;

      visualization_msgs::Marker marker;
      marker.header.frame_id = m_drivingWay.frame_id;
      marker.header.stamp = ros::Time::now();

      marker.ns = m_drivingWay.id;
      marker.id = id;

      marker.type = visualization_msgs::Marker::CYLINDER;
      marker.action = visualization_msgs::Marker::ADD;

      marker.pose.position.x = x;
      marker.pose.position.y = y;
      marker.pose.position.z = 0.0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.1;
      marker.pose.orientation.w = 1.0;
      marker.color.r = 1.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0;
      marker.scale.x = 0.1;
      marker.scale.y = 0.1;
      marker.scale.z = 0.1;
      marker.lifetime = ros::Duration(0.2);

      markerArray.markers.push_back(marker);
      x += interval;
      id++;
    }

    m_rosPubDrivingWay.publish(markerArray);
  }

protected:
  autonomous_msg::EnvironmentModeArea m_environmentArea;
  bool isEnvironmentCallback = false;

public:
  void environmentAreaCallback(const autonomous_msg::EnvironmentModeArea::ConstPtr &msg){
    m_environmentArea = *msg;
    isEnvironmentCallback = true;
  }

  void mark_environmentArea(){
    visualization_msgs::MarkerArray markerArray;

    for(int i_ice=0; i_ice < m_environmentArea.ice_x.size(); i_ice++){
      visualization_msgs::Marker marker;

      marker.header.frame_id = "/world";
      marker.header.stamp = ros::Time::now();

      marker.ns = "ice";
      marker.id = i_ice;

      marker.type = visualization_msgs::Marker::CYLINDER;
      marker.action = visualization_msgs::Marker::ADD;

      marker.color.r = 0.684f;
      marker.color.g = 0.866f;
      marker.color.b = 0.940f;
      marker.color.a = 0.7;
      marker.scale.x = m_environmentArea.ice_radius[i_ice];
      marker.scale.y = m_environmentArea.ice_radius[i_ice];
      marker.scale.z = 0.05;
      marker.lifetime = ros::Duration();
      marker.pose.position.x = m_environmentArea.ice_x[i_ice];
      marker.pose.position.y = m_environmentArea.ice_y[i_ice];
      marker.pose.position.z = -0.02;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.1;
      marker.pose.orientation.w = 1.0;

      markerArray.markers.push_back(marker);
    }

    m_rosPubEnvirnmentArea.publish(markerArray);
  }

  protected:
    autonomous_msg::EnvironmentMode m_environmentMode;

  public:
    void environmentModeCallback(const autonomous_msg::EnvironmentMode::ConstPtr &msg) {
      m_environmentMode = *msg;
      mark_environmentMode();
    }

    void mark_environmentMode(){
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

      if(m_environmentMode.ice_mode == "Asphalt"){
        if(m_environmentMode.hill_mode == "Plain"){
          modeText.text = "Ice Mode : Asphalt \n Hill Mode :  Plain ";
        }else if(m_environmentMode.hill_mode == "Up Hill"){
          modeText.text = "Ice Mode : Asphalt \n Hill Mode :  Up Hill ";
        }else{
          modeText.text = "Ice Mode : Asphalt \n Hill Mode : Down Hill";
        }
      }else{
        if(m_environmentMode.hill_mode == "Plain"){
          modeText.text = "Ice Mode : Ice \n Hill Mode :  Plain ";
        }else if(m_environmentMode.hill_mode == "Up Hill"){
          modeText.text = "Ice Mode : Ice \n Hill Mode :  Up Hill ";
        }else{
          modeText.text = "Ice Mode : Ice \n Hill Mode : Down Hill";
        }
      } 
      m_rosPubEnvironmentMode.publish(modeText);
    }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "display");
  // Vehicle vehicle;
  Display display;

  double prev_vehiclesMarkTime = ros::Time::now().toSec();
  double prev_lanesMarkTime = ros::Time::now().toSec();
  double prev_csvlanesMarkTime = ros::Time::now().toSec();
  // The approximate control time is 100 Hz
  ros::Rate loop_rate(100);
  while (ros::ok()) {
    
    if ((ros::Time::now().toSec() - prev_vehiclesMarkTime) > 0.1) {
      prev_vehiclesMarkTime = ros::Time::now().toSec();
      display.mark_vehicle();
    }
    if ((ros::Time::now().toSec() - prev_lanesMarkTime) > 0.2) {
      prev_lanesMarkTime = ros::Time::now().toSec();
      display.mark_ROILanes();
      display.mark_ROIpolyLanes();
      display.mark_drivingWay();
      display.mark_environmentMode();
    }
    if ((ros::Time::now().toSec() - prev_csvlanesMarkTime) > 1.0) {
      prev_csvlanesMarkTime = ros::Time::now().toSec();
      display.mark_csvLanes();
      display.mark_environmentArea();
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
