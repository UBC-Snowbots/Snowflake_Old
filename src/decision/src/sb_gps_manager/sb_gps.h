/*
* Header File for GPS waypoint manager (sb_gps.cpp)
* Author: Vincent Yuan 
*/ 
 
#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <string>
#include <ros/ros.h> 
#include <std_msgs/String.h> 
#include "messages/gps.h"
#include "decision/gps_waypoint.h"
using namespace std;

struct waypoint{
  double lon; 
  double lat; 
  waypoint(){
  lon = lat = 0; 
  }
  waypoint(double i, double y){
  lon = i;
  lat = y; 
  }
};

waypoint CurrentWaypoint, LastWaypoint, nextWaypoint;
/* CurrentWaypoint holds current waypoint in lon lat
*  LastWaypoint holds last waypoint in lon lat
   nextWaypoint holds nextWaypoint in meters dx dy */ 

static const string ROS_NODE_NAME = "gps_manager"; 
static const int ROS_LOOP_RATE = 200; //Hz
static const string INPUT_TOPIC = "GPS"; 
static const string SERVICE_NAME = "gps_waypoint_manager";

bool waypointHandle(decision::gps_waypoint::Request &req, decision::gps_waypoint::Response &res);

void gpsSubHandle(const std_msgs::String::ConstPtr& msg); 
