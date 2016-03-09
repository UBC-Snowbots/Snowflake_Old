/*
* Header File for GPS Driver (GPS_Driver.cpp) 
* Author : Vincent Yuan 
*/ 

#include <stdlib.h>
#include <iostream> 
#include <sstream> 
#include <string> 
#include <ros/ros.h> 
#include "drivers/GPS.msg"
#include "SerialCommunication.h"

using namespace std; 

//constant declerations
static const string ROS_NODE_NAME = "gps_driver"; 
static const int ROS_LOOP_RATE = 10; 
static const int BAUD_RATE = 115200; 
static const string SENSOR_OUTPUT_TOPIC = "GPS" 
static const string ARDUINO_PORT_NAME = "/dev/ttyACM"; 

//Objects 
SerialCommunication link_port; 
sensor_msgs::Imu IMU; 

string to_string(int i); 
void Serial_Store(char *buffer); 
void data_request(char c); 
