/*
* Header File for GPS Driver (GPS_Driver.cpp) 
* Author : Vincent Yuan 
*/ 

#include <stdlib.h>
#include <iostream> 
#include <sstream> 
#include <string> 
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h> 
#include "sb_messages/gps.h"
#include "SerialCommunication.h" 

#define DATA 0
//constant declerations
static const string ROS_NODE_NAME = "gps_driver"; 
static const int ROS_LOOP_RATE = 1; 
static const int BAUD_RATE = 115200; 
static const string SENSOR_OUTPUT_TOPIC = "GPS"; 
static const string ARDUINO_PORT_NAME = "/dev/ttyACM"; 

struct gps_comp_data {
  float latitude, longitude;
  int fix;
  float x, y, z, headingDegrees;
} gps_comp_data;

sb_messages::gps gps_msg; 

//Objects 
SerialCommunication link_port; 
//messages::gps GPS;
std::string msg; 
std::string to_string(int i); 
std::string to_string2(char* c);
bool gps_store(char *buffer); 
void data_request(char c, char *buffer, int mode);
bool connect_device(std::string device_name); 
bool open_port(unsigned int count);
void gps_msg_create(void);
bool read_bit (char *c);
