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
#include "messages/gps.h"
#include "SerialCommunication.h" 

//constant declerations
static const string ROS_NODE_NAME = "gps_driver"; 
static const int ROS_LOOP_RATE = 10; 
static const int BAUD_RATE = 115200; 
static const string SENSOR_OUTPUT_TOPIC = "GPS"; 
static const string ARDUINO_PORT_NAME = "/dev/ttyACM"; 

::messages::gps gps_msg; 
//Objects 
SerialCommunication link_port; 
//messages::gps GPS; 
std::string msg; 
std::string to_string(int i); 
std::string to_string2(char* c);
bool gps_store(char *buffer); 
void data_request(char c, char *buffer);
bool connect_device(std::string device_name); 
bool open_port(unsigned int count);
