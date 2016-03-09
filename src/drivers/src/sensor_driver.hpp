/*
* Header File for Driver Node (Sensor_Driver.cpp)
* Author: Vincent Yuan
*/
#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <string>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include "SerialCommunication.h"
#include <sensor_msgs/Imu.h>

using namespace std; 

//Constant declerations 
static const string ROS_NODE_NAME = "sensor_driver";
static const int ROS_LOOP_RATE = 10; //units of Hz, default should be 200
//Loop rate slowed due to lag in serial, can try increasing 
static const int BAUD_RATE = 115200; 
static const string SENSOR_OUTPUT_TOPIC = "IMU"; 
static const string ARDUINO_PORT_NAME = "/dev/ttyACM";
static const double ARDUINO_ANALOG_PU = 0.0049; //v/pu Arduino analog to voltage conversion
static const double ADXL_V_G_PU = 0.3; //Units: Volts/g 
static const double G_TO_ACCEL = 9.80665; //Units: 9.8(m/s^2)/g 

//Object Declerations 
SerialCommunication link_port;
sensor_msgs::Imu IMU; 

std::string to_string(int i); 
void IMU_write(int c, double val);
bool Serial_Store(char *buffer, int sensor);
void data_request(char c); 
