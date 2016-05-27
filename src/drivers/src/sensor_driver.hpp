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
#include <nav_msgs/Odometry.h>

#define DATA 0 

using namespace std; 

//Constant declerations 
static const string ROS_NODE_NAME = "sensor_driver";
static const int ROS_LOOP_RATE = 200; //units of Hz, default should be 200
//Loop rate slowed due to lag in serial, can try increasing 
static const int BAUD_RATE = 115200; 
static const string IMU_TOPIC = "IMU"; 
static const string ODOM_TOPIC = "ODOM"; 
static const string ARDUINO_PORT_NAME = "/dev/ttyACM";
static const double ARDUINO_ANALOG_PU = 0.0049; //v/pu Arduino analog to voltage conversion
static const double ADXL_V_G_PU = 0.3; //Units: Volts/g 
static const double G_TO_ACCEL = 9.80665; //Units: 9.8(m/s^2)/g 

//Object Declerations 
SerialCommunication link_port;
sensor_msgs::Imu IMU;
nav_msgs::Odometry odom; 
std::string to_string2(char* c);
std::string to_string(int i); 
bool connect_device(std::string device_name);
bool open_port(unsigned int count);
void IMU_write(int c, double val);
bool Serial_Store(char *buffer);
void data_request(char c, char *buffer, int mode); 
void msg_store(int type, char *data);
