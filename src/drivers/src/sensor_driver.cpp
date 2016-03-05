/*
* Driver Node for arduino sensors: (Accelerometer, Gyroscope, Compass, GPS)
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

static const string SENSOR_NODE_NAME = "sensor_driver";
static const int ROS_LOOP_RATE = 200; //units of Hz
static const int BAUD_RATE = 9200; 
static const string SENSOR_OUTPUT_TOPIC = "IMU" 

int main (int argc, char** argv){
	ros::init(argc,argv, ROS_NODE_NAME);
	ros::NodeHandle nh; 
	ros::Rate loop_rate(ROS_LOOP_RATE);
  ros::Publisher sensor_imu_pubisher = nh.advertise<sensor_msgs::Imu>(SENSOR_OUTPUT_TOPIC,20);
  
  SerialCommunication link;

  stringstream ss;
  ss << 0;
  while(!link.connect(BAUD_RATE,(ARDUINO_PORT_NAME + ss.str()) || atoi(ss.str())>9){ss << atoi(ss.str())+1;}
  if (link.connect(BAUD_RATE,(ARDUINO_PORT_NAME + ss.str()){cout << "Connected on port" << ARDUINO_PORT_NAME << ss.str() << endl;}
  else{ cout << "[1]Check Permissions" << endl << "[2]Check USB" << endl;
      return 0;}
  
  usleep(10000);//wait
  
  while(ros::ok()){
    link.readData(32,serial_buffer);
    cout << buffer << endl; //to check received message
    //parse
  }

  
}
