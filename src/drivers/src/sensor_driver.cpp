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

static const string ROS_NODE_NAME = "sensor_driver";
static const int ROS_LOOP_RATE = 10; //units of Hz should be 200
static const int BAUD_RATE = 9600; 
static const string SENSOR_OUTPUT_TOPIC = "IMU"; 
static const string ARDUINO_PORT_NAME = "/dev/ttyACM";

sensor_msgs::Imu IMU; 
char buffer[64];
std::string to_string(int i){
  ostringstream out;
  out << i;
  return out.str();
}
void Serial_Store(void){
  //stores input buffer into int
  char temp[64];
  int i,j = 0;
  for ( i = 0; i < 64 && temp[i] != '\0' ; i++){
    if (buffer[i] != ','){
      temp[j] = buffer[i];
      j++;
    }
    //else { cout << atoi(temp) << endl;j = 0;} 
  }
  cout << atoi(temp) << endl; 
  /*
  while (buffer[i] != ',' && buffer[i] != '\0'){
  temp[j] = buffer[i];
  i++; j++;
  }
  IMU.linear_acceleration.x = atoi(temp);
  j = 0;
  while (buffer[i] != ','){
  temp[j] = buffer[i];
  i++; j++;
  }
  IMU.linear_acceleration.y = atoi(temp);
  j = 0;
  while (buffer[i] != '\n'){
  temp[j] = buffer[i];
  i++; j++;
  }
  IMU.linear_acceleration.z = atoi(temp);
  */
}

int main (int argc, char** argv){
	ros::init(argc,argv, ROS_NODE_NAME);
	ros::NodeHandle nh; 
	ros::Rate loop_rate(ROS_LOOP_RATE);
  ros::Publisher sensor_imu_publisher = nh.advertise<sensor_msgs::Imu>(SENSOR_OUTPUT_TOPIC,20);
  
  SerialCommunication link;

  unsigned int count = 0;
  while(
      !link.connect(BAUD_RATE,(ARDUINO_PORT_NAME + to_string(count)))
      && count < 9){
    count++;
  }
  cout << endl;
  if (link.connect(BAUD_RATE,(ARDUINO_PORT_NAME + to_string(count)))){
    cout << "Connected on port" << ARDUINO_PORT_NAME << count << endl;
    }
  else{ 
    cout << "[1]Check Permissions" << endl << "[2]Check USB" << endl;
    return 1;
      }
  for (int i = 0; i < 9; i++){
  IMU.linear_acceleration_covariance[i] = 0; 
  IMU.angular_velocity_covariance[i]=0;
  IMU.orientation_covariance[i]=0;
  }
  usleep(10000);//wait  
  while(ros::ok()){ //to check received message
  link.readData(16,buffer);
  Serial_Store();
 // cout << buffer;
  }

  
}
