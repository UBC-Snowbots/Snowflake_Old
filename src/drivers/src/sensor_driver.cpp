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
static const int BAUD_RATE = 15200; 
static const string SENSOR_OUTPUT_TOPIC = "IMU"; 
static const string ARDUINO_PORT_NAME = "/dev/ttyACM";

sensor_msgs::Imu IMU; 
//char buffer[64];
std::string to_string(int i){
  ostringstream out;
  out << i;
  return out.str();
}
void IMU_write(int c, int val){
  switch (c){
  case 1: 
    IMU.linear_acceleration.x = val;
    cout << "linear_accel_x: " << IMU.linear_acceleration.x;
    break;
  case 2: 
    IMU.linear_acceleration.y = val;
    cout << "linear_accel_y: " << IMU.linear_acceleration.y;
    break; 
  case 3: 
    IMU.linear_acceleration.z = val;
    cout << "linear_accel_z: " << IMU.linear_acceleration.z;
    break;
  }
}

int pow (int base, int power){
  int val = 0; 
  for (int z = 0; z < power+1; z++){
    val*=base;
  }
}

bool Serial_Store(char *buffer){
  char *c;  
  c = strtok(buffer,",:");
  while (c != NULL){
    if (c[0] == 'X'){
      c = strtok(NULL,",:");
      IMU_write(1,atoi(c));     
    }
    else if (c[0] == 'Y'){
      c = strtok(NULL,",:");
      IMU_write(2,atoi(c));
    }
    else if (c[0] == 'Z'){
      c = strtok(NULL,",:");
      IMU_write(3,atoi(c));
      return true;
    }
    else{
      c = strtok(NULL,",:");
      cout << "Error in truncation: " << c << endl; 
      return false;
    }
      c = strtok(NULL,",:");
  }
  return true;
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
  int x = 0;
  while(ros::ok()){ //to check received message
  char buffer[64];
  link.readData(16,buffer);
  loop_rate.sleep();
  cout << "buffer:" << buffer << endl;
  Serial_Store(buffer);
  cout << endl;
  link.clearBuffer();
  }
}
