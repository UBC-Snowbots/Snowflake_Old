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

SerialCommunication link_port;

sensor_msgs::Imu IMU; 

std::string to_string(int i){
  ostringstream out;
  out << i;
  return out.str();
}

void IMU_write(int c, int val){
  //Takes in value val and writes to IMU.msg depending on situation 
  switch (c){
  case 1: 
    IMU.linear_acceleration.x = val;
    //cout << "linear_accel_x: " << IMU.linear_acceleration.x;
    break;
  case 2: 
    IMU.linear_acceleration.y = val;
    //cout << "linear_accel_y: " << IMU.linear_acceleration.y;
    break; 
  case 3: 
    IMU.linear_acceleration.z = val;
    //cout << "linear_accel_z: " << IMU.linear_acceleration.z;
    break;
  }
}

bool Serial_Store(char *buffer){
  //Parses the buffer characters and stores them into the respective IMU position 
  //When strtok is passed a NULL pointer, it checks the previous succeful truncation location
  char *c;  //temp char that stores the char to convert to integer
  c = strtok(buffer,",:");
  while (c != NULL){
    if (c[0] == 'X'){ //Looks for linear_accel_x 
      c = strtok(NULL,",:");
      IMU_write(1,atoi(c));     
    }
    else if (c[0] == 'Y'){ //Looks for linear_accel_y
      c = strtok(NULL,",:");
      IMU_write(2,atoi(c));
    }
    else if (c[0] == 'Z'){ //Looks for linear_accel_y
      c = strtok(NULL,",:");
      IMU_write(3,atoi(c));
      return true;
    }
   
      c = strtok(NULL,",:");
  }
  return true;
}

void data_request(void){
  //clears input buffer and sends confirmation byte to prepare acceptance of message in serial
  link_port.clearBuffer();
  stringstream ss; 
  ss << 'B'; 
  link_port.writeData(ss.str(),1); 
}

int main (int argc, char** argv){
	ros::init(argc,argv, ROS_NODE_NAME);
	ros::NodeHandle nh; 
	ros::Rate loop_rate(ROS_LOOP_RATE);
  ros::Publisher sensor_imu_publisher = nh.advertise<sensor_msgs::Imu>(SENSOR_OUTPUT_TOPIC,20);
  //Attempts at opening the Serial Port
  unsigned int count = 0;
  while( !link_port.connect(BAUD_RATE,(ARDUINO_PORT_NAME + to_string(count)))&& count < 9){
    count++;
  }
  cout << endl;
  if (link_port.connect(BAUD_RATE,(ARDUINO_PORT_NAME + to_string(count)))){
    cout << "Connected on port" << ARDUINO_PORT_NAME << count << endl;
    }
  else{ 
    cout << "[1]Check Permissions" << endl << "[2]Check USB" << endl;
    return 1;
      }
  
  //Temporary Function for Setting covariance values 
  for (int i = 0; i < 9; i++){
  IMU.linear_acceleration_covariance[i] = 0; 
  IMU.angular_velocity_covariance[i]=0;
  IMU.orientation_covariance[i]=0;
  }
  
  int x = 0;
  while(ros::ok()){
    //ROS Loop - All procedures repeated are done here
    char buffer[32];
    data_request();//request dat
    loop_rate.sleep(); // Wait for messages to arrive 
    link_port.readData(16,buffer); // Read messsages 
    if(Serial_Store(buffer))//if the messages read are good, publish them, else do nothing
    sensor_imu_publisher.publish(IMU);
  }
  ROS_ERROR("Sensor Input Node Terminated"); 
}
