/*
* Node: sb_gps_driver in driver node
* Purpose: Collecting gps/compass data via serial 
* Author: Vincent Yuan 
* Date: March 7, 2016
*/
#include "gps_driver.hpp"

using namespace std; 
int main (int argc, char **argv){
  ros::init(argc, argv, ROS_NODE_NAME);
  ros::NodeHandle nh; 
  ros::Rate loop_rate(ROS_LOOP_RATE); 
  ros::Publisher gps_publisher = nh.advertise<std_msgs::String>(SENSOR_OUTPUT_TOPIC,20); 
  //Attempts to open Serial Port
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
  while(ros::ok() && link_port.isActive()){} 
  ROS_ERROR("GPS Node Terminated"); 
  return 0;
}

std::string to_string(int i){
  ostringstream out;
  out << i; 
  return out.str();
}
