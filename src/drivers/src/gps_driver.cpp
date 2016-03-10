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
  if(!connect_device("GPS")){return 1;//notify error;};   
  while(ros::ok() && link_port.isActive()){} 
  ROS_ERROR("GPS Node Terminated"); 
  return 0;
}
}
bool connect_device(std::string device_name){
  char buff[32]; 
  int i = 0; 
  while (i < 9){
    if(!open_port(i)){
      ROS_ERROR("PORT ERROR"); 
      return false; 
    }
    data_request('I');
    link_port.readData(32,buff); 
    if(buff[0]=='G' && buff[1]=='P' && buff[2]=='S')
      return true; 
    else 
      i++; 
  }
  ROS_ERROR("Connected device is not GPS"); 
  return false; 
} 

void data_request(char c){
  //clear input buffer and sends confirmation byte to prepare acceptance of message in serial
  link_port.clearBuffer(); 
  stringstream ss; 
  ss << c; 
  link_port.writeData(ss.str(),1);
}

bool open_port(unsigned int count){
 //Attempts to open Serial Port 
  while( !link_port.connect(BAUD_RATE,(ARDUINO_PORT_NAME + to_string(count
        )))&& count < 9){
  count++;
  }
  cout << endl; 
  if (link_port.connect(BAUD_RATE,(ARDUINO_PORT_NAME + to_string(count)))){
    cout << "Connected on port" << ARDUINO_PORT_NAME << count << endl; 
    return true; 
  }
  else{ 
    cout << "[1]Check Permissions" << endl << "[2]Check USB" << endl; 
    return false; 
  } 
 
}
std::string to_string(int i){
  ostringstream out;
  out << i; 
  return out.str();
}
