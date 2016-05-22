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
  ros::Rate loop_rate(10); 
  ros::Publisher gps_publisher = nh.advertise<::messages::gps>(SENSOR_OUTPUT_TOPIC,20); 
  if(!connect_device("GPS"))
    return 1;//Notify Error
  else 
    cout << "Connected to GPS Arduino" << endl;  
	//usleep(1000);
	//link_port.clearBuffer();
  while(ros::ok() && link_port.isActive()){
		    char buff[32] = "\0";
        data_request('D',buff,DATA);
        std::string g_input = to_string2(buff);
        //cout << buff;
        if (g_input.find('\n')!=std::string::npos 
            && g_input.find('D')!=std::string::npos){
          gps_store(buff);
          gps_msg_create();
          cout << gps_msg;
        }
        loop_rate.sleep();
    } 
  ROS_ERROR("GPS Node Terminated"); 
  return 0;
}

void gps_msg_create(void){
  gps_msg.Head = gps_comp_data.headingDegrees;
	if (gps_comp_data.fix){
	gps_msg.Lon = gps_comp_data.longitude;
	gps_msg.Lat = gps_comp_data.latitude;}
	else {
	gps_msg.Lon = -1;
	gps_msg.Lat = -1; 
  } 
return;
}

bool connect_device(std::string device_name){
  int i = 0;
  while ( i < 9 ){
  char buff[32] = "\0";
    if(open_port(i)){
      data_request('I',buff,1);
      std::string s_input = to_string2(buff);
      while(s_input.find('\n')==std::string::npos){
          data_request('I',buff,1);
          s_input = to_string2(buff);
      }
      if(s_input.find(device_name)!=std::string::npos)
          return true;
    }
   i++;
  }

  ROS_ERROR("Connected device is not GPS"); 
  return false; 
} 

bool open_port(unsigned int count){
 //Attempts to open Serial Port 
  while( !link_port.connect(BAUD_RATE,(ARDUINO_PORT_NAME + to_string(count
        )))&& count < 9){
  count++;
  }
  //cout << endl; 
  if (link_port.connect(BAUD_RATE,(ARDUINO_PORT_NAME + to_string(count)))){
    cout << "Connected on port" << ARDUINO_PORT_NAME << count << endl; 
    usleep(5000);
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
std::string to_string2(char* c){
  //used in connect_device
  ostringstream out;    
  out << c;
  return out.str();
}

void data_request(char c, char *buffer, int mode){
  //clear input buffer and sends confirmation byte to prepare acceptance of message in seriali
  link_port.clearBuffer(); 
  stringstream ss; 
  ss << c; 
  int i = 0;
  link_port.writeData(ss.str(),1);
  if (mode == 0){
    while(buffer[0] == '\0'){
      link_port.writeData(ss.str(),1);
      link_port.readData(32,buffer); 
    } 
  }
  else{
    while(buffer[0] == '\0' && i < 20){
      link_port.readData(32,buffer);
    }
  }
  return;
}


void struct_store(int type, char *val){
  switch(type){
    case 1: gps_comp_data.latitude = atof(val); break;
    case 2: gps_comp_data.longitude = atof(val); break;
    case 3: gps_comp_data.fix = atoi(val); break;
    case 4: gps_comp_data.headingDegrees = atof(val); break;
  }
}

bool gps_store(char *buffer){
  //To parse GPS data coming from arduino 
  //Assumed format for data already being parsed from arduino side
	char *c;
  int i = 1;
  c = strtok(buffer,",");
  if(c[0] == 'D'){
    while (i < 5){
        if ( c != NULL){
        c = strtok(NULL,",");
        struct_store(i, c); 
        i++;
        }
     }
    return true;    
  }
  else{ //cout << "fail" << endl;
      return false;}
}


