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
  ros::Publisher gps_publisher = nh.advertise<::messages::gps>(SENSOR_OUTPUT_TOPIC,20); 
  if(!connect_device("GPS"))
    return 1;//Notify Error
  else 
    cout << "Connected to GPS Arduino" << endl;  
	//usleep(1000);
	//link_port.clearBuffer();
  while(ros::ok() && link_port.isActive()){
//    cout << "inside loop" << endl; 
		char buff[32]="\0"; 
    data_request('D',buff);
		if(gps_store(buff)){
    gps_msg_create();  
		cout << gps_msg << endl;
		gps_publisher.publish(gps_msg); 
		}
	} 
  ROS_ERROR("GPS Node Terminated"); 
  return 0;
}

void gps_msg_create(void){
	if (gps_comp_data.fix){
	gps_msg.Lon = gps_comp_data.longitude;
	gps_msg.Lat = gps_comp_data.latitude;
	gps_msg.Head = gps_comp_data.headingDegrees;}
	else {
	gps_msg.Lon = -1;
	gps_msg.Lat = -1; 
	gps_msg.Head = -1;} 
return;
}

bool connect_device(std::string device_name){
//  char buff[32] = "\0"; 
  int i = 0; 
  while (i < 9){
  char buff[64] = "\0";
    /*if(open_port(i){
      data_request
    }*/
    if(!open_port(i)){
      ROS_ERROR("PORT ERROR"); 
      return false; 
    }
   data_request('I',buff);
  //  link_port.readData(32,buff);
     
//Repeat until read gives value?
  //  cout << "buff:" << buff << endl;  
    std::string s_input = to_string2(buff);
//    cout << "s_input: " << s_input << endl;
    if(s_input.find(device_name)!=std::string::npos)
      return true; 
    else 
      i++; 
  }
  ROS_ERROR("Connected device is not GPS"); 
  return false; 
} 

void data_request(char c, char *buffer){
  //clear input buffer and sends confirmation byte to prepare acceptance of message in seriali
	//link_port.clearBuffer(); 
  stringstream ss; 
  ss << c; 
  link_port.writeData(ss.str(),1);
	//usleep(200); 
  while(buffer[0] == '\0'){
  //link_port.clearBuffer(); 
  link_port.readData(64,buffer);
 	//cout << buffer << endl;
	//cout << "data_request" << endl;
	usleep(500);
 } 
  return;
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
std::string to_string2(char* c){
  ostringstream out;    
  out << c;
  return out.str();
}


bool gps_store(char *buffer){
  //To parse GPS data coming from arduino 
  //Assumed format for data already being parsed from arduino side
	char *c;
	//usleep(100);
cout << buffer;
//  strcpy (buffer, "G,49.2622,-123.2483,1,-11.73,-17.82,15.10,252.65");
  c = strtok(buffer,",");
  if (c == NULL) return false; 
	if (c[0] == 'D'){
    //buffer += 1;
    c = strtok(NULL,",");
   // cout << "C: "<< c  << endl;
    if( c != NULL){
   // cout << "0";
    gps_comp_data.latitude = atof(c);
//cout << "1";
      c = strtok(NULL,",");
      gps_comp_data.longitude = atof(c);
//cout << "2";
      c = strtok(NULL,",");
      gps_comp_data.fix = atoi(c);
//cout << "3";
      c = strtok(NULL,",");
       gps_comp_data.x = atof(c);
//cout << "4";
      c = strtok(NULL,",");
      gps_comp_data.y = atof(c);
//cout << "5";
      c = strtok(NULL,",");
      gps_comp_data.z = atof(c);
//cout << "6";
      c = strtok(NULL,",");
      gps_comp_data.headingDegrees = atof(c);
     //cout << "c not null" << endl;;
    return true;
  }
  else{ //cout << "fail" << endl;
    return false;}
}
}

