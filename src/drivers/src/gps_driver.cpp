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
  ros::NodeHandle private_nh("~");
  ros::Rate loop_rate(10);
  ros::Publisher gps_publisher = nh.advertise<sb_messages::gps>(SENSOR_OUTPUT_TOPIC,20);
  ros::Publisher nav_sat_fix_publisher = nh.advertise<sensor_msgs::NavSatFix>("nav_sat_fix", 20);
  ros::Publisher odom_publisher = nh.advertise<nav_msgs::Odometry>(ODOM_TOPIC,20);
  //autoconnect under development
  /*if(!connect_device("GPS"))
    return 1;//Notify Error
  else
    cout << "Connected to GPS Arduino" << endl;
*/
  string port = "/dev/ttyACM0";
  if (!private_nh.getParam("port", port))
    cout << "Param not found, defaulting to default param" << endl;
  else
    cout << "Param read: "<< port << endl;
  if(!link_port.connect(BAUD_RATE,port)){
cout << "Unable to connect to a device on " << port << endl        << "Did you remember to set the correct port as a param? You should go do that" << endl;
    return 1;
}
  else
    cout << "Connected to: " << port << endl;

/*
    for (int i = 0; ; i++)
	{
	    stringstream ss;
	    ss << i;
	    if (link_port.connect(BAUD_RATE,(port + ss.str())))
	    {
	        cout << "connected on port " << port << i << endl;
	        break;
	    }  else if (i > 15) {
	        cout << "unable to find a device," << endl
		        << "did you remember to set usb permissions?" << endl
			<< "sudo chmod a+rw /dev/ttyACM*" << endl;
	        return 0;
	    }
	}
*/
	//usleep(1000);
	//link_port.clearBuffer();
  while(ros::ok() && link_port.isActive()){
		char buff[64] = "\0";
        data_request('D',buff,DATA);
        std::string g_input = to_string2(buff);
        //cout << buff;
        if (g_input.find('\n')!=std::string::npos
            && g_input.find('D')!=std::string::npos){
          gps_store(buff);
          gps_msg_create();
          nav_sat_fix_msg_create();
         // cout << setprecision(9) << gps_msg.lon << endl;
          //cout << setprecision(9) << gps_msg.lat << endl;
          gps_publisher.publish(gps_msg);
          odom_publisher.publish(odom);
          nav_sat_fix_publisher.publish(nav_sat_fix_msg);
         }
        loop_rate.sleep();
      cout << gps_msg << endl;
      cout << odom << endl;
    }
  ROS_ERROR("GPS Node Terminated");
  return 0;
}

void gps_msg_create(void){
  gps_msg.head = gps_comp_data.headingDegrees;
  double theta = gps_comp_data.headingDegrees/360*2*M_PI;
  odom.pose.pose.orientation.z = sin(theta/2);
  odom.pose.pose.orientation.w = cos(theta/2);
	if (gps_comp_data.fix){
	gps_msg.lon = gps_comp_data.longitude;
	gps_msg.lat = gps_comp_data.latitude;
    } else {
	gps_msg.lon = -1;
	gps_msg.lat = -1;
  }
return;
}

void nav_sat_fix_msg_create(void){
    nav_sat_fix_msg.header.frame_id = "base_link";
    nav_sat_fix_msg.header.stamp = ros::Time::now();
    nav_sat_fix_msg.status.service = 1;
    if (gps_comp_data.fix){
        nav_sat_fix_msg.status.status = 1;
        nav_sat_fix_msg.latitude = gps_comp_data.latitude;
        nav_sat_fix_msg.longitude = gps_comp_data.longitude;
    } else {
        nav_sat_fix_msg.status.status = -1;
        nav_sat_fix_msg.latitude = 0;
        nav_sat_fix_msg.longitude = 0;
    }
    nav_sat_fix_msg.position_covariance = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    nav_sat_fix_msg.position_covariance_type = 0;
return;
}

bool connect_device(std::string device_name){
  int i = 0;
  while ( i < 9 ){
  char buff[32] = "\0";
    if(open_port(i)){
      data_request('I',buff,1);
      std::string s_input = to_string2(buff);
      cout << "If you are not notified that GPS is connected, restart node" << endl;
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
      link_port.readData(64,buffer);
    }
  }
  else{
    while(buffer[0] == '\0' && i < 50){
      link_port.readData(64,buffer);
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
