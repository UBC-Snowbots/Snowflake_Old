/*
*  driver node to communicate and control arduino
*  sorts incoming sensor information and outputs correct ros messages 
*/

#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <string>
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include "SerialCommunication.h"
#include "elsa_driver.h"


using namespace std;
using namespace ros;

//global constants
static const string ROS_NODE_NAME = "driver";
static const int ROS_LOOP_RATE = 200; //hz

static const int BAUD_RATE = 115200;
//static const string PORT_NAME = "/dev/ttyUSB";
static const string UNO_PORT_NAME = "/dev/ttyACM";
static const string BLUETOOTH_PORT_NAME = "/dev/rfcomm";

static const string ESTOP_TOPIC = "eStop";

static const string INIT_STRING = "BG";
static const char IDENTIFIER_BYTE = 'B';

static const int SECOND = 1000000;

int main(int argc, char** argv)
{
    //initialize ros
    init(argc, argv, ROS_NODE_NAME);
	NodeHandle n;
	Rate loop_rate(ROS_LOOP_RATE);

    //Set all values to neutral 
    char twist_X[3]={'1','2','5'}; //Strafe (normally not used, look up strafe)
    char twist_Y[3]={'1','2','5'}; //Forward/Backward
    char twist_z[3]={'1','2','5'}; //Rotation
    bool eStop = false;

	//initialize serial communication
	SerialCommunication link;
	for (int i = 0; ; i++)
	{
	    stringstream ss;
	    ss << i;	    
	    if (link.connect(BAUD_RATE,(UNO_PORT_NAME + ss.str())))
	    {
	        cout << "connected on port " << UNO_PORT_NAME << i << endl;
	        break;
		} else if (link.connect(BAUD_RATE,(BLUETOOTH_PORT_NAME + ss.str()))) {
			cout << "connected on bluetooth port " << BLUETOOTH_PORT_NAME << i << endl;
			break;
	    } else if (i > 15) {
	        cout << "unable to find a device" << endl;
	        return 0;
	    }
	}
	usleep(10*SECOND);
		
	//subscribers and publishers

	Subscriber command_sub = n.subscribe<geometry_msgs::Twist>("move_straight_line/command", 10, boost::function<void(geometry_msgs::Twist)>([&](geometry_msgs::Twist twist){
		if(twist.linear.x > 0){
		    twist_Y[0] = '0';
		    twist_Y[1] = '7';
		    twist_Y[2] = '5';
		} else if (twist.linear.x == 0) {
		    twist_Y[0] = '1';
		    twist_Y[1] = '2';
		    twist_Y[2] = '5';
		} else {
		    twist_Y[0] = '2';
		    twist_Y[1] = '5';
		    twist_Y[2] = '5';
		}
		if(twist.angular.z > 0){
		    twist_z[0] = '1';
		    twist_z[1] = '0';
                    twist_z[2] = '0';
		} else if (twist.angular.z < 0){
		    twist_z[0] = '1';
		    twist_z[1] = '5';
                    twist_z[2] = '0';
		} else {
		    twist_z[0] = '1';
		    twist_z[1] = '2';
                    twist_z[2] = '5';
		}
	}));	
	
	ROS_INFO("arduino_driver ready");

	//while ros is good
	link.clearBuffer();
	while(ok())
	{
	    
	    //write to arduino	 
	    stringstream ss;
	    if (eStop)
	    {	
		cout << "eStop on" << endl;
		ss << (char)IDENTIFIER_BYTE << "125125125";
	    } else {  
		ss << (char)IDENTIFIER_BYTE<< twist_X[0] << twist_X[1] << twist_X[2] << twist_Y[0] << twist_Y[1] << twist_Y[2] << twist_z[0] << twist_z[1] << twist_z[2];

	    }
	    cout << ss.str() << endl;
	    link.writeData(ss.str(), 10); 
	    //delay for sync
	    usleep(2000000);
	    
	    //publish data
	    char test[24];
	    link.readData(24, test);
  	    cout << test;
	   
	    spinOnce();
	    loop_rate.sleep();
	}
	
	//killing driver
	ROS_INFO("shutting down arduino_driver");
	string s = "ED" ;
    link.writeData(s, 2);
	
	return 0;
}
