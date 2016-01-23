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

#define _USE_MATH_DEFINES

using namespace std;
using namespace ros;

//global constants
static const string ROS_NODE_NAME = "driver";
static const int ROS_LOOP_RATE = 200; //hz

static const int BAUD_RATE = 115200;
//static const string PORT_NAME = "/dev/ttyUSB";
static const string UNO_PORT_NAME = "/dev/ttyACM";

static const string INIT_STRING = "BG";
static const char IDENTIFIER_BYTE = 'B';

static const int SECOND = 1000000;

// Increasing these values will make the robot more responsive, but also less controllable.
// The robot may have issue with really high rates, with just one motor working in some cases
// ADJUST WITH CAUTION
static const int TURN_RATE = 75; // 125 is max, 0 is min (will not turn)
static const int MOVE_RATE = 25; // 125 is max, 0 is min (will not move)


// Converts a velocity command (-1 to 1) to an apm command (255 to 000)
string velocityCommandToAPMCommand(float velocity){
    // Convert velocity to value between 255 and 0
    string apm_command = to_string(floor(125 - (velocity * MOVE_RATE)));
    // Add zeros to the front until the lenght is 3
    while (apm_command.length() < 3){
        apm_command.insert(0, "0");
    }
    return apm_command;
}

double bound_rotation(double rotation){
	if(rotation > M_PI){
		return M_PI;
	}else if(rotation < -M_PI){
		return -M_PI;
	}else{
		return rotation;
	}
}

int bounded_rotation_to_APMValue(double rotation){
	return 125 - round(TURN_RATE * rotation_in_bounds / M_PI);
}

int rotationToAPMValue(double rotation){
	// ceiling and floor to max/min expected inputs
	double rotation_in_bounds = bound_rotation(rotation);
	int val_from_float = bounded_rotation_to_APMValue(rotation_in_bounds);
	
	const unsigned char MIN_TURN = 125 - TURN_RATE;
	const unsigned char MAX_TURN = 125 + TURN_RATE;
	// ceiling/floor to max/min specified outputs
	int apm_value;{
		if(val_from_float < MIN_TURN){
			apm_value = MIN_TURN;
		}else if(val_from_float > MAX_TURN){
			apm_value = MAX_TURN;
		}else{
			apm_value = val_from_float;
		}
	}
	return apm_value;
}

string rotationValueToAPMCommand(unsigned char value){
	char output[4] = {0};
	snprintf(&output[0], 4, "%03u", value);
	return string(&output[0], 3);
}

//Converts a rotation command (-PI to PI) to an apm command (000 to 255)
/*string rotationCommandToAPMCommand(float rotation){
    // If value is outside the bounds (-PI to PI) then take remainder
    // (fmod is just modulus for floats)
    rotation = fmod(rotation, M_PI);
    // Convert rotation to value between 0 and 255
    string apm_command = to_string(125 - (TURN_RATE * (rotation / M_PI)));
    // Add zeros to the front until length is 3
    while (apm_command.length() < 3){
        apm_command.insert(0, "0");
    }
    return apm_command;
}*/


int main(int argc, char** argv)
{
    //initialize ros
    init(argc, argv, ROS_NODE_NAME);
	NodeHandle n;
	Rate loop_rate(ROS_LOOP_RATE);

    //Set all values to neutral 
    char twist_Y[3]={'1','2','5'}; //Strafe (normally not used, look up strafe)
    char twist_X[3]={'1','2','5'}; //Forward/Backward
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
	    }  else if (i > 15) {
	        cout << "unable to find a device," << endl
		        << "did you remember to set usb permissions?" << endl
			<< "sudo chmod a+rw /dev/ttyACM0" << endl;
	        return 0;
	    }
	}
	usleep(10*SECOND);
		
	//subscribers and publishers

	Subscriber command_sub = n.subscribe<geometry_msgs::Twist>("move_straight_line/command", 10, boost::function<void(geometry_msgs::Twist)>([&](geometry_msgs::Twist twist){
		// Write converted velocity and rotate commands to twist_Y and twist_z 
		velocityCommandToAPMCommand(twist.linear.x).copy(twist_X, 3, 0);
		rotationValueToAPMCommand(
			rotationToAPMValue(twist.angular.z)).copy(twist_z, 3, 0);
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
		    ss << (char)IDENTIFIER_BYTE<< twist_Y[0] << twist_Y[1] << twist_Y[2] << twist_X[0] << twist_X[1] << twist_X[2] << twist_z[0] << twist_z[1] << twist_z[2];
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
