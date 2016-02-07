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

//global constants
static const string ROS_NODE_NAME = "elsa_driver";
static const int ROS_LOOP_RATE = 200; //hz

static const int BAUD_RATE = 115200;
//static const string PORT_NAME = "/dev/ttyUSB";
static const string UNO_PORT_NAME = "/dev/ttyACM";

static const string INIT_STRING = "BG";
static const char IDENTIFIER_BYTE = 'B';

static const int SECOND = 1000000;


// Converts a velocity command (-1 to 1) to an apm command (255 to 000)
string velocityCommandToAPMCommand(double velocity){
    // Convert velocity to value between 255 and 0
    unsigned char apm_command = round((125 - (velocity * MOVE_RATE)));
    char output[4] = {0};
	snprintf(&output[0], 4, "%03u", apm_command);
	return string(&output[0], 3);
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

int bounded_rotation_to_APMValue(double rotation_in_bounds){
	// This scales rotation to whatever value is passed to iti
	return 125 - round(TURN_RATE * rotation_in_bounds);
}

int rotationToAPMValue(double rotation){
	// ceiling and floor to max/min expected inputs
	double rotation_in_bounds = bound_rotation(rotation);
	int val_from_float = bounded_rotation_to_APMValue(rotation_in_bounds);

	const unsigned char MIN_TURN = 125 - MAX_APM_TURN_RATE;
	const unsigned char MAX_TURN = 125 + MAX_APM_TURN_RATE;
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

int main(int argc, char** argv)
{
    //initialize ros
  ros::init(argc, argv, ROS_NODE_NAME);
	ros::NodeHandle public_nh;
  ros::NodeHandle private_nh("~");
	ros::Rate loop_rate(ROS_LOOP_RATE);

    // Get any parameters
        // The maximum that the turn rate will go to
        // (ie. if max_turn_rate = 40, the max command sent to the apm would be 125 +/- 40 )
    double max_turn_rate = 40;
    if (private_nh.getParam("max_turn_rate", max_turn_rate_param)){
      max_turn_rate = max_turn_rate_param;
    }
        // How sensitive the robot is to a given turn command
        // A higher value means that smaller commands will have a greater effect
    double turn_rate_sensitivity = 160;
    if (private_nh.getParam("turn_rate_sensitivity", turn_rate_sensitivity_param)){
      turn_rate_sensitivity = turn_rate_sensitivity_param;
    }
        // How sensitive the robot is to a given turn command
        // A higher value means that smaller commands will have a greater effect
    double move_speed_sensitivity = 25;
    if (private_nh.getParam("move_speed_sensitivity", move_speed_sensitivity_param)){
      move_speed_sensitivity = move_speed_sensitivity_param;
    }

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
  ROS_INFO("elsa_driver ready");

	//subscribers and publishers
	Subscriber command_sub = n.subscribe<geometry_msgs::Twist>("move_straight_line/command", 10, boost::function<void(geometry_msgs::Twist)>([&](geometry_msgs::Twist twist){
		// Write converted velocity and rotate commands to twist_Y and twist_z
		velocityCommandToAPMCommand(twist.linear.x).copy(twist_X, 3, 0);
		rotationValueToAPMCommand(
			rotationToAPMValue(twist.angular.z)).copy(twist_z, 3, 0);
    }));

	//While ROS is running
	link.clearBuffer();
	while(ok())
	{

	    //write to apm
	    stringstream ss;
	    if (eStop)
	    {
		    cout << "eStop on" << endl;
		    ss << (char)IDENTIFIER_BYTE << "125125125";
	    } else {
		    ss << (char)IDENTIFIER_BYTE<< twist_Y[0] << twist_Y[1] << twist_Y[2] << twist_X[0] << twist_X[1] << twist_X[2] << twist_z[0] << twist_z[1] << twist_z[2];
	    }
	    link.writeData(ss.str(), 10);

	    //publish data
	    char test[24];
	    link.readData(24, test);
  	    //cout << test;

	    spinOnce();
	    loop_rate.sleep();
	}

	//killing driver
	ROS_INFO("shutting down arduino_driver");
	string s = "ED" ;
    link.writeData(s, 2);

	return 0;
}
