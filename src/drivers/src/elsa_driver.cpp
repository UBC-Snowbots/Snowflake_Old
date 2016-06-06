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

static const string INIT_STRING = "BG";
static const char IDENTIFIER_BYTE = 'B';

static const int SECOND = 1000000;


// If rotation is greater or less then (-pi, pi), constrain it
double bound_rotation(double rotation, max_turn_rate){
	if(rotation > max_turn_rate){
		return max_turn_rate;
	}else if(rotation < -max_turn_rate){
		return -max_turn_rate;
	}else{
		return rotation;
	}
}

// Converts a given integer command to one the apm can understand
string commandToAPMCommand (int command){
    // Convert the integer command to a 3 long string, adding zero's if needed
    char output[4] = {0};
    snprintf(&output[0], 4, "%03u", command);
    return string(&output[0], 3);
}

// Converts a rotation command to an apm command (255 to 000)
string rotationCommandToAPMCommand(double rotation, double turn_rate_sensitivity, double max_turn_rate){
    int command = 125 - round(turn_rate_sensitivity * bound_rotation(rotation, max_turn_rate));
    return commandToAPMCommand(command);
}

// Converts a velocity command (-1 to 1) to an apm command (255 to 000)
string velocityCommandToAPMCommand(double velocity, double move_rate_sensitivity){
    // Convert velocity to value between 255 and 0
    unsigned char command = round((125 - (velocity * move_rate_sensitivity)));
    return commandToAPMCommand(command);
}


int main(int argc, char** argv) {
    //initialize ros
    ros::init(argc, argv, ROS_NODE_NAME);
    ros::NodeHandle public_nh;
    ros::NodeHandle private_nh("~");
	ros::Rate loop_rate(ROS_LOOP_RATE);

    //Set all values to neutral
    char twist_Y[3]={'1','2','5'}; //Strafe (normally not used, robot has no lateral movement)
    char twist_X[3]={'1','2','5'}; //Forward/Backward
    char twist_z[3]={'1','2','5'}; //Rotation


// Get any parameters
    // The maximum that the turn rate will go to
    double max_turn_rate = M_PI/2;
    private_nh.param("max_turn_rate", max_turn_rate);

    // How sensitive the robot is to a given turn command
    // A higher value means that smaller commands will have a greater effect
    // (ie. if turn_rate_sensitivity = 10, the apm command will be 125 + (turn_rate_sensitivity * given command))
    double turn_rate_sensitivity = 160;
    private_nh.param("turn_rate_sensitivity", turn_rate_sensitivity);

    // How sensitive the robot is to a given movement (forward/backward) command
    // A higher value means that smaller commands will have a greater effect
    // (ie. if move_rate_sensitivity = 10, the apm command will be 125 + (move_rate_sensitivity * given command))
    double move_rate_sensitivity = 25;
    private_nh.param("move_rate_sensitivity", move_rate_sensitivity);

    // The USB port where the driver will look for the APM
    string port = "/dev/ttyACM";
    private_nh.param("port", port);


    //Set Subscribers and Publishers
  	ros::Subscriber command_sub = public_nh.subscribe<geometry_msgs::Twist>
      ("move_straight_line/command", 10, boost::function<void(geometry_msgs::Twist)>
      ([&](geometry_msgs::Twist twist){
  		// Write converted velocity and rotate commands to twist_Y and twist_z
  		velocityCommandToAPMCommand(twist.linear.x, move_rate_sensitivity).copy(twist_X, 3, 0);
  		rotationCommandToAPMCommand(twist.angular.z, turn_rate_sensitivity, max_turn_rate).copy(twist_z, 3, 0);
      }));


	//initialize serial communication
	SerialCommunication link;
    for (int i = 0; ; i++)
	{
	    stringstream ss;
	    ss << i;
	    if (link.connect(BAUD_RATE,(port + ss.str())))
	    {
	        cout << "connected on port " << port << i << endl;
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


	//While ROS is running
	link.clearBuffer();
	while(ros::ok())
	{

	    //write to apm
		stringstream ss;
        ss << (char)IDENTIFIER_BYTE<< twist_Y[0]
                                  << twist_Y[1]
                                  << twist_Y[2]
                                  << twist_X[0]
                                  << twist_X[1]
                                  << twist_X[2]
                                  << twist_z[0]
                                  << twist_z[1]
                                  << twist_z[2];
	    link.writeData(ss.str(), 10);

       cout << ss.str() << endl;

	    //publish data
	    char test[24];
	    link.readData(24, test);
  	    //cout << test;

	    ros::spinOnce();
	    loop_rate.sleep();
	}

	//killing driver
	ROS_INFO("shutting down arduino_driver");
	string s = "ED" ;
    link.writeData(s, 2);

	return 0;
}
