/*
*  driver node to communicate and control arduino
*  sorts incoming sensor information and outputs correct ros messages 
*/

#include <iostream>
#include <sstream>
#include <string>
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include "sb_msgs/TurretCommand.h"
#include "sb_msgs/RobotState.h"
#include "sb_msgs/IMU.h"
#include "SerialCommunication.h"
#include "avalanche_driver.h"

using namespace std;
using namespace ros;

//global constants
static const string ROS_NODE_NAME = "driver";
static const int ROS_LOOP_RATE = 20; //hz

static const int BAUD_RATE = 115200;
//static const string PORT_NAME = "/dev/ttyUSB";
static const string UNO_PORT_NAME = "/dev/ttyACM";
static const string BLUETOOTH_PORT_NAME = "/dev/rfcomm";

static const string CAR_COMMAND_TOPIC = "lidar_nav";
static const string TURRET_COMMAND_TOPIC = "turret_command";
static const string ESTOP_TOPIC = "eStop";
static const string ROBOT_STATE_TOPIC = "robot_state";
static const string GPS_STATE_TOPIC = "gps_state";

static const string INIT_STRING = "BG";
static const char IDENTIFIER_BYTE = 'B';

static const int SECOND = 1000000;

//global variables
ServoControl servo;
MechControl mech;
char twist_x[3]={'1','2','5'};
char twist_y[3]={'1','2','5'};
char twist_z[3]={'1','2','5'};

bool eStop = false;

int main(int argc, char** argv)
{
    //initialize ros
    init(argc, argv, ROS_NODE_NAME);
	NodeHandle n;
	Rate loop_rate(ROS_LOOP_RATE);

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
	
	usleep(3*SECOND);
		
	//subscribers and publishers
	Subscriber car_command = n.subscribe(CAR_COMMAND_TOPIC, 1, car_command_callback);
	Subscriber turret_command = n.subscribe(TURRET_COMMAND_TOPIC, 1, turret_command_callback);
	Subscriber eStop_topic = n.subscribe(ESTOP_TOPIC, 1, eStop_callback);
	
	Publisher robot_state = n.advertise<sb_msgs::RobotState>(ROBOT_STATE_TOPIC,1);
	Publisher gps_state = n.advertise<std_msgs::String>(GPS_STATE_TOPIC,1);
	
	sb_msgs::IMU imu;
	sb_msgs::RobotState state;
	std_msgs::String gps_data;
	
	ROS_INFO("arduino_driver ready");
	mech.twist_x=125;
	mech.twist_y=125;
	mech.twist_z=125;

	//while ros is good
	ros::Time begin = ros::Time::now();
	int counter = 0;
	link.clearBuffer();
	while(ok())
	{
		if (int((ros::Time::now() - begin).toSec()) >= 1) {
			cout << counter << endl;
			counter = 0;
			begin = ros::Time::now();
		}
		counter++;
	    //write to arduino	 
		stringstream ss;
	    if (eStop)
	    {	
			cout << "eStop on" << endl;
			ss << (char)IDENTIFIER_BYTE << "125125125";
	    } else {  
            //use carCommand and turretCommand
			ss << (char)IDENTIFIER_BYTE << twist_x[0] << twist_x[1] << twist_x[2] << twist_y[0] << twist_y[1] << twist_y[2] << twist_z[0] << twist_z[1] << twist_z[2];

	    }
	    link.writeData(ss.str(), 10);
	    
	    //delay for sync
	    usleep(20000);
	    
	    //publish data
	    robot_state.publish(state);
	    
	    /*gps_data.data = link.readData(38);
	    cout << "GPS DATA " << gps_data.data << endl; // print out gps datas
	    gps_state.publish(gps_data);*/
	    
	    //clear buffer (MAY NOT WORK)
	    link.clearBuffer();
	    
	    //log and loop
	   // ROS_INFO("%i,%i,%i",mech.twist_x, mech.twist_y, mech.twist_z);
	    spinOnce();
		loop_rate.sleep();
	}
	
	//killing driver
	ROS_INFO("shutting down arduino_driver");
	string s = "ED" ;
    link.writeData(s, 2);
	
	return 0;
}

//dummy function
void processData(string data,sb_msgs::RobotState &state)
{
	state.ir.push_back(data[0] << 8|data[1]);
	state.ir.push_back(data[2] << 8|data[3]);
	state.ir.push_back(data[4] << 8|data[5]);
	state.ir.push_back(data[6] << 8|data[7]);
	state.ir.push_back(data[8] << 8|data[9]);
	state.num_analog = (int)state.ir.size();
}

//car_command_callback
void car_command_callback(const geometry_msgs::TwistConstPtr& msg_ptr)
{
	mech.twist_x = 125;
	mech.twist_y = msg_ptr->linear.y * 125+125; 
	mech.twist_z = -msg_ptr->angular.z * 125+125;

	sprintf(twist_x,"%03d",mech.twist_x);
	sprintf(twist_y,"%03d",mech.twist_y);
	sprintf(twist_z,"%03d",mech.twist_z);
	ROS_INFO("Twist_y: %s", twist_y);
	ROS_INFO("Twist_z: %s", twist_z);
}

//turret_command_callback
void turret_command_callback(const sb_msgs::TurretCommandConstPtr& msg_ptr)
{
	servo.pan = msg_ptr->pan * 90 + 90;
    servo.tilt = msg_ptr->tilt * 90 + 90;
}

//eStop_callback
void eStop_callback(const std_msgs::BoolConstPtr& msg_ptr)
{
    eStop = msg_ptr->data;
}
