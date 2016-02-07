/*
 File: arduino_driver.h

 Author: Nick Adams and Jarek Ignasmenzies
 Last Edited: December 21, 2010
 Version 1.0

 header file for driver node
*/
#ifndef ARDUINO_DRIVER
#define ARDUINO_DRIVER

#include <iostream>
#include <sstream>
#include <string>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
//#include "sb_msgs/TurretCommand.h"
#include "SerialCommunication.h"

using namespace std;

/*
 eStop_callback function
 takes a bool which is the on/off state for the wireless eStop
*/
void eStop_callback(const std_msgs::BoolConstPtr& msg_ptr);

#endif
