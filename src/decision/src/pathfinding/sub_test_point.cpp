/**
 * Test Node, publishing
 * Publishes: Pose2D initial
 */


#include <stdio.h>
#include <string>

//ROS MESSAGES
#include <geometry_msgs/Point.h>

#include <ros/ros.h>

using namespace std;

int main(int argc, char** argv)
{
	string node_name = "point_sub_node";
	string topic_name = "waypoint";
	ros::init(argc, argv, node_name);
	ros::NodeHandle nh;

	ros::Publisher pointPub = nh.advertise<nav_msgs::OccupancyGrid>(topic_name, 1);
	ros::Rate loop_rate(5);


	//Create pose
	geometry_msgs::Pose2D curr_pos;
	curr_pos.x = 18;
	curr_pos.y = 18;
	curr_pos.theta = 0;

	while(nh.ok()){
		pointPub.publish(curr_pos);
		ros::spinOnce();
	}

}