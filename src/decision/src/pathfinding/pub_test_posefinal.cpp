/**
 * Test Node, publishing
 * Publishes: Pose2D initial
 */


#include <stdio.h>
#include <string>

//ROS MESSAGES
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>

#include <ros/ros.h>

using namespace std;

int main(int argc, char** argv)
{
	string node_name = "final_pose_pub_node";
	string topic_name = "final_pose";
	ros::init(argc, argv, node_name);
	ros::NodeHandle nh;

	ros::Publisher pointPub = nh.advertise<geometry_msgs::Pose2D>(topic_name, 1);
	
	ros::Rate loop_rate(1);

	//Create pose
	geometry_msgs::Pose2D point;
	point.x = 18;
	point.y = 18;
	point.theta = 0;

	while(nh.ok()){
		pointPub.publish(point);
		loop_rate.sleep();
		ros::spinOnce();
	}

}