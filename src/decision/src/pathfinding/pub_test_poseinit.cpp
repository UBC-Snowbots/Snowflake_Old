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
#include <nav_msgs/Odometry.h>

#include <ros/ros.h>

using namespace std;
nav_msgs::Odometry odom_pos;

void pointCallback(const geometry_msgs::Point::ConstPtr& msg){
	odom_pos.pose.pose.position.x = msg->x;
	odom_pos.pose.pose.position.y = msg->y;
}

int main(int argc, char** argv)
{

	string node_name = "init_pose_pub_node";
	string topic_name = "initial_pose";
	string sub_topic = "waypoint";
	ros::init(argc, argv, node_name);
	ros::NodeHandle nh;

	ros::Publisher pointPub = nh.advertise<nav_msgs::Odometry>(topic_name, 1);
	ros::Subscriber pointSub = nh.subscribe(sub_topic, 5, pointCallback);
	ros::Rate loop_rate(1);

	cout << "In main" << endl;
	//Create pose

	odom_pos.pose.pose.position.x = 0;
	odom_pos.pose.pose.position.y = 0;




	while(nh.ok()){
		cout << "(" << odom_pos.pose.pose.position.x  << "," << odom_pos.pose.pose.position.y << ")" << endl;
		pointPub.publish(odom_pos);
		loop_rate.sleep();
		ros::spinOnce();
	}

}
