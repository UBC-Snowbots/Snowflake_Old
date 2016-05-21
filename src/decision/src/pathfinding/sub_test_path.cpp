/**
 * Test Node, subscribing
 * Publishes: path
 */


#include <stdio.h>
#include <string>
#include <fstream>

//ROS MESSAGES
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <ros/ros.h>

using namespace std;
nav_msgs::Path path;

void poseCallback(const nav_msgs::Path::ConstPtr& msg){
	path.poses = msg->poses;
	path.header = msg->header;
}

int main(int argc, char** argv)
{

	string node_name = "path_sub_node";
	string sub_topic = "path";
	ros::init(argc, argv, node_name);
	ros::NodeHandle nh;
	ros::Subscriber pointSub = nh.subscribe(sub_topic, 5, poseCallback);
	ros::Rate loop_rate(1);

	cout << "In main" << endl;
	//Create pose

	while(nh.ok()){
		if (path.poses.size() != 0){
			for (int i = 0 ; i < path.poses.size(); i++){
				cout << "(" << path.poses[i].pose.position.x << "," << path.poses[i].pose.position.y << ")-";   
			}
			cout << endl;
		}
		loop_rate.sleep();
		ros::spinOnce();
	}

}