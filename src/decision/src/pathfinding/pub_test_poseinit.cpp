/**
 * Test Node, publishing
 * Publishes: Pose2D initial
 */


#include <stdio.h>
#include <string>
#include <fstream>

//ROS MESSAGES
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>

#include <ros/ros.h>

using namespace std;
geometry_msgs::Pose2D curr_pos;

void pointCallback(const geometry_msgs::Point::ConstPtr& msg){
	curr_pos.x = msg->x;
	curr_pos.y = msg->y;
}

int main(int argc, char** argv)
{

	string node_name = "init_pose_pub_node";
	string topic_name = "initial_pose";
	string sub_topic = "waypoint";
	ros::init(argc, argv, node_name);
	ros::NodeHandle nh;

	ros::Publisher pointPub = nh.advertise<geometry_msgs::Pose2D>(topic_name, 1);
	ros::Subscriber pointSub = nh.subscribe(sub_topic, 5, pointCallback);
	ros::Rate loop_rate(1);

	cout << "In main" << endl;
	//Create pose

	curr_pos.x = 1;
	curr_pos.y = 8;
	curr_pos.theta = 0;

	ofstream file;
	file.open("/home/valerian/test.txt", ios_base::app);



	while(nh.ok()){
		pointPub.publish(curr_pos);
		file << "(" << curr_pos.x << "," << curr_pos.y << ")" << endl;
		cout << "(" << curr_pos.x << "," << curr_pos.y << ")" << endl;
		loop_rate.sleep();
		ros::spinOnce();
	}

}