/**
 * Test Node, publishing
 * Publishes: Occ Grid
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
	string node_name = "occ_grid_pub_node_2";
	string topic_name = "map2";
	ros::init(argc, argv, node_name);
	ros::NodeHandle nh;

	ros::Publisher mapPub = nh.advertise<nav_msgs::OccupancyGrid>(topic_name, 1);
	ros::Rate loop_rate(1);


	//Create map
	geometry_msgs::Pose origin;
	origin.position.x = 0;
	origin.position.y = 0;
	origin.position.z = 0;
	origin.orientation.x = 0.0;
	origin.orientation.y = 0.0;
	origin.orientation.z = 0.0;
	origin.orientation.w = 0.0;
	nav_msgs::OccupancyGrid map;
	map.info.resolution = 1;
	map.info.width = 20;
	map.info.height = 20;
	map.info.origin = origin;
	//create a map with some boundaries
	for (int i = 0; i < 20*20; i++){
		map.data.push_back(0);
	}
	for (int i = 0; i < map.info.width; i++){
		map.data[map.info.width*2 + i] = 10;
	}
	for (int i = 0; i < map.info.width; i++){
		map.data[map.info.width*4 + i] = 10;
	}

	map.header.frame_id = "map";

	while(nh.ok()){
		mapPub.publish(map);
		loop_rate.sleep();
		ros::spinOnce();
	}

}
