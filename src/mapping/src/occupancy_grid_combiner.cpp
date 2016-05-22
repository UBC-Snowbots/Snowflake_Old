/**
 * A node that combines two occupancy grids together
 * Author: Valerian Ratu
 *
 * TODO: make callback boost functions that can take in params
 */

#include <occupancy_grid_utils/combine_grids.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <unistd.h>
#include <boost/bind.hpp>

using namespace std;

vector<nav_msgs::OccupancyGrid::ConstPtr> grids;

void gridCallback1(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	if (grids.size() >= 2){
		grids[0] = msg;
	} else {
		grids.push_back(msg);
	}
}

void gridCallback2(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	if (grids.size() >= 2){
		grids[1] = msg;
	} else {
		grids.push_back(msg);
	}
}

int main(int argc, char** argv)
{
	string node_name = "occ_grid_combiner";
	string occ_grid_sub_1 = "map1";
	string occ_grid_sub_2 = "map2";
	string publish_topic = "combined_grid";
	ros::init(argc, argv, node_name);
	ros::NodeHandle nh;

	ros::Publisher occGridPub = nh.advertise<nav_msgs::OccupancyGrid>(publish_topic, 1);
	ros::Subscriber occGridSub1 = nh.subscribe(occ_grid_sub_1, 5, gridCallback1);
	ros::Subscriber occGridSub2 = nh.subscribe(occ_grid_sub_2, 5, gridCallback2);
	ros::Rate loop_rate(1);

	while (nh.ok())
	{
		if (!grids.empty()){
			occGridPub.publish(occupancy_grid_utils::combineGrids(grids));
		}
		loop_rate.sleep();
		ros::spinOnce();
	}
}
