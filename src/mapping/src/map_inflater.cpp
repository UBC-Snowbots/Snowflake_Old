/*
 * Inflates all obstacles in the map using occupancy_grid_utils
 * Author: Gareth Ellis
 * Date: May 21, 2016
 */


#include <ros/ros.h>
#include <occupancy_grid_utils/shortest_path.h> // This contains inflateObstacles
#include <nav_msgs/OccupancyGrid.h>

using namespace std;


class MapInflater{
    public:
        MapInflater();
        void mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& map);
    private:
        ros::Subscriber map_sub;
        ros::Publisher map_pub;
};

MapInflater::MapInflater(){
    ros::NodeHandle nh;
    map_sub = nh.subscribe("map", 1, &MapInflater::mapCallBack, this);
    string topic = nh.resolveName("inflated_map");
    map_pub = nh.advertise<nav_msgs::OccupancyGrid>(topic, 1);
}

void MapInflater::mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& map){
    nav_msgs::OccupancyGrid::Ptr inflated_map;
    inflated_map = occupancy_grid_utils::inflateObstacles(*map, 0.5, false);
    map_pub.publish(inflated_map);
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "map_inflater");

    MapInflater map_inflater;

    ros::spin();

    return 0;
}
