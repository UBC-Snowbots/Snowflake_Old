#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Bool.h>
#include <vector>

// A list of destinations, where each destination is a desired state (Pose2D msg)
class Destinations{
        std::vector<geometry_msgs::Pose2D> destinations;
    public:
        Destinations(){};
        Destinations(std::vector<geometry_msgs::Pose2D> dest){
            destinations = dest;
        };
        
        // Adds a given Pose2D to the end of the list
        void add(geometry_msgs::Pose2D dest){
            destinations.push_back(dest);
        }

        // NEED TO REMEMBER TO DEAL WITH CASE OF NO DESTINATIONS HERE
        geometry_msgs::Pose2D getDestination(){
            return destinations.front();
        }
        
        // Removes the first destination and returns new first destination
        geometry_msgs::Pose2D getNextDestination(){
            destinations.erase(destinations.begin());
            return destinations.front();            
        }
};


int main(int argc, char **argv){
    ros::init(argc, argv, "sb_waypoint_creator");
    ros::NodeHandle nh;
    ros::Rate loop_rate(5);
    
    // Initialize destination publisher
    ros::Publisher forward_pub = nh.advertise<geometry_msgs::Pose2D>("destination", 10);
    
    // Get map data
    ros::Subscriber map = nh.subscribe<nav_msgs::OccupancyGrid>("map", 10, boost::function<void(nav_msgs::OccupancyGrid)>([&](nav_msgs::OccupancyGrid map){
    
    }));

    // Get pose data
    ros::Subscriber pose2d = nh.subscribe<geometry_msgs::Pose2D>("pose2D", 10, boost::function<void(geometry_msgs::Pose2D)>([&](geometry_msgs::Pose2D pose){
        
    }));

    bool at_destination = false;
    // Get at_destination (published by move_straight_line)
    ros::Subscriber at_destination_sub = nh.subscribe<std_msgs::Bool>("move_straight_line/at_destination", 10, boost::function<void(std_msgs::Bool)>([&](std_msgs::Bool at_dest){
        at_destination = at_dest.data;
    }));      
   
    // Create waypoints and add to list
    geometry_msgs::Pose2D dest1;
    dest1.x = 1;
    dest1.y = 0;
    dest1.theta = 0;
    geometry_msgs::Pose2D dest2;
    dest2.x = 0;
    dest2.y = 0;
    dest1.theta = 0;
    geometry_msgs::Pose2D dest3;
    dest3.x = 0;
    dest3.y = 0;
    dest3.theta = 0;

    
    Destinations destinations;
    destinations.add(dest1);
    destinations.add(dest2);
    destinations.add(dest3);
    
    // Main Loop to run while node is running
    while (ros::ok()){
        geometry_msgs::Pose2D destination;
        // If you've arrived at a destination, start broadcasting the next one
        if (at_destination){
	    printf("AT DESTINATION \n");
            destination = destinations.getNextDestination();
            loop_rate.sleep();
	} else {
	    printf("NOT AT DESTINATION \n");
            destination = destinations.getDestination();
 	    printf("%f", destination.x);		
        }
        forward_pub.publish(destination);
	ros::spinOnce();
	loop_rate.sleep();
    }
}
