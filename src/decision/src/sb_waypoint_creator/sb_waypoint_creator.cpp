#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
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
    ros::NodeHandle public_nh;
    ros::NodeHandle private_nh("~");
    ros::Rate loop_rate(5);

    Destinations destinations; // The destinations for the robot to go to
   

// Get parameters
    // "path" is a vector of alternating x,y coordinates, that make up a 
    // series of waypoints for the robot to move to. 
    // If it does not exist, autonomous_mode should be set true
    bool autonomous_mode = false;
    if (private_nh.hasParam("path")){
        std::vector<double> path; // Path for the robot to follow {x,y,x,y, ...}
        private_nh.getParam("path", path);
        // Add all waypoints in "path"  as pose2D's in destinations
        for (int i = 0; i < path.size()/2; i++){
            geometry_msgs::Pose2D dest;
            dest.x = path[i * 2];
            dest.y = path[i * 2 + 1];
            dest.theta = 0;
            destinations.add(dest);
            ROS_INFO("X: %f\n", path[i]);
        }
    } else {
        autonomous_mode = true;
    }

    // Initialize destination publisher
    ros::Publisher forward_pub = public_nh.advertise<geometry_msgs::Pose2D>("destination", 10);
    
    bool at_destination = false;
    // Get at_destination (published by move_straight_line)
    ros::Subscriber at_destination_sub = public_nh.subscribe<std_msgs::Bool>("move_straight_line/at_destination", 10, boost::function<void(std_msgs::Bool)>([&](std_msgs::Bool at_dest){
        at_destination = at_dest.data;
    }));      

/*
//TEST PATH   
    // Create waypoints and add to list
    geometry_msgs::Pose2D dest1;
    dest1.x = 2;
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

    
    destinations.add(dest1);
    destinations.add(dest2);
    destinations.add(dest3);
*/
    
    int at_destination_counter = 0; // A ghetto solution to allow at_destination to publish twice, but only go the next destination 

    // Main Loop to run while node is running
    while (ros::ok()){
        geometry_msgs::Pose2D destination;
        // If you've arrived at a destination, start broadcasting the next one
        if (at_destination_counter == 2){
            at_destination_counter = 0;
            destination = destinations.getNextDestination();
            loop_rate.sleep();
	    } else if (at_destination){
            at_destination_counter++;
            destination = destinations.getDestination();
        } else {
            at_destination_counter = 0;
            destination = destinations.getDestination();
        }
        forward_pub.publish(destination);
	ros::spinOnce();
	loop_rate.sleep();
    }
}
