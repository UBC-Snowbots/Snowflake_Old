#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>
#include <vector>
#include <boost/math/constants/constants.hpp>

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


// Converts a given longitude and latitude to a position relative to the current longitude and latitude
std::vector<double> longLatToRelativePosition(double dest_longitude, double dest_latitude, double present_longitude, double present_latitude, double present_rotation){
	/*
	dest_longitude *= boost::math::double_constants::pi/180;
	dest_latitude *= boost::math::double_constants::pi/180;
	present_longitude *= boost::math::double_constants::pi/180;
	present_latitude *= boost::math::double_constants::pi/180;
	double distance = std::pow(std::sin((present_latitude-dest_latitude)/2),2)+std::cos(dest_latitude)*std::cos(present_latitude)*std::pow(std::sin((present_longitude-dest_longitude)),2);
	distance = 2.0*std::atan2(std::sqrt(distance),std::sqrt(1-distance));
	distance *= 637100.0;*/
	
    std::vector<double> nullvector;
    nullvector.push_back(0);
    nullvector.push_back(0);
    return nullvector;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "sb_waypoint_creator");
    ros::NodeHandle public_nh;
    ros::NodeHandle private_nh("~");
    ros::Rate loop_rate(5);

    Destinations destinations; // The destinations for the robot to go to

    // Initialize destination publisher

    ros::Publisher forward_pub = public_nh.advertise<geometry_msgs::Pose2D>("destination", 10);

    // Get at_destination (published by move_straight_line)
    bool at_destination = false;
    ros::Subscriber at_destination_sub =
        public_nh.subscribe<std_msgs::Bool>("move_straight_line/at_destination", 10,
                                            boost::function<void(std_msgs::Bool)>
                                            ([&](std_msgs::Bool at_dest){
                                                at_destination = at_dest.data;
                                            }));

    // Get present longitude and latitude from gps topic
    double present_longitude = 0;
    double present_latitude = 0;
    ros::Subscriber present_long_and_lat_sub = public_nh.subscribe<geometry_msgs::Point>("gps", 10, boost::function<void(geometry_msgs::Point)>([&](geometry_msgs::Point present_long_lat){
        present_longitude = present_long_lat.y;
        present_latitude = present_long_lat.x;
    }));

    // Get present rotation from pose2D
    double present_rotation;
    ros::Subscriber pose2D_sub = public_nh.subscribe<geometry_msgs::Pose2D>("pose2D", 10, boost::function<void(geometry_msgs::Pose2D)>([&](geometry_msgs::Pose2D pose2D){
        present_rotation = pose2D.theta;
    }));

// Get parameters
    // Whether or not the waypoints created will use some algorithm
    // to avoid obstacles; if false just go straight to all nodes on given path
    bool obstacle_avoidance = false;
    private_nh.getParam("avoid_obstacles", obstacle_avoidance);

    // Whether the coordinates given to the robot are in longitude/latitude (long/lat)
    // or position relative to where the robot started (relative_pos)
    std::string coordinate_type = "relative_pos";
    private_nh.getParam("coordinate_type", coordinate_type);

    // "path" is a vector of alternating x,y coordinates, that make up a
    // series of waypoints for the robot to move to.
        std::vector<double> path; // Path for the robot to follow {x,y,x,y, ...}
        private_nh.getParam("path", path);
        // Add all waypoints in "path"  as pose2D's in destinations
        for (int i = 0; i < path.size()/2; i++){
            geometry_msgs::Pose2D dest;
            // If coordinates are longitudes and latitudes, convert them to
            // relative positions before adding them to the list of destinations
            if (coordinate_type == "long/lat"){
                std::vector<double> rel_pos_coor = longLatToRelativePosition(path[i * 2], path[i * 2 + 1], present_longitude, present_latitude, present_rotation);
                dest.x = rel_pos_coor[0];
                dest.y = rel_pos_coor[1];
            } else { // if coordinate_type="relative_pos"
                dest.x = path[i * 2];
                dest.y = path[i * 2 + 1];
            }
            dest.theta = 0;
            destinations.add(dest);
        }

    int at_destination_counter = 0; // A ghetto solution to allow at_destination to publish twice,
                                    // but only go the next destination

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
