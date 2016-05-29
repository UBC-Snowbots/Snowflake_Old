/*
    - This node is intended to take a series of longitudes and latitudes as params, and then publish them
    sequentially, publishing the next one when the robot is located at the present one, allowing the robot
    to navigate to a series of long/lat waypoints
    - NOTE: All waypoints are a lat/lon coordinate, they are converted before they are advertised
    Author: Vincent Yuan / Gareth Ellis
    Date: May 16th, 2016
*/

#include "ros/ros.h"
#include "math.h"
#include "sb_messages/gps.h"
#include "geometry_msgs/Pose2D.h"


using namespace std;

// Finds the distancce between two waypoints
double distanceBetweenWaypoints(sb_messages::gps waypoint1, sb_messages::gps waypoint2){
    double lat1 = waypoint1.lat*M_PI/180;
    cout << "lat1: " << lat1 << endl;
    double lon1 = waypoint1.lon*M_PI/180;
    cout << "lon1: " << lat1 << endl;
    double lat2 = waypoint2.lat*M_PI/180;
    cout << "lat2: " << lat1 << endl;
    double lon2 = waypoint2.lon*M_PI/180;
    cout << "lon2: " << lat1 << endl;

    //Haversine:
    double distance = pow(sin((lat2 - lat1)/2),2) +
                        cos(lat1)*cos(lat2)*pow(sin((lon2 - lon1)/2),2);
    distance = 2*atan2(sqrt(distance),sqrt(1-distance));
    distance *= 6371000; //Earth radius in m
    return distance;
}

double bearingBetweenWaypoints(sb_messages::gps waypoint1, sb_messages::gps waypoint2){
    double lat1 = waypoint1.lat*M_PI/180;
    cout << "lat1: " << lat1 << endl;
    double lon1 = waypoint1.lon*M_PI/180;
    cout << "lon1: " << lat1 << endl;
    double lat2 = waypoint2.lat*M_PI/180;
    cout << "lat2: " << lat1 << endl;
    double lon2 = waypoint2.lon*M_PI/180;
    cout << "lon2: " << lat1 << endl;
    double delta_lon = lon1 - lon2;
    double y = sin(delta_lon) * cos(lat2);
    double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(delta_lon);
    return atan2(y, x);
}

// A class designed to broadcast waypoints sequentially, with the next one being broadcast when the
// robot has arrived at the previous one
class gpsManager {
    public:
        gpsManager();
        void gpsCallBack(const sb_messages::gps::ConstPtr& gps);
    private:
        void publishNextWaypoint();
        double distanceToNextWayPoint();
        void parseWayPoints();
        void printWaypoints();
                    // Where the robot started
        sb_messages::gps origin;
                    // Present location
        sb_messages::gps present_location;
                    //  The  list of all waypoints to go to, in reverse order
        vector<sb_messages::gps> waypoints;
                    //  The tolerance for being at a waypoint
        double tolerance;
        ros::Publisher waypoint_pub;
        ros::Subscriber gps_sub;
        //  If true, a gps message has been received, otherwise no gps messages have been received
        bool gps_message_recieved;
        vector<double> waypoints_raw;
};

gpsManager::gpsManager(){
    ros::NodeHandle public_nh;
    ros::NodeHandle private_nh("~");
    // Get Params
    private_nh.getParam("tolerance", tolerance);
    private_nh.getParam("waypoints", waypoints_raw);
    if (waypoints_raw.size() <= 0){
        cout << "ERROR: Did you specify waypoints for this node?" << endl
                << "CHECK THAT YOU SET ALL PARAMS" << endl;
    }
    this->parseWayPoints();
    // Setup subscibers
    gps_sub = public_nh.subscribe("gps_topic", 1, &gpsManager::gpsCallBack, this);
    // Setup Publishers
    string topic = public_nh.resolveName("waypoint");
    uint32_t queue_size = 1;
    ros::Rate loop_rate(5);
    waypoint_pub = public_nh.advertise<geometry_msgs::Pose2D>(topic, queue_size);
    gps_message_recieved = false;
}

void gpsManager::parseWayPoints(){
    for (int i = 0; i < waypoints_raw.size(); i += 2){
        sb_messages::gps waypoint;
        waypoint.lat = waypoints_raw[i];
        waypoint.lon = waypoints_raw[i + 1];
        waypoint.head = 0;
        waypoints.push_back(waypoint);
    }
    cout << "Waypoints size: " << waypoints.size() << endl;
}

void gpsManager::printWaypoints(){
    for (int i = 0; i < waypoints.size(); i++){
        cout << "Lat: " << waypoints[i].lat << endl
            << "Lon: " << waypoints[i].lon << endl
            << "Head: " << waypoints[i].head << endl
            << "~~~~~~~~~~~~~~~~~~~~~~~~~~" << endl;
    }
    cout << "!!!!!!!!!!" << endl;
}

void gpsManager::gpsCallBack(const sb_messages::gps::ConstPtr& gps){
        // Set the origin only if it has not been set already (ie. when this node is first started)
    present_location = *gps;
    if (!gps_message_recieved){
        origin = *gps;
        gps_message_recieved = true;
    }
    if (distanceToNextWayPoint() <= tolerance){
        waypoints.pop_back();
    }
    this->publishNextWaypoint();
}

// Publishes the next waypoint to go to, in the GLOBAL frame of the robot (ie. as a pose2D)
void gpsManager::publishNextWaypoint(){
    sb_messages::gps current_waypoint = waypoints[waypoints.size() - 1];
    // Convert current waypoint to robot's prespective
    double distance = distanceBetweenWaypoints(origin, current_waypoint);
    cout << "Distance: " << distance << endl;
    double theta = bearingBetweenWaypoints(origin, current_waypoint);
    cout << "Theta: " << theta << endl;
    // x and y values from gps presepective
    double gps_x = cos(theta) * distance;
    double gps_y = sin(theta) * distance;
    // Local coordinates determined via a matrix algebra rotation matrix
    geometry_msgs::Pose2D present_waypoint_pose; // A pose representing the next waypoint to go to
    present_waypoint_pose.x = cos(origin.head) * gps_x + sin(origin.head) * gps_x;
    present_waypoint_pose.y = -sin(origin.head) * gps_y + cos(origin.head) * gps_y;
    present_waypoint_pose.theta = 0;
// Publish the waypoint
    waypoint_pub.publish(present_waypoint_pose);
}

double gpsManager::distanceToNextWayPoint(){
    sb_messages::gps current_waypoint = waypoints[waypoints.size() - 1];
    return distanceBetweenWaypoints(present_location, current_waypoint);
}


int main(int argc, char **argv){
  ros::init(argc, argv, "gps_decision");

  gpsManager gps_manager;

  ros::spin();

  return 0;
}
