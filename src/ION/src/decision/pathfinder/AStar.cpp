#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <ION/decision/pathfinder/Pathfinder.h>


int main(int argc, char **argv){
	ros::init(argc, argv, "a_star");
	ros::NodeHandle private_nh("~");

	//CHANGE IT TO THE TOPICS YOU WANT
	/*ros::Subscriber mapSub = private_nh.subscribe<nav_msgs::OccupancyGrid>("map_topic", 1);
	ros::Subscriber startSub = private_nh.subscribe<geometry_msgs::Pose2D>("start_topic", 1);
	ros::Subscriber goalSub = private_nh.subscribe<geometry_msgs::Pose2D>("goal_topic", 1);
*/
	ros::Publisher pathPub = private_nh.advertise<geometry_msgs::Twist>("path", 1);
	
	ros::Rate loop_rate(20);

	while (ros::ok()){

		
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}
