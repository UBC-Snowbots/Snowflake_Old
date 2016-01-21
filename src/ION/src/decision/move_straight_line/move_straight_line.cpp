#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Bool.h>

#include <cstdio>

#include <ION/decision/move_straight_line/MoveStraightLine.hpp>

using namespace ION::decision::move_straight_line;

geometry_msgs::Twist commandToTwist(const Command& command){
	geometry_msgs::Twist twist;
	twist.linear.x = command.dx;
	twist.angular.z = command.turn;
	return twist;
}

int main(int argc, char **argv){
	ros::init(argc, argv, "move_straight_line");
	ros::NodeHandle public_nh;
	
	ros::NodeHandle private_nh("~");

	ros::Publisher forward_pub = private_nh.advertise<geometry_msgs::Twist>("command", 10);
	ros::Publisher at_destination_pub = private_nh.advertise<std_msgs::Bool>("at_destination", 10, true);
	ros::Rate loop_rate(20);
	
	double stop_threshold;
	if(!private_nh.getParam("stop_threshold", stop_threshold)){
		stop_threshold = 0.1;
	}
	double move_speed;
	if(!private_nh.getParam("move_speed", move_speed)){
		move_speed = 1;
	}
	
	State initState;
	initState.position = {0,0};
	initState.direction = {1,0};
	State initDestination;
	initDestination.position = {4, 0};
	initDestination.direction = {1, 0};

	Mover mover(
		initState,
		initDestination,
		move_speed,
		stop_threshold);
	
	bool have_pose = false, have_destination = false; // flag to wait on first pose update

	ros::Subscriber pose2d = public_nh.subscribe<geometry_msgs::Pose2D>("pose2D", 10, boost::function<void(geometry_msgs::Pose2D)>([&](geometry_msgs::Pose2D pose){
		have_pose = true;
		
		State currentState;
		currentState.position = arma::vec{pose.x, pose.y};
		currentState.direction = direction_vector_from_north(pose.theta);
		mover.setCurrentState(currentState);
	}));
	
	ros::Subscriber dest = public_nh.subscribe<geometry_msgs::Pose2D>("destination", 10, boost::function<void(geometry_msgs::Pose2D)>([&](geometry_msgs::Pose2D pose){
		have_destination = true;
		State dest;
		dest.position = arma::vec{pose.x, pose.y};
		dest.direction = direction_vector_from_north(pose.theta);
		mover.setDestination(dest);
	}));
	
	bool was_at_destination = false;

	while (ros::ok()){
		if(have_pose && have_destination){
			Command command = mover.getCommand();
			geometry_msgs::Twist twistCommand = commandToTwist(command);
			forward_pub.publish(twistCommand);
			
			bool at_destination = mover.atDestination();
			if(at_destination != was_at_destination){
				was_at_destination = at_destination;
				std_msgs::Bool at_dest;
				at_dest.data = at_destination;
				at_destination_pub.publish(at_dest);
			}
		}
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}
