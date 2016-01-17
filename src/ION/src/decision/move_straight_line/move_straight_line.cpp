#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>

#include <ION/decision/move_straight_line/MoveStraightLine.hpp>

using namespace ION::decision::move_straight_line;

geometry_msgs::Twist commandToTwist(const Command& command){
	geometry_msgs::Twist twist;
	twist.linear.x = command.dx;
	twist.angular.z = command.turn;
	return twist;
}

int main(int argc, char **argv){
	ros::init(argc, argv, "goForward");
	ros::NodeHandle n;

	ros::Publisher forward_pub = n.advertise<geometry_msgs::Twist>("driveCommand", 10);
	ros::Rate loop_rate(10);
	
	State initState;
	initState.position = {0,0};
	initState.direction = {1,0};
	State initDestination;
	initDestination.position = {4, 0};
	initDestination.direction = {1, 0};

	Mover mover(
		initState,
		initDestination,
		1,
		0.1);

	n.subscribe<geometry_msgs::Pose2D>("robot_pose", 10, boost::function<void(geometry_msgs::Pose2D)>([&](geometry_msgs::Pose2D pose){
		State currentState;
		currentState.position = arma::vec{pose.x, pose.y};
		currentState.direction = direction_vector(pose.theta);
		mover.setCurrentState(currentState);
	}));

	while (ros::ok()){
		Command command = mover.getCommand();
		geometry_msgs::Twist twistCommand = commandToTwist(command);
		forward_pub.publish(twistCommand);

		loop_rate.sleep();
	}
	
	return 0;
}
