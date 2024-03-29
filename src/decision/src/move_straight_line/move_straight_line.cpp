#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

#include <MoveStraightLine.hpp>

using namespace decision::move_straight_line;

geometry_msgs::Twist commandToTwist(const Command& command){
	geometry_msgs::Twist twist;
	twist.linear.x = command.dx;
	twist.angular.z = -command.turn;
	return twist;
}

int main(int argc, char **argv){
	ros::init(argc, argv, "move_straight_line");
	ros::NodeHandle public_nh;

	ros::NodeHandle private_nh("~");

	ros::Publisher forward_pub = private_nh.advertise<geometry_msgs::Twist>("command", 10);
	ros::Publisher at_destination_pub = private_nh.advertise<std_msgs::Bool>("at_destination", 1, true);
	ros::Rate loop_rate(20);

	Mover mover;{
		double stop_threshold;
		if(private_nh.getParam("stop_threshold", stop_threshold)){
			mover.setStopThreshold(stop_threshold);
		}
		double move_speed;
		if(private_nh.getParam("move_speed", move_speed)){
			mover.setMoveSpeed(move_speed);
		}
		double explicit_turn_threshold;
		if(private_nh.getParam("explicit_turn_threshold", explicit_turn_threshold)){
			mover.setExplicitTurnThreshold(explicit_turn_threshold);
		}
	}
	ROS_INFO("move_straight_line started\n");
	ROS_INFO("move_speed: %f\n", mover.getMoveSpeed());
	ROS_INFO("stop_threshold: %f\n", mover.getStopThreshold());
	ROS_INFO("explicit_turn_threshold: %f\n", mover.getExplicitTurnThreshold());

	bool have_pose = false, have_destination = false; // flag to wait on first pose update

	ros::Subscriber pose2d = public_nh.subscribe<nav_msgs::Odometry>("odom", 10, boost::function<void(nav_msgs::Odometry)>([&](nav_msgs::Odometry odom){
		have_pose = true;

		State currentState;
		currentState.position = arma::vec{odom.pose.pose.position.x, -odom.pose.pose.position.y};
		//TODO: Make sure the angle is actually correct
		currentState.direction = direction_vector_from_north(- 2 * asin(odom.pose.pose.orientation.z));
		mover.setCurrentState(currentState);
	}));

	ros::Subscriber dest = public_nh.subscribe<geometry_msgs::Pose2D>("destination", 10, boost::function<void(geometry_msgs::Pose2D)>([&](geometry_msgs::Pose2D pose){
		have_destination = true;
		State dest;
		dest.position = arma::vec{pose.x, pose.y};
		dest.direction = direction_vector_from_north(-pose.theta);
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
