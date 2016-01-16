#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>


/*
class moveStraight{
	
	private:
	ros::NodeHandle n;
	ros::Publisher move_straight;
	public:
	moveStraight(ros::NodeHandle &input) {
		n = input;
		move_straight = n.advertise<geometry_msgs::Twist>("driveCommand", 1000)
	}



}


int main(int argc, char** argv){
	ros::init(argc, argv, "insertDriverNameHere");
	ros::Nodehandle n;

	moveStraight go(n);

}
*/

struct State{
	double x, y, direction;
};

struct Command{
	double dx, dy, turn;
};

const double FORWARD_MOVE = 1;
const double STOP_THRESHOLD = 0.1;
const State DESIRED_STATE = {
	4,
	0,
	0
};

double getCorrectionAngle(const State& state){
	return -state.direction - atan(state.y/state.x);
}

Command getCommand(const State& state){
	//Assuming we'll always start less than four
	Command retCommand;
	double dx = state.x - DESIRED_STATE.x;
	retCommand.dx = dx > STOP_THRESHOLD ?
						std::copysign(FORWARD_MOVE, dx) :
						0;
	retCommand.dy = 0;
	retCommand.turn = getCorrectionAngle(state);
	return retCommand;
}

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

	State currentState;

	n.subscribe<geometry_msgs::Pose2D>("robot_pose", 10, boost::function<void(geometry_msgs::Pose2D)>([&](geometry_msgs::Pose2D pose){
		currentState.x = pose.x;
		currentState.y = pose.y;
		currentState.direction = pose.theta;
	}));

	while (ros::ok()){
		Command command = getCommand(currentState);
		geometry_msgs::Twist twistCommand = commandToTwist(command);
		forward_pub.publish(twistCommand);

		loop_rate.sleep();
	}
}


