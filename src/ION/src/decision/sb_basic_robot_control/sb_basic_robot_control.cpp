/* Author: Gareth Ellis - gareth.ellis0@gmail.com)
 * Purpose: Simplest possible "ai" for robot
 * Date: Jan 24, 2016
 * Run: rosrun PACKAGE_NAME_HERE basic_robot_control
 *
 * Notes:
 *  - This was created for worst case scenario. If you have anything better to use, and I mean ANYTHING,
 *  use that instead.
 *  - 3 types of goals, x,y, and theta
 *  - x and y type goals keep moving forward until the goal has been met
 *  - theta goals keep rotating until the goal has been meet
 *  - Goal in this node does NOT refer to an x,y coordinate, but rather to a threshold,
 *  such as x=4, or y=2 or z=3.14 (rotation)
 *  - Snowflake is the name of the robot for which this was developed
 */


#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <vector>
#include <math.h>

#define _USE_MATH_DEFINES

// Constant Definitions
const double AT_GOAL_MOVEMENT_TOLERANCE = 0.1; // The tolerance to consider a x or y goal reached
const double AT_GOAL_ROTATION_TOLERANCE = 0.04; // The tolerance to consider a theta goal reached
const double MOVE_SPEED = 1; // The speed at which the robot will move forwards and backwards
const double TURN_SPEED = 1; // The speed at which the robot will turn


// CLASSES / STRUCTS
//The robot, with x, y, and theta values
struct Robot {
    double x;     // X coordinate
    double y;     // Y coordinate
    double theta; // Rotation
} snowflake; // Create a robot named snowflake

//A goal, with type (x=1, y=2, theta=3) and value
struct Goal {
    int type;
    double value;
};

// FUNCTIONS 

// Returns true if robot is within tolerance of goal
bool atGoal(Robot snowflake, Goal goal){
    if (goal.type == 1){
        if (abs(snowflake.x - goal.value) < AT_GOAL_MOVEMENT_TOLERANCE){ return true; } 
        else { return false; }
    } else if (goal.type == 2){
        if (abs(snowflake.y - goal.value) < AT_GOAL_MOVEMENT_TOLERANCE){ return true; } 
        else { return false; }
    } else if (goal.type == 3){
        if (abs(snowflake.theta - goal.value) < AT_GOAL_ROTATION_TOLERANCE){ return true; } 
        else { return false; }
    } else { return false; }
}

// Figures out what command to send to the robot, given the robot and present goal
geometry_msgs::Twist getCommand(Robot snowflake, Goal goal){
    geometry_msgs::Twist command; // The command to return
    if (goal.type == 1){
        command.linear.x = MOVE_SPEED;
        return command;
    } else if (goal.type == 2){
        command.linear.x = MOVE_SPEED;
        return command;
    } else if (goal.type == 3){
        if ((snowflake.theta - goal.value) < M_PI){
            command.angular.z = TURN_SPEED;
        } else {
            command.angular.z = -TURN_SPEED;
        }
        return command;
    } else {
        return command;
    }
}


int main(int argc, char **argv){
    ros::init(argc, argv, "basic_robot_control");
    ros::NodeHandle nh;
    ros::Rate loop_rate(1);

    // Goals for the robot to go to
    std::vector<Goal> goals;
    // Add goals to vector
    Goal goal1;
    goal1.type = 1;
    goal1.value = 4;
    Goal goal2;
    goal2.type = 3;
    goal2.value = M_PI;
    Goal goal3;
    goal3.type = 1;
    goal3.value = 0;

    // Publishers and Subscribers
    // The topic commands are published on
    ros::Publisher command = nh.advertise<geometry_msgs::Twist>("command", 10);

    // Get map data (Not used right now, included to make it easy to make this more complicated in the future)
    /*
    ros::Subscriber map = nh.subscribe<nav_msgs::OccupancyGrid>("map", 10, boost::function<void(nav_msgs::OccupancyGrid)>([&](nav_msgs::OccupancyGrid map){
    
    }));
    */

    // Get pose data
    ros::Subscriber pose2d = nh.subscribe<geometry_msgs::Pose2D>("pose2D", 10, boost::function<void(geometry_msgs::Pose2D)>([&](geometry_msgs::Pose2D pose){
        snowflake.x = pose.x;
        snowflake.y = pose.y;
        snowflake.theta = fmod(pose.theta, (2 * M_PI));
    }));

    // Main Loop to run while node is running
    while (ros::ok()){
        // If on last goal and at destination, stop
        if (goals.size() == 1){
            geometry_msgs::Twist stop_command;
            command.publish(stop_command);
        }
        Goal present_goal = goals[0]; 
	    if (atGoal(snowflake, present_goal)){
            // Delete the first element in goals, so the next "present_goal" is the next one in the vector
            goals.erase(goals.begin());
        } else {
            command.publish(getCommand(snowflake, present_goal));
        }
        ros::spinOnce();
	    loop_rate.sleep();
    }
}
