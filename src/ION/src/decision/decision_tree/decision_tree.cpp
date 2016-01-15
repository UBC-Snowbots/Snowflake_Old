/*Created by Gareth Ellis - October 21, 2015 * /

/*
This is the basic decision tree for getting for point A to point B
It is(or was) designed for the Autonomous SnowPlow Competition (January 2015)
*/

/*
~~~README~~~
Definitions:
map : a 1D vector of 1's and 0's, where 1 is an obstacle and 0 is open space
coordinates : a std::pair<int, int>

"_data" is appended to the end of any variables internal to prevent conflicts between internal class
variables having the same name as external ones being passed in to internal class functions
*/

#define _USE_MATH_DEFINES

#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <utility>
#include <algorithm>
#include <math.h>
#include <cmath>
#include "decision_tree.hpp"



float CUTOFF = 10; // Distance away from the robot to search for obstacles
float POINTED_AT_DESTINATION_MIN_ANGLE = 5;
float RIGHT_ANGLE = 60;
float LEFT_ANGLE = -60;
float BACK_RIGHT = 135;
float BACK_LEFT = -135;
float AT_TARGET_RADIUS = 3;


/*~~~~~~~~~~~~~~~~~~~~~~~~~~ CLASSES ~~~~~~~~~~~~~~~~~~~~~~~~~~*/


command::command(float move_value, float turn_value) {
	move_value_data = move_value;
	turn_value_data = turn_value;
}
float command::move_val() { return move_value_data; };
float command::turn_val() { return turn_value_data; };


obstacle::obstacle(std::string type, std::string sector, int x_pos, int y_pos) {
	type_data = type;
	sector_data = sector;
	x_pos_data = x_pos;
	y_pos_data = y_pos;
};
obstacle obstacle::make_sector(std::string sector) {
	sector_data = sector;
	return obstacle(type_data, sector_data, x_pos_data, y_pos_data);
};
std::string obstacle::type() { return type_data; };
std::string obstacle::sector() { return sector_data; };
int obstacle::x() { return x_pos_data; };
int obstacle::y() { return y_pos_data; };


robot::robot(float width, float length, float rot_angle) {
	width_data = width;
	length_data = length;
	rot_angle_data = rot_angle;
};
void robot::set_rot_angle(float rot_angle) { rot_angle_data = rot_angle; };
float robot::width() { return width_data; };
float robot::length() { return length_data; };
float robot::rot_angle() { return rot_angle_data; };
std::pair<int, int> robot::front_right() { return front_right_data; };
std::pair<int, int> robot::front_left() { return front_left_data; };
std::pair<int, int> robot::back_right() { return back_right_data; };
std::pair<int, int> robot::back_left() { return back_left_data; };

/*~~~~~~~~~~~~~~~~~~~~~~~~~~ FUNCTIONS ~~~~~~~~~~~~~~~~~~~~~~~~~~*/

/*~~~~~~~~~~ Utility Functions ~~~~~~~~~~*/

//Prints all elements in a vector<char>
void printIntVector(std::vector<int> int_vector) {
	for (int i = 0; i < int_vector.size(); i++) {
		std::cout << int_vector[i] << std::endl;
	}
}


//Checks if a number is between two other numbers
bool isBetween(float num_1, float num_2, float num_in_between) {
	if ((num_1 < num_in_between < num_2) || (num_1 > num_in_between > num_2)) return true;
	else return false;
}

//Converts a given angle in degrees to a slope value
float degreesToSlope(float angle_in_degrees) {
	return tan((angle_in_degrees * 3.14159) / 180);
}

//Converts a given slope to an angle in degrees
float slopeToDegrees(float slope) {
	return (atan(slope) * (180 / 3.14159));
}


/*~~~~~~~~~~ Funtions To Get Information ~~~~~~~~~~*/


// Gets the file name of file_description from user
std::string getFileNameOf(std::string file_description) {
	std::string file_name;

	std::cout << "Please enter the full name of " << file_description << ": ";
	std::cin >> file_name;

	return file_name;
}


// Gets the map from a file !!! WILL NEED TO BE CHANGED FOR ROS INTEGRATION
vector_map getMap(std::string map_file_name, int width) {
	std::ifstream map_file(map_file_name);
	std::vector<int> map_data;
	int present_line;

	//Checks to make sure the map was succesfully opened, asks the user for another file if not
	if (!(map_file.is_open())) {
		throw std::runtime_error("File not found: " + map_file_name);
	}

	//Gets the integers from the file
	while (map_file >> present_line) {
		map_data.push_back(present_line);
	}

	return vector_map(map_data, width, map_data.size() / width);
}


// Gets the width of the map !!! WILL NEED TO BE CHANGED FOR ROS INTEGRATION
int getWidth() {
	int width;

	//Get the width from the user
	std::cout << "Please enter the map width: ";
	while (!(std::cin >> width)) {
		std::cout << "Invalid input, please enter the width as an integer: ";
		std::cin >> width;
	}

	return width;
}

// Gets the width of the robot !!! WILL NEED TO BE CHANGED FOR ROS INTEGRATION
int getRobotWidth() {
	int width;

	//Get the width from the user
	std::cout << "Please enter the Robot's width: ";
	while (!(std::cin >> width)) {
		std::cout << "Invalid input, please enter the width as an integer: ";
		std::cin >> width;
	}

	return width;
}

// Gets the present coordinates !!! WILL NEED TO BE CHANGED FOR ROS INTEGRATION
std::pair<int, int> getCurrentPos() {
	std::pair<int, int> present_pos = std::make_pair(0, 0);
	return present_pos;
}


// Gets the destination coordinates !!! WILL NEED TO BE CHANGED FOR ROS INTEGRATION
std::pair<int, int> getDestination() {
	std::pair<int, int> destination_pos = std::make_pair(10, 10);
	return destination_pos;
}


/*~~~~~~~~~~ Funtions To Process Information ~~~~~~~~~~*/

// Checks whether the robot has arrived at the destination
bool atDestination(std::pair<int, int> current_coor, std::pair<int, int> destination_coor) {
	//use pythagorean theorem to check if the distance between the current_coor and the destination_coor
	//is less then the pre-set tolerance
	return ((sqrt(pow((destination_coor.second - current_coor.second), 2)) + 
		pow((destination_coor.first - current_coor.first), 2)) 
		< AT_TARGET_RADIUS);
}

/*~~ Find Closest Obstacle ~~*/
// Gets the closest obstacle to the Robot

/*
How the area around the robot is searched for obstacles
+-------------------------------------------------+
|                                                 |
|                                       111111111 |
|                        1111111        300000002 |
|            11111       3000002        300000002 |
| 111        30002       3000002        300000002 |
| 3X2 +----> 30X02       300X002 +----> 3000X0002 |
| 444        30002       3000002        300000002 |
|            44444       3000002        300000002 |
|                        4444444        300000002 |
|                                       444444444 |
|                                                 |
|       0 - Checked nodes                         |
|       1 - Top Row                               |
|       2 - Right Column                          |
|       3 - Left Column                           |
|       4 - Bottom Row                            |
|       X - current_coor of robot                 |
|                                                 |
+-------------------------------------------------+
*/


//Gets which sector a given obstacle is in, from it's relative angle to the robot
obstacle addSector(obstacle obstacle_with_unkown_sector, robot snowflake, std::pair<int, int> current_coor) {
	std::pair<int, int> obstacle_coor = std::make_pair(obstacle_with_unkown_sector.x(), obstacle_with_unkown_sector.y());

	//Get relative angle of obstacle to robot
	//Get coordinates of obstacle RELATIVE to robot
	int relative_obstacle_x = obstacle_coor.first - current_coor.first;
	int relative_obstacle_y = obstacle_coor.second - current_coor.second;

	// Note: atan2 does normally take (y,x) however (x,y) is used here to swtich the frame of reference so the math works out
	float angle_robot_obstacle = atan2(relative_obstacle_x, relative_obstacle_y);

	//Convert angle to degrees
	angle_robot_obstacle = angle_robot_obstacle * (180 / M_PI);
	//Account for robots rotation
	float relative_obstacle_angle = angle_robot_obstacle - snowflake.rot_angle();

	//Check against angles to determine sector
	if ((relative_obstacle_angle >= 0) && (relative_obstacle_angle <= RIGHT_ANGLE)) {  // Obstacle is in front right
		return obstacle_with_unkown_sector.make_sector("front-right");
	}
	else if ((relative_obstacle_angle >= LEFT_ANGLE) && (relative_obstacle_angle < 0)) {  // Obstacle is in front left
		return obstacle_with_unkown_sector.make_sector("front-left");
	}
	else if ((relative_obstacle_angle >= RIGHT_ANGLE) && (relative_obstacle_angle <= BACK_RIGHT)) {  // Obstacle is in right
		return obstacle_with_unkown_sector.make_sector("right");
	}
	else if ((relative_obstacle_angle <= LEFT_ANGLE) && (relative_obstacle_angle >= BACK_LEFT)) {  // Obstacle is in left
		return obstacle_with_unkown_sector.make_sector("left");
	}
	else {                                                                                   // Obstacle is in back
		return obstacle_with_unkown_sector.make_sector("back");
	}

}

// Checks a given row for obstacles from a given x-min value to a given x-max value on a given map
// If found, returns the obstacle as: obstacle((pole or wall), "unkown", x_coor, y_coor) 
//                       else returns obstacle("none", "none", -1, -1)
obstacle checkRow(const map_interface &map, int row_num, int x_min, int x_max) {

	for (int i = 0; i < ((x_max - x_min) + 1); i++) {
		int x_coor = x_min + i;
		int y_coor = row_num;

		int present_node_val = 0;

		if (x_coor >= map.width()) {
			return obstacle("none", "none", -1, -1);
		}
		try
		{
			present_node_val = map.at(x_coor, y_coor);
		}
		// If out of bounds, there is no obstacle
		catch (const std::out_of_range & r_e)
		{
			return obstacle("none", "none", -1, -1);
		}

		if (present_node_val == 1) {
			return obstacle("pole", "unkown", x_coor, y_coor);
			break;
		}
		else if (present_node_val == 2) {
			return obstacle("wall", "unkown", x_coor, y_coor);
			break;
		}
	}
	return obstacle("none", "none", -1, -1);
}

// Checks a given column for obstacles from a given y-min value to a given y-max value on a given map
// If found, returns the obstacle as: obstacle((pole or wall), "unkown", x_coor, y_coor) 
//                       else returns obstacle("none", "none", -1, -1)
obstacle checkColumn(const map_interface &map, int column_num, int y_min, int y_max) {
	//If x_coor is greater then map width, node does not exist, so return no obstacle
	if (column_num >= map.width()) {
		return obstacle("none", "none", -1, -1);
	}
	
	for (int i = 0; i < ((y_max - y_min) + 1); i++) {
		int x_coor = column_num;
		int y_coor = y_min + i;
		
		int present_node_val = 0;

		try 
		{
			present_node_val = map.at(x_coor, y_coor);
		} 
		// If out of range, there is no obstacle
		catch (const std::out_of_range & r_e)
		{
			return obstacle("none", "none", -1, -1);
		}

		if (present_node_val == 1) {
			return obstacle("pole", "unkown", x_coor, y_coor);
			break;
		}
		else if (present_node_val == 2) {
			return obstacle("wall", "unkown", x_coor, y_coor);
			break;
		}
	}
	return obstacle("none", "none", -1, -1);
}

//Returns the closest obstacle to the present coordinates of the robot
obstacle getClosestObstacle(std::pair<int, int> current_coor, const map_interface &map, robot snowflake) {

	//Moves out from the center position of the robot, checking each row and column as it goes
	for (int i = 0; i < CUTOFF; i++) {
		int y_max = current_coor.second + i; // The max y value to checked on each column AND the top row to be checked
		int y_min = current_coor.second - i; // The min y value to checked on each column AND the bottom row to be checked
		int x_max = current_coor.first + i;  // The max x value to be checked on each row AND the left column to be checked
		int x_min = current_coor.first - i;  // The min x value to be checked on each row AND the right column to be checked

											 //Look for any obstacles in the top and bottom rows, left and right columns, at i nodes away from the robot
		obstacle possible_obstacles[4] = {
			checkRow(map, y_max, x_min, x_max),
			checkRow(map, y_min, x_min, x_max),
			checkColumn(map, x_max, y_min, y_max),
			checkColumn(map, x_min, y_min, y_max)
		};

		//Look through all obstacles in possible obstacles, and return the first obstacle found
		for (int obstacle = 0; obstacle < 4; obstacle++) {
			if (possible_obstacles[obstacle].type() != "none") {
				// If a obstacle, Adds a sector to said obstacle (checkRow and checkColumn return the obstacle with sector "unkonwn")
				return addSector(possible_obstacles[obstacle], snowflake, current_coor);
			}
		}
	}

	//No obstacles found
	return obstacle("none", "none", -1, -1);
}


//Returns the angle of the destination, relative to the angle that the robot is pointing
float get_relative_angle_robot_destination(std::pair<int, int> current_coor, std::pair<int, int> destination_coor, robot snowflake) {

	//Get x and y coordinates of destination RELATIVE to those of current_coor
	float relative_destination_x = destination_coor.first - current_coor.first;
	float relative_destination_y = destination_coor.second - current_coor.second;

	// Note: atan2 does normally take (y,x) however (x,y) is used here to swtich the frame of reference so the math works out
	float angle_robot_destination = atan2(relative_destination_x, relative_destination_y);

	//Convert angle to degrees, make relative to straight up being 0 degrees
	angle_robot_destination = angle_robot_destination * (180 / M_PI);
	float relative_angle_robot_destination = angle_robot_destination - snowflake.rot_angle();
	return relative_angle_robot_destination;
}



/*~~~~~~~~~~ Main Decision Tree ~~~~~~~~~~*/
// gets the next command to send to the robot
command getNextCommand(const map_interface &map, robot snowflake, std::pair<int, int> current_coor, std::pair<int, int> destination_coor) {
	if (!atDestination(current_coor, destination_coor)) {
		//The closest obstacle
		obstacle closest_obstacle = getClosestObstacle(current_coor, map, snowflake);
		//Angle to destination relative to robots current angle of rotation
		float destination_angle = get_relative_angle_robot_destination(current_coor, destination_coor, snowflake);

		if ((closest_obstacle.sector() == "back")) {  // Not facing obstacle
			if ((abs(destination_angle) < POINTED_AT_DESTINATION_MIN_ANGLE)) //Pointed at destination
			{
				return command(10, 0);  // command to move straight forward
			}
			else  //Not directly facing the destination
			{
				if (destination_angle < 0)  // Pointed to left of destination
				{
					return command(10, 10);  // command to move forward and turn right
				}
				else  // Pointed to right of destination
				{
					return command(10, -10);  // command to move forward and turn left
				}
			}
		}
		else if (closest_obstacle.type() == "pole") {
			if (closest_obstacle.sector() == "left") {
				return command(10, 0);  // command to move straight forward
			}
			else if (closest_obstacle.sector() == "right") {
				return command(10, 0);  // command to move straight forward
			}
			else { // obstacle_sector == front
				return command(4, -10);  // command to move forward SLIGHTLY and turn left
			}
		}
		else {  // Facing wall
			if ((closest_obstacle.sector() == "left") || (closest_obstacle.sector() == "front-left"))  // Wall to left of robot
			{
				return command(10, 10);  // command to move forward and turn right
			}
			else if ((closest_obstacle.sector() == "right") || (closest_obstacle.sector() == "front-right"))  // Wall to right of robot
			{
				return command(10, -10);  // command to move forward and turn left
			}
		}
	}

	//If this function ever reaches this point.... well it shouldn't happen
	return command(0, 0);
}

/*~~~~~~~~~~ Funtions To Publish Information ~~~~~~~~~~*/
// Publishes the command to the correct ROS topic in the correct format !!! WILL NEED TO BE CHANGED FOR ROS INTEGRATION
void publishCommand(command cmd) {

}



