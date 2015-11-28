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


float CUTOFF = 10; // Distance away from the robot to search for obstacles
float POINTED_AT_DESTINATION_MIN_ANGLE = 5;
float RIGHT_ANGLE = 60;
float LEFT_ANGLE = -60;
float BACK_RIGHT = 135;
float BACK_LEFT = -135;
float AT_TARGET_RADIUS = 3;


/*~~~~~~~~~~~~~~~~~~~~~~~~~~ CLASSES ~~~~~~~~~~~~~~~~~~~~~~~~~~*/

//Command is a command to send the the robot where:
//  - Value is the value of a given command, ie) 1 meter (1), 30 degrees (30), 1 meter and 30 degrees (1,30)
class command {
	int move_value_data = 0;
	int turn_value_data = 0;
	
public:
	command(float move_value, float turn_value) {
		move_value_data = move_value;
		turn_value_data = turn_value;
	}
	float move_val() { return move_value_data; };
	float turn_val() { return turn_value_data; };
};


// Obstacle represents an obstacle with:
//  - type ("wall", "pole", "none")
//  - sector ("front-right", "front-left", "right", "left", "back", "none", "unkown")
//  - x coordinate (-1 if no obstacle)
//  - y coordinate (-1 if no obstacle)
class obstacle {
	std::string type_data;
	std::string sector_data;
	int x_pos_data;
	int y_pos_data;

public:
	obstacle(std::string type, std::string sector, int x_pos, int y_pos) {
		type_data = type;
		sector_data = sector;
		x_pos_data = x_pos;
		y_pos_data = y_pos;
	};
	obstacle make_sector(std::string sector) {
		sector_data = sector;
		return obstacle(type_data, sector_data, x_pos_data, y_pos_data);
	};
	std::string type() { return type_data; };
	std::string sector() { return sector_data; };
	int x() { return x_pos_data; };
	int y() { return y_pos_data; };
};


/*This class represents the robot with:
- width
- length
- present angle (relative to initial angle) (in degrees)
- coordinates of each corner, relative to the center of robot as 0
- 5 sectors, front_left, front_right, right, left, back
*/
class robot {
	float width_data;
	float length_data;
	float rot_angle_data;
	std::pair<int, int> front_right_data = std::make_pair(int(width_data / 2), int(length_data / 2));
	std::pair<int, int> front_left_data = std::make_pair(int(width_data / -2), int(length_data / 2));
	std::pair<int, int> back_right_data = std::make_pair(int(width_data / 2), int(length_data / -2));
	std::pair<int, int> back_left_data = std::make_pair(int(width_data / -2), int(length_data / -2));

public:
	robot(float width, float length, float rot_angle) {
		width_data = width;
		length_data = length;
		rot_angle_data = rot_angle;
	};
	void set_rot_angle(float rot_angle) { rot_angle = rot_angle; };
	float width() { return width_data; };
	float length() { return length_data; };
	float rot_angle() { return rot_angle_data; };
	std::pair<int, int> front_right() { return front_right_data; };
	std::pair<int, int> front_left() { return front_left_data; };
	std::pair<int, int> back_right() { return back_right_data; };
	std::pair<int, int> back_left() { return back_left_data; };
};

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

// Returns the value of a map node on a given map at a given x and y postion
// LARGELY DEPRECEATED RIGHT NOW, CHECK FOR DEPENDENCIES ON IT AND REMOVE IF POSSIBLE
int mapValAt(std::vector<int> &map, int map_width, int x_coor, int y_coor) {
	return (map[y_coor * map_width + x_coor]);
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
std::vector<int> getMap(std::string map_file_name) {
	std::ifstream map_file(map_file_name);
	std::vector<int> map;
	int present_line;

	//Checks to make sure the map was succesfully opened, asks the user for another file if not
	while (!(map_file.is_open())) {
		std::cout << "File not found, please enter another file name: ";
		std::cin >> map_file_name;
		map_file.open(map_file_name);
	}

	//Gets the integers from the file
	while (map_file >> present_line) {
		map.push_back(present_line);
	}

	return map;
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

// Checks whether the robot has arrived at the target
// (Whether or not the target is between the 4 corners of the robot)
/*
bool atTarget(robot snowflake, std::pair<int, int> current_coor, std::pair<int, int> destination_coor) {

	int robot_x_coors[4] = { // List of x coordinates for all corners of robot
		(snowflake.front_left().first + current_coor.first),
		(snowflake.front_right().first + current_coor.first),
		(snowflake.back_left().first + current_coor.first),
		(snowflake.back_right().first + current_coor.first)
	};
	int robot_y_coors[4] = { // List of y coordinates for all corners of robot
		(snowflake.front_left().second + current_coor.second),
		(snowflake.front_right().second + current_coor.second),
		(snowflake.back_left().second + current_coor.second),
		(snowflake.back_right().second + current_coor.second)
	};
	int robot_max_x = current_coor.first + *std::max_element(robot_x_coors, robot_x_coors + 4);     // The max x value out of all corners of the robot
	int robot_max_y = current_coor.second + *std::max_element(robot_y_coors, robot_y_coors + 4); // The max y value out of all corners of the robot
	int robot_min_x = current_coor.first + *std::min_element(robot_x_coors, robot_x_coors + 4);  // The min x value out of all corners of the robot
	int robot_min_y = current_coor.second + *std::min_element(robot_y_coors, robot_y_coors + 4); // The min y value out of all corners of the robot

																								 // Check if the the destination x and y coordinates are between the current max x and y coordinates 
																								 //  (out of the 4 corners of the robot)
	if ((isBetween(robot_max_x, robot_min_x, destination_coor.first)) &&
		(isBetween(robot_max_y, robot_min_y, destination_coor.second))) {
		return true;
	}
	else {
		return false;
	}
}
*/

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
obstacle checkRow(std::vector<int> &map, int map_width, int row_num, int x_min, int x_max) {

	for (int i = 0; i < ((x_max - x_min) + 1); i++) {
		int x_coor = x_min + i;
		int y_coor = row_num;

		int present_node_val = 0;

		if (x_coor >= map_width) {
			return obstacle("none", "none", -1, -1);
		}
		try
		{
			present_node_val = map.at((y_coor * map_width) + x_coor);
		}
		// If an error is thrown here, probably trying to access node off of the map, so return no obstacle
		catch (const std::exception & r_e)
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
obstacle checkColumn(std::vector<int> &map, int map_width, int column_num, int y_min, int y_max) {
	//If x_coor is greater then map width, node does not exist, so return no obstacle
	if (column_num >= map_width) {
		return obstacle("none", "none", -1, -1);
	}
	
	for (int i = 0; i < ((y_max - y_min) + 1); i++) {
		int x_coor = column_num;
		int y_coor = y_min + i;
		
		int present_node_val = 0;

		try 
		{
			present_node_val = map.at((y_coor * map_width) + x_coor);
		} 
		// If an error is thrown here, probably trying to access node off of the map, so return no obstacle
		catch (const std::exception & r_e)
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
obstacle getClosestObstacle(std::pair<int, int> current_coor, std::vector<int> &map, int map_width, robot snowflake) {

	//Moves out from the center position of the robot, checking each row and column as it goes
	for (int i = 0; i < CUTOFF; i++) {
		int y_max = current_coor.second + i; // The max y value to checked on each column AND the top row to be checked
		int y_min = current_coor.second - i; // The min y value to checked on each column AND the bottom row to be checked
		int x_max = current_coor.first + i;  // The max x value to be checked on each row AND the left column to be checked
		int x_min = current_coor.first - i;  // The min x value to be checked on each row AND the right column to be checked

											 //Look for any obstacles in the top and bottom rows, left and right columns, at i nodes away from the robot
		obstacle possible_obstacles[4] = {
			checkRow(map, map_width, y_max, x_min, x_max),
			checkRow(map, map_width, y_min, x_min, x_max),
			checkColumn(map, map_width, x_max, y_min, y_max),
			checkColumn(map, map_width, x_min, y_min, y_max)
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
command getNextCommand(std::vector<int> &map, int map_width, robot snowflake, std::pair<int, int> current_coor, std::pair<int, int> destination_coor) {
	if (!atDestination(current_coor, destination_coor)) {
		//The closest obstacle
		obstacle closest_obstacle = getClosestObstacle(current_coor, map, map_width, snowflake);
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
// Publishs the command to the correct ROS topic in the correct format !!! WILL NEED TO BE CHANGED FOR ROS INTEGRATION
void publishCommand(command cmd) {

}

// This Definition of main is not used for unit testing, catch provides it's own main function
/*
int main() {
robot snowflake(10, 10, 0); //The robot initally with length 10, width 10 and 0 degrees of rotation

std::string map_file_name = getFileNameOf("the map file to be analyzed");
std::vector<int> map = getMap(map_file_name);
int map_width = getWidth();
int robot_width = getRobotWidth();
std::pair<int, int> current_coor = getCurrentPos();
std::pair<int, int> destination_coor = getDestination();

command cmd_to_send = getNextCommand(map, map_width, snowflake, current_coor, destination_coor);

publishCommand(cmd_to_send);

return 0;
}
*/





/*~~~~~~~~~~ Unit Testing ~~~~~~~~~~*/

#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main() - only do this in one cpp file
#include "catch.hpp"


//EXAMPLE
/*
unsigned int Factorial(unsigned int number) {
return number <= 1 ? number : Factorial(number - 1)*number;
}

TEST_CASE("Factorials are computed", "[factorial]") {
REQUIRE(Factorial(0) == 1);
REQUIRE(Factorial(2) == 2);
REQUIRE(Factorial(3) == 6);
REQUIRE(Factorial(10) == 3628800);
}
*/


TEST_CASE("degreesToSlope", "[degreesToSlope]") {
	REQUIRE(degreesToSlope(0) == Approx(0));
	REQUIRE(degreesToSlope(45) == Approx(1.0));
	REQUIRE(degreesToSlope(180) == Approx(0));
	REQUIRE(degreesToSlope(234) == Approx(1.3762).epsilon(10));
	REQUIRE(degreesToSlope(360) == Approx(0));
}

TEST_CASE("Relative angle of robot to destination", "[get_relative_angle_robot_destination]") {
	REQUIRE(get_relative_angle_robot_destination(std::make_pair(0, 0), std::make_pair(1, 1), robot(1, 1, 0)) == Approx(45));
	REQUIRE(get_relative_angle_robot_destination(std::make_pair(1, 1), std::make_pair(0, 0), robot(1, 1, 0)) == Approx(-135));
	REQUIRE(get_relative_angle_robot_destination(std::make_pair(0, 0), std::make_pair(0, 0), robot(1, 1, 0)) == Approx(0));
	REQUIRE(get_relative_angle_robot_destination(std::make_pair(0, 0), std::make_pair(1, 1), robot(1, 1, 45)) == Approx(0));
	REQUIRE(get_relative_angle_robot_destination(std::make_pair(0, 0), std::make_pair(1, 1), robot(1, 1, 90)) == Approx(-45));
	REQUIRE(get_relative_angle_robot_destination(std::make_pair(1, 1), std::make_pair(2, 2), robot(1, 1, 0)) == Approx(45));
	REQUIRE(get_relative_angle_robot_destination(std::make_pair(0, 1), std::make_pair(1, 2), robot(1, 1, 0)) == Approx(45));
	REQUIRE(get_relative_angle_robot_destination(std::make_pair(10, 10), std::make_pair(5, 5), robot(1, 1, 0)) == Approx(-135)); // To top right of destination
	REQUIRE(get_relative_angle_robot_destination(std::make_pair(0, 10), std::make_pair(5, 5), robot(1, 1, 0)) == Approx(135)); // To top left of destination
	REQUIRE(get_relative_angle_robot_destination(std::make_pair(10, 0), std::make_pair(5, 5), robot(1, 1, 0)) == Approx(-45)); // To bottom right of destination
	REQUIRE(get_relative_angle_robot_destination(std::make_pair(0, 0), std::make_pair(5, 5), robot(1, 1, 0)) == Approx(45)); // To bottom left of destination
}

TEST_CASE("Map Value at Position", "[mapValAt]") {
	std::vector<int> testmap1 = getMap("map1_width_6.map");
	REQUIRE(mapValAt(testmap1, 6, 0, 0) == 1);
	REQUIRE(mapValAt(testmap1, 6, 1, 1) == 0);
	REQUIRE(mapValAt(testmap1, 6, 2, 1) == 0);
	REQUIRE(mapValAt(testmap1, 6, 4, 2) == 1);
	REQUIRE(mapValAt(testmap1, 6, 5, 3) == 1);
}

TEST_CASE("slopeToDegrees", "[slopeToDegrees]") {
	REQUIRE(slopeToDegrees(1) == Approx(45));
	REQUIRE(slopeToDegrees(.5) == Approx(26.57).epsilon(10));
	REQUIRE(slopeToDegrees(.25) == Approx(14.036).epsilon(10));
	REQUIRE(slopeToDegrees(1) == Approx(45));

}

TEST_CASE("addSector", "[addSector]") {
	REQUIRE(addSector(obstacle("pole", "unkown", 2, 2), robot(1, 1, 0), std::make_pair(1, 1)).sector() == "front-right");  // Obstacle in front-right
	REQUIRE(addSector(obstacle("pole", "unkown", 5, 5), robot(1, 1, 0), std::make_pair(8, 1)).sector() == "front-left");  // Obstacle in front-left
	REQUIRE(addSector(obstacle("pole", "unkown", 5, 5), robot(1, 1, 0), std::make_pair(7, 5)).sector() == "left");  // Obstacle in left
	REQUIRE(addSector(obstacle("pole", "unkown", 5, 5), robot(1, 1, 0), std::make_pair(3, 5)).sector() == "right");  // Obstacle in right
	REQUIRE(addSector(obstacle("pole", "unkown", 5, 5), robot(1, 1, 0), std::make_pair(5, 8)).sector() == "back");  // Obstacle in back
	REQUIRE(addSector(obstacle("pole", "unkown", 5, 5), robot(1, 1, 180), std::make_pair(5, 2)).sector() == "back");  // Obstacle in back, robot turned
	REQUIRE(addSector(obstacle("pole", "unkown", 5, 5), robot(1, 1, 180), std::make_pair(2, 5)).sector() == "left");  // Obstacle in left, robot turned
}

TEST_CASE("obstacle functions", "[obstacle]") {
	// sector()
	obstacle test_obstacle_1 = obstacle("pole", "left", 0, 0);
	REQUIRE(test_obstacle_1.sector() == "left");

	// make_sector()
	obstacle test_obstacle_2 = obstacle("pole", "unkown", 0, 0);
	REQUIRE(test_obstacle_2.make_sector("left").sector() == "left");
}

TEST_CASE("checkColumn", "[checkColumn]") {
	std::vector<int> testmap1 = getMap("map2_width_10.map");
	REQUIRE(checkColumn(testmap1, 10, 0, 2, 5).sector() == "unkown");
	REQUIRE(checkColumn(testmap1, 10, 0, 2, 5).type() == "wall");
	REQUIRE(checkColumn(testmap1, 10, 0, 2, 5).x() == 0);
	REQUIRE(checkColumn(testmap1, 10, 0, 2, 5).y() == 2);

	REQUIRE(checkColumn(testmap1, 10, 1, 1, 5).sector() == "unkown");
	REQUIRE(checkColumn(testmap1, 10, 1, 1, 5).type() == "pole");
	REQUIRE(checkColumn(testmap1, 10, 1, 1, 5).x() == 1);
	REQUIRE(checkColumn(testmap1, 10, 1, 1, 5).y() == 2);

	//Checking a column outside the map
	REQUIRE(checkColumn(testmap1, 10, 50, 1, 5).sector() == "none");
	REQUIRE(checkColumn(testmap1, 10, 50, 1, 5).type() == "none");
	REQUIRE(checkColumn(testmap1, 10, 50, 1, 5).x() == -1);
	REQUIRE(checkColumn(testmap1, 10, 50, 1, 5).y() == -1);

	//Checking a column just barely outside the map
	REQUIRE(checkColumn(testmap1, 10, 10, 1, 5).sector() == "none");
	REQUIRE(checkColumn(testmap1, 10, 10, 1, 5).type() == "none");
	REQUIRE(checkColumn(testmap1, 10, 10, 1, 5).x() == -1);
	REQUIRE(checkColumn(testmap1, 10, 10, 1, 5).y() == -1);
}

TEST_CASE("checkRow", "[checkRow]") {
	std::vector<int> testmap1 = getMap("map2_width_10.map");
	REQUIRE(checkRow(testmap1, 10, 0, 1, 5).sector() == "unkown");
	REQUIRE(checkRow(testmap1, 10, 0, 1, 5).type() == "wall");
	REQUIRE(checkRow(testmap1, 10, 0, 1, 5).x() == 1);
	REQUIRE(checkRow(testmap1, 10, 0, 1, 5).y() == 0);

	REQUIRE(checkRow(testmap1, 10, 3, 1, 5).sector() == "unkown");
	REQUIRE(checkRow(testmap1, 10, 3, 1, 5).type() == "pole");
	REQUIRE(checkRow(testmap1, 10, 3, 1, 5).x() == 4);
	REQUIRE(checkRow(testmap1, 10, 3, 1, 5).y() == 3);

	REQUIRE(checkRow(testmap1, 10, 4, 1, 5).sector() == "none");
	REQUIRE(checkRow(testmap1, 10, 4, 1, 5).type() == "none");
	REQUIRE(checkRow(testmap1, 10, 4, 1, 5).x() == -1);
	REQUIRE(checkRow(testmap1, 10, 4, 1, 5).y() == -1);

	//Try row just outside the map
	REQUIRE(checkRow(testmap1, 10, 7, 1, 5).sector() == "none");
	REQUIRE(checkRow(testmap1, 10, 7, 1, 5).type() == "none");
	REQUIRE(checkRow(testmap1, 10, 7, 1, 5).x() == -1);
	REQUIRE(checkRow(testmap1, 10, 7, 1, 5).y() == -1);

}

TEST_CASE("getClosestObstacle", "[getClosestObstacle]") {
	std::vector<int> testmap1 = getMap("map2_width_10.map");
	robot snowflake = robot(1, 1, 0);

	// Pole directly beside the right side of robot
	REQUIRE(getClosestObstacle(std::make_pair(3, 3), testmap1, 10, snowflake).sector() == "right");
	REQUIRE(getClosestObstacle(std::make_pair(3, 3), testmap1, 10, snowflake).type() == "pole");
	REQUIRE(getClosestObstacle(std::make_pair(3, 3), testmap1, 10, snowflake).x() == 4);
	REQUIRE(getClosestObstacle(std::make_pair(3, 3), testmap1, 10, snowflake).y() == 3);

	// Pole directly to left of robot, two nodes away
	REQUIRE(getClosestObstacle(std::make_pair(6, 3), testmap1, 10, snowflake).sector() == "left");
	REQUIRE(getClosestObstacle(std::make_pair(6, 3), testmap1, 10, snowflake).type() == "pole");
	REQUIRE(getClosestObstacle(std::make_pair(6, 3), testmap1, 10, snowflake).x() == 4);
	REQUIRE(getClosestObstacle(std::make_pair(6, 3), testmap1, 10, snowflake).y() == 3);

	// Pole in the front-left sector
	REQUIRE(getClosestObstacle(std::make_pair(5, 2), testmap1, 10, snowflake).sector() == "front-left");
	REQUIRE(getClosestObstacle(std::make_pair(5, 2), testmap1, 10, snowflake).type() == "pole");
	REQUIRE(getClosestObstacle(std::make_pair(5, 2), testmap1, 10, snowflake).x() == 4);
	REQUIRE(getClosestObstacle(std::make_pair(5, 2), testmap1, 10, snowflake).y() == 3);

	// Wall directly to right (NOTE: Due to the way that checkColumn works (it starts at the top of a column and goes down))
	// the first "wall" section found is in the front-right section
	REQUIRE(getClosestObstacle(std::make_pair(8, 2), testmap1, 10, snowflake).sector() == "front-right");
	REQUIRE(getClosestObstacle(std::make_pair(8, 2), testmap1, 10, snowflake).type() == "wall");
	REQUIRE(getClosestObstacle(std::make_pair(8, 2), testmap1, 10, snowflake).x() == 9);
	REQUIRE(getClosestObstacle(std::make_pair(8, 2), testmap1, 10, snowflake).y() == 3);
}

TEST_CASE("atDestination", "[atDestination]") {
	REQUIRE(atDestination(std::make_pair(1, 1), std::make_pair(5, 5)) == false);
	REQUIRE(atDestination(std::make_pair(2, 3), std::make_pair(3, 3)) == true);
	REQUIRE(atDestination(std::make_pair(10, 10), std::make_pair(5, 5)) == false);
}

TEST_CASE("getNextCommand", "[getNextCommand]") {
	std::vector<int> testmap3 = getMap("map3_width_10.map");
	REQUIRE(getNextCommand(testmap3, 512, robot(1,1,0), std::make_pair(,70),))
}