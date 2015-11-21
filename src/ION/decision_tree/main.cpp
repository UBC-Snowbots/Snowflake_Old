/*Created by Gareth Ellis - October 21, 2015*/

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

#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <utility>
#include <algorithm>
#include <math.h>


float CUTOFF = 100; // Distance away from the robot to search for obstacles
float RIGHT_ANGLE = 90; 
float LEFT_ANGLE = -90;
float BACK_RIGHT = 135; 
float BACK_LEFT = -135;


/*~~~~~~~~~~~~~~~~~~~~~~~~~~ CLASSES ~~~~~~~~~~~~~~~~~~~~~~~~~~*/

//Command is a command to send the the robot where:
//  - Type is one of: "turn" "move" or "turn-move"
//  - Value is the value of a given command, ie) 1 meter (1), 30 degrees (30), 1 meter and 30 degrees (1,30)
class command {
	std::string type;
	int move_value = 0;
	int turn_value = 0;

	public:
		void makeMoveCommand(double value) {
			type = "move";
			move_value = value;
		}
		void makeTurnCommand(double value) {
			type = "turn";
			turn_value = value;
		}
		void makeMoveTurnCommand(double move_val, double turn_val) {
			type = "turn-move";
			move_value = move_val;
			turn_value = turn_val;
		}
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
			type = type_data;
			sector = sector_data;
			x_pos = x_pos_data;
			y_pos = y_pos_data;
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
	float width_data = 10;
	float length_data = 10;
	float rot_angle_data = 0;
	std::pair<int, int> front_right_data = std::make_pair(int(width_data / 2), int(length_data / 2));
	std::pair<int, int> front_left_data = std::make_pair(int(width_data / -2), int(length_data / 2));
	std::pair<int, int> back_right_data = std::make_pair(int(width_data / 2), int(length_data / -2));
	std::pair<int, int> back_left_data = std::make_pair(int(width_data / -2), int(length_data / -2));

public:
	robot(float width, float length, float rot_angle) {
		width = width;
		length = length;
		rot_angle = rot_angle;
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
int mapValAt(std::vector<int> &map, int map_width, int x_coor, int y_coor) {
	return (map[x_coor * map_width + y_coor]);
}

//Converts a given angle in degrees to a slope value
float degreesToSlope(float angle_in_degrees) {
	return tan((angle_in_degrees * 3.14159 )/ 180);
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

bool notAtTarget(robot snowflake, std::pair<int, int> current_coor, std::pair<int, int> destination_coor) {
   
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
    int robot_max_x = current_coor.first + *std::max_element(robot_x_coors,robot_x_coors+4);     // The max x value out of all corners of the robot
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

/*
ONLY ROUGH ANGLES - ACTUAL ANGLES DEFINED ELSEWHERE
DIAGRAM NOT PRESENTLY VALID, USING ANGLE RELATIVE TO CENTER OF ROBOT AND ROBOTS PRESENT ROT_ANGLE INSTEAD
+-----------------------------------------+
|       *   front-   *  front-    *       |
|        *   left    *   right   *        |
|         *          *          *         |
|          *         *         *          |
|   left    *      front      *   right   |
|            +---------------+            |
|*           |               |           *|
| **         |               |         ** |
|   **       |               |       **   |
|     **     |               |     **     |
|       **   |               |   **       |
|         ** |               | **         |
|           *+---------------+*           |
|                                         |
|                  behind                 |
+-----------------------------------------+
*/


//Gets which sector a given obstacle is in from it's relative angle to the robot
obstacle addSector(obstacle obstacle_with_unkown_sector, robot snowflake, std::pair<int, int> current_coor) {
	std::pair<int, int> obstacle_coors = std::make_pair(obstacle_with_unkown_sector.x(), obstacle_with_unkown_sector.y());

	//Get relative angle of obstacle to robot
	float obstacle_angle = slopeToDegrees((obstacle_with_unkown_sector.y() - current_coor.second) / (obstacle_with_unkown_sector.x() - current_coor.first));
	float relative_obstacle_angle = obstacle_angle - snowflake.rot_angle();

	//Check against angles to determine sector
	if ((relative_obstacle_angle > 0) && (relative_obstacle_angle < RIGHT_ANGLE)) {  // Obstacle is in front right
		return obstacle_with_unkown_sector.make_sector("front-right");
	}
	else if ((relative_obstacle_angle > LEFT_ANGLE) && (relative_obstacle_angle < 0)) {  // Obstacle is in front left
		return obstacle_with_unkown_sector.make_sector("front-left");
	}
	else if ((relative_obstacle_angle > RIGHT_ANGLE) && (relative_obstacle_angle < BACK_RIGHT)) {  // Obstacle is in right
		return obstacle_with_unkown_sector.make_sector("right");
	}
	else if ((relative_obstacle_angle < LEFT_ANGLE) && (relative_obstacle_angle > BACK_LEFT)) {
		return obstacle_with_unkown_sector.make_sector("left");
	}
	else {
		return obstacle_with_unkown_sector.make_sector("back");
	}

}

// Checks a given row for obstacles from a given y-min value to a given y-max value on a given map
// If found, returns the obstacle as: obstacle((pole or wall), "unkown", x_coor, y_coor) 
//                       else returns obstacle("none", "none", -1, -1)
obstacle checkRow(std::vector<int> &map, int map_width, int row_num, int x_min, int x_max) {
	for (int i = 0; i < (x_max - x_min); i++) {
		int x_coor = x_min + i;
		int y_coor = row_num;

		int present_node_val = mapValAt(map, map_width, x_coor, y_coor);
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

// Checks a given column for obstacles from a given x-min value to a given x-max value on a given map
// If found, returns the obstacle as: obstacle((pole or wall), "unkown", x_coor, y_coor) 
//                       else returns obstacle("none", "none", -1, -1)
obstacle checkColumn(std::vector<int> &map, int map_width, int column_num, int y_min, int y_max) {
	for (int i = 0; i < (y_max - y_min); i++) {
		int x_coor = column_num;
		int y_coor = y_min + i;

		int present_node_val = mapValAt(map, map_width, x_coor, y_coor);
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

obstacle getClosestObstacle(std::pair<int, int> current_coor, std::vector<int> &map, int map_width, robot snowflake) {

	//Moves out from the center position of the robot, checking each row and column as it goes
    for (int i = 1; i < CUTOFF; i++) {
        int y_max = current_coor.second + i; // The max y value to checked on each column AND the top row to be checked
        int y_min = current_coor.second + i; // The min y value to checked on each column AND the bottom row to be checked
        int x_max = current_coor.first + i;  // The max x value to be checked on each row AND the left column to be checked
        int x_min = current_coor.first - i;  // The min x value to be checked on each row AND the right column to be checked
		
		//Look for any obstacles in the top and bottom rows, left and right columns, at i nodes away from the robot
		obstacle possible_obstacles[4] = {
			checkRow(map, map_width, y_max, x_min, x_max),
			checkRow(map, map_width, y_min, x_min, x_max),
			checkColumn(map, map_width, x_min, y_min, y_max),
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


// Checks whether or not the robot is directly facing the destination
// !!!
bool directlyFacingDest(std::pair<int,int> current_coor, std::pair<int,int> destination_coor, robot snowflake) {
	float angle_robot_destination = slopeToDegrees((current_coor.second - destination_coor.second) / (current_coor.first - destination_coor.first));
	float relative_angle_robot_destination = angle_robot_destination - snowflake.rot_angle();
	//If robot is pointed less then 5 degrees off of the destination
	return ((abs(relative_angle_robot_destination) < 5));
}



/*~~~~~~~~~~ Main Decision Tree ~~~~~~~~~~*/
// gets the next command to send to the robot
command getNextCommand(std::vector<int> &map, int map_width, robot snowflake, std::pair<int, int> current_coor, std::pair<int, int> destination_coor) {
	if (notAtTarget(snowflake, current_coor, destination_coor)) {
		obstacle closest_obstacle = getClosestObstacle(current_coor, map, map_width, snowflake);
		if ((closest_obstacle.sector() == "back")) {  // Not facing obstacle
			if (directlyFacingDest(current_coor, destination_coor, snowflake)) {

			}
			else { //Not directly facing the destination

			}
		}
		else if (closest_obstacle.type() == "pole") {
			if (closest_obstacle.sector() == "left") {

			}
			else if (closest_obstacle.sector() == "right"){

			}
			else { // obstacle_sector == front

			}
		}
		else {  // Facing wall
			if ((closest_obstacle.sector() == "left") || (closest_obstacle.sector() == "front-left")) {

			}
			else if ((closest_obstacle.sector() == "right") || (closest_obstacle.sector() == "front-right")) {

			}
			else { // obstacle_sector == front

			}
		}
	}
	command next_command;
	next_command.makeMoveCommand(0);
	return next_command;
}

/*~~~~~~~~~~ Funtions To Publish Information ~~~~~~~~~~*/
// Publishs the command to the correct ROS topic in the correct format !!! WILL NEED TO BE CHANGED FOR ROS INTEGRATION
void publishCommand(command cmd) {

}


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