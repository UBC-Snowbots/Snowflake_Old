#pragma once

#include <utility>
#include <vector>
#include <string>

#ifndef DECISION_TREE
#define DECISION_TREE


/*~~~~~~~~~ CLASSES ~~~~~~~~~*/

//Command is a command to send the the robot where:
//  - Value is the value of a given command, ie) 1 meter (1), 30 degrees (30), 1 meter and 30 degrees (1,30)
class command {
	int move_value_data = 0;
	int turn_value_data = 0;

public:
	command(float move_value, float turn_value);
	float move_val();
	float turn_val();
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
	obstacle(std::string type, std::string sector, int x_pos, int y_pos);
	obstacle make_sector(std::string sector);
	std::string type();
	std::string sector();
	int x();
	int y();
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
	robot(float width, float length, float rot_angle);
	void set_rot_angle(float rot_angle);
	float width();
	float length();
	float rot_angle();
	std::pair<int, int> front_right();
	std::pair<int, int> front_left();
	std::pair<int, int> back_right();
	std::pair<int, int> back_left();
};

/*~~~~~~~~~ FUNCTIONS ~~~~~~~~~*/
float degreesToSlope(float angle_in_degrees);
std::vector<int> getMap(std::string map_file_name);
int mapValAt(std::vector<int> &map, int map_width, int x_coor, int y_coor);
float slopeToDegrees(float slope);

float get_relative_angle_robot_destination(std::pair<int, int> current_coor, std::pair<int, int> destination_coor, robot snowflake);
obstacle addSector(obstacle obstacle_with_unkown_sector, robot snowflake, std::pair<int, int> current_coor);
obstacle checkColumn(std::vector<int> &map, int map_width, int column_num, int y_min, int y_max);
obstacle checkRow(std::vector<int> &map, int map_width, int row_num, int x_min, int x_max);
obstacle getClosestObstacle(std::pair<int, int> current_coor, std::vector<int> &map, int map_width, robot snowflake);
bool atDestination(std::pair<int, int> current_coor, std::pair<int, int> destination_coor);
command getNextCommand(std::vector<int> &map, int map_width, robot snowflake, std::pair<int, int> current_coor, std::pair<int, int> destination_coor);


#endif

