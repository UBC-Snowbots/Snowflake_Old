#pragma once

#include <utility>
#include <vector>
#include <string>
#include "map_interface.hpp"

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

class vector_map: public map_interface{
	std::vector<int> data;
	int width_data;
	int height_data;
	public:
	vector_map(std::vector<int>& data, int width, int height):
			data(std::move(data)),
			width_data(width),
			height_data(height)
		{}
	int width() const override{
		return width_data;
	}
	int height() const override{
		return height_data;
	}
	int at(int x, int y) const override{
		return data.at(x + y*width_data);
	}
	void set(int x, int y, int value) override {
		data[x + y*width_data] = value;
	}
	bool withinBounds(int x, int y) override {
		return (x < width_data && x >= 0) && (y < height_data && y >= 0);
	}
};

/*~~~~~~~~~ FUNCTIONS ~~~~~~~~~*/

float degreesToSlope(float angle_in_degrees);
vector_map getMap(std::string map_file_name, int width);
float slopeToDegrees(float slope);

float get_relative_angle_robot_destination(std::pair<int, int> current_coor, std::pair<int, int> destination_coor, robot snowflake);
obstacle addSector(obstacle obstacle_with_unkown_sector, robot snowflake, std::pair<int, int> current_coor);
obstacle checkColumn(const map_interface &map, int col_num, int y_min, int y_max);
obstacle checkRow(const map_interface &map, int row_num, int x_min, int x_max);
obstacle getClosestObstacle(std::pair<int, int> current_coor, const map_interface &map, robot snowflake);
bool atDestination(std::pair<int, int> current_coor, std::pair<int, int> destination_coor);
command getNextCommand(const map_interface &map, robot snowflake, std::pair<int, int> current_coor, std::pair<int, int> destination_coor);


#endif

