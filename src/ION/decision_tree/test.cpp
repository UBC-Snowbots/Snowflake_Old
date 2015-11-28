/*~~~~~~~~~~ Unit Testing ~~~~~~~~~~*/

#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main() - only do this in one cpp file
#include "decision_tree.hpp"
#include "catch.hpp"


//EXAMPLE
/*
unsigned int Factorial(unsigned int number) {
return number <= 1 ? number : Factorial(number - 1)*number;
}

TEST_CASE("Factorials are computed", "[factorial]") {
CHECK(Factorial(0) == 1);
CHECK(Factorial(2) == 2);
CHECK(Factorial(3) == 6);
CHECK(Factorial(10) == 3628800);
}
*/



TEST_CASE("degreesToSlope", "[degreesToSlope]") {
	CHECK(degreesToSlope(0) == Approx(0));
	CHECK(degreesToSlope(45) == Approx(1.0));
	CHECK(degreesToSlope(180) == Approx(0));
	CHECK(degreesToSlope(234) == Approx(1.3762).epsilon(10));
	CHECK(degreesToSlope(360) == Approx(0));
}

TEST_CASE("Relative angle of robot to destination", "[get_relative_angle_robot_destination]") {
	CHECK(get_relative_angle_robot_destination(std::make_pair(0, 0), std::make_pair(1, 1), robot(1, 1, 0)) == Approx(45));
	CHECK(get_relative_angle_robot_destination(std::make_pair(1, 1), std::make_pair(0, 0), robot(1, 1, 0)) == Approx(-135));
	CHECK(get_relative_angle_robot_destination(std::make_pair(0, 0), std::make_pair(0, 0), robot(1, 1, 0)) == Approx(0));
	CHECK(get_relative_angle_robot_destination(std::make_pair(0, 0), std::make_pair(1, 1), robot(1, 1, 45)) == Approx(0));
	CHECK(get_relative_angle_robot_destination(std::make_pair(0, 0), std::make_pair(1, 1), robot(1, 1, 90)) == Approx(-45));
	CHECK(get_relative_angle_robot_destination(std::make_pair(1, 1), std::make_pair(2, 2), robot(1, 1, 0)) == Approx(45));
	CHECK(get_relative_angle_robot_destination(std::make_pair(0, 1), std::make_pair(1, 2), robot(1, 1, 0)) == Approx(45));
	CHECK(get_relative_angle_robot_destination(std::make_pair(10, 10), std::make_pair(5, 5), robot(1, 1, 0)) == Approx(-135)); // To top right of destination
	CHECK(get_relative_angle_robot_destination(std::make_pair(0, 10), std::make_pair(5, 5), robot(1, 1, 0)) == Approx(135)); // To top left of destination
	CHECK(get_relative_angle_robot_destination(std::make_pair(10, 0), std::make_pair(5, 5), robot(1, 1, 0)) == Approx(-45)); // To bottom right of destination
	CHECK(get_relative_angle_robot_destination(std::make_pair(0, 0), std::make_pair(5, 5), robot(1, 1, 0)) == Approx(45)); // To bottom left of destination
}

TEST_CASE("Map Value at Position", "[mapValAt]") {
	std::vector<int> testmap1 = getMap("map1_width_6.map");
	CHECK(mapValAt(testmap1, 6, 0, 0) == 1);
	CHECK(mapValAt(testmap1, 6, 1, 1) == 0);
	CHECK(mapValAt(testmap1, 6, 2, 1) == 0);
	CHECK(mapValAt(testmap1, 6, 4, 2) == 1);
	CHECK(mapValAt(testmap1, 6, 5, 3) == 1);
}

TEST_CASE("slopeToDegrees", "[slopeToDegrees]") {
	CHECK(slopeToDegrees(1) == Approx(45));
	CHECK(slopeToDegrees(.5) == Approx(26.57).epsilon(10));
	CHECK(slopeToDegrees(.25) == Approx(14.036).epsilon(10));
	CHECK(slopeToDegrees(1) == Approx(45));

}


TEST_CASE("addSector", "[addSector]") {
	CHECK(addSector(obstacle("pole", "unkown", 2, 2), robot(1, 1, 0), std::make_pair(1, 1)).sector() == "front-right");  // Obstacle in front-right
	CHECK(addSector(obstacle("pole", "unkown", 5, 5), robot(1, 1, 0), std::make_pair(8, 1)).sector() == "front-left");  // Obstacle in front-left
	CHECK(addSector(obstacle("pole", "unkown", 5, 5), robot(1, 1, 0), std::make_pair(7, 5)).sector() == "left");  // Obstacle in left
	CHECK(addSector(obstacle("pole", "unkown", 5, 5), robot(1, 1, 0), std::make_pair(3, 5)).sector() == "right");  // Obstacle in right
	CHECK(addSector(obstacle("pole", "unkown", 5, 5), robot(1, 1, 0), std::make_pair(5, 8)).sector() == "back");  // Obstacle in back
	CHECK(addSector(obstacle("pole", "unkown", 5, 5), robot(1, 1, 180), std::make_pair(5, 2)).sector() == "back");  // Obstacle in back, robot turned
	CHECK(addSector(obstacle("pole", "unkown", 5, 5), robot(1, 1, 180), std::make_pair(2, 5)).sector() == "left");  // Obstacle in left, robot turned
}


TEST_CASE("checkColumn", "[checkColumn]") {
	std::vector<int> testmap1 = getMap("map2_width_10.map");
	CHECK(checkColumn(testmap1, 10, 0, 2, 5).sector() == "unkown");
	CHECK(checkColumn(testmap1, 10, 0, 2, 5).type() == "wall");
	CHECK(checkColumn(testmap1, 10, 0, 2, 5).x() == 0);
	CHECK(checkColumn(testmap1, 10, 0, 2, 5).y() == 2);

	CHECK(checkColumn(testmap1, 10, 1, 1, 5).sector() == "unkown");
	CHECK(checkColumn(testmap1, 10, 1, 1, 5).type() == "pole");
	CHECK(checkColumn(testmap1, 10, 1, 1, 5).x() == 1);
	CHECK(checkColumn(testmap1, 10, 1, 1, 5).y() == 2);

	//Checking a column outside the map
	CHECK(checkColumn(testmap1, 10, 50, 1, 5).sector() == "none");
	CHECK(checkColumn(testmap1, 10, 50, 1, 5).type() == "none");
	CHECK(checkColumn(testmap1, 10, 50, 1, 5).x() == -1);
	CHECK(checkColumn(testmap1, 10, 50, 1, 5).y() == -1);

	//Checking a column just barely outside the map
	CHECK(checkColumn(testmap1, 10, 10, 1, 5).sector() == "none");
	CHECK(checkColumn(testmap1, 10, 10, 1, 5).type() == "none");
	CHECK(checkColumn(testmap1, 10, 10, 1, 5).x() == -1);
	CHECK(checkColumn(testmap1, 10, 10, 1, 5).y() == -1);
}

TEST_CASE("checkRow", "[checkRow]") {
	std::vector<int> testmap1 = getMap("map2_width_10.map");
	CHECK(checkRow(testmap1, 10, 0, 1, 5).sector() == "unkown");
	CHECK(checkRow(testmap1, 10, 0, 1, 5).type() == "wall");
	CHECK(checkRow(testmap1, 10, 0, 1, 5).x() == 1);
	CHECK(checkRow(testmap1, 10, 0, 1, 5).y() == 0);

	CHECK(checkRow(testmap1, 10, 3, 1, 5).sector() == "unkown");
	CHECK(checkRow(testmap1, 10, 3, 1, 5).type() == "pole");
	CHECK(checkRow(testmap1, 10, 3, 1, 5).x() == 4);
	CHECK(checkRow(testmap1, 10, 3, 1, 5).y() == 3);

	CHECK(checkRow(testmap1, 10, 4, 1, 5).sector() == "none");
	CHECK(checkRow(testmap1, 10, 4, 1, 5).type() == "none");
	CHECK(checkRow(testmap1, 10, 4, 1, 5).x() == -1);
	CHECK(checkRow(testmap1, 10, 4, 1, 5).y() == -1);

	//Try row just outside the map
	CHECK(checkRow(testmap1, 10, 7, 1, 5).sector() == "none");
	CHECK(checkRow(testmap1, 10, 7, 1, 5).type() == "none");
	CHECK(checkRow(testmap1, 10, 7, 1, 5).x() == -1);
	CHECK(checkRow(testmap1, 10, 7, 1, 5).y() == -1);

}

TEST_CASE("getClosestObstacle", "[getClosestObstacle]") {
	std::vector<int> testmap1 = getMap("map2_width_10.map");
	robot snowflake = robot(1, 1, 0);

	// Pole directly beside the right side of robot
	CHECK(getClosestObstacle(std::make_pair(3, 3), testmap1, 10, snowflake).sector() == "right");
	CHECK(getClosestObstacle(std::make_pair(3, 3), testmap1, 10, snowflake).type() == "pole");
	CHECK(getClosestObstacle(std::make_pair(3, 3), testmap1, 10, snowflake).x() == 4);
	CHECK(getClosestObstacle(std::make_pair(3, 3), testmap1, 10, snowflake).y() == 3);

	// Pole directly to left of robot, two nodes away
	CHECK(getClosestObstacle(std::make_pair(6, 3), testmap1, 10, snowflake).sector() == "left");
	CHECK(getClosestObstacle(std::make_pair(6, 3), testmap1, 10, snowflake).type() == "pole");
	CHECK(getClosestObstacle(std::make_pair(6, 3), testmap1, 10, snowflake).x() == 4);
	CHECK(getClosestObstacle(std::make_pair(6, 3), testmap1, 10, snowflake).y() == 3);

	// Pole in the front-left sector
	CHECK(getClosestObstacle(std::make_pair(5, 2), testmap1, 10, snowflake).sector() == "front-left");
	CHECK(getClosestObstacle(std::make_pair(5, 2), testmap1, 10, snowflake).type() == "pole");
	CHECK(getClosestObstacle(std::make_pair(5, 2), testmap1, 10, snowflake).x() == 4);
	CHECK(getClosestObstacle(std::make_pair(5, 2), testmap1, 10, snowflake).y() == 3);

	// Wall directly to right (NOTE: Due to the way that checkColumn works (it starts at the top of a column and goes down))
	// the first "wall" section found is in the front-right section
	CHECK(getClosestObstacle(std::make_pair(8, 2), testmap1, 10, snowflake).sector() == "front-right");
	CHECK(getClosestObstacle(std::make_pair(8, 2), testmap1, 10, snowflake).type() == "wall");
	CHECK(getClosestObstacle(std::make_pair(8, 2), testmap1, 10, snowflake).x() == 9);
	CHECK(getClosestObstacle(std::make_pair(8, 2), testmap1, 10, snowflake).y() == 3);
}

TEST_CASE("atDestination", "[atDestination]") {
	CHECK(atDestination(std::make_pair(1, 1), std::make_pair(5, 5)) == false);
	CHECK(atDestination(std::make_pair(2, 3), std::make_pair(3, 3)) == true);
	CHECK(atDestination(std::make_pair(10, 10), std::make_pair(5, 5)) == false);
}

TEST_CASE("getNextCommand", "[getNextCommand]") {
	std::vector<int> testmap3 = getMap("map3_width_10.map");
}


