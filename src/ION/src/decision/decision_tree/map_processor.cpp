/*Created by Aaron Mishkin - January 14, 2016 */

/*
	This is a preprocessor for a one dimensional vector map that implements the map_interface class defined in decision_tree.hpp. 
	It will identify and change the value of walls in the map in order to distinguish them from obstacles.
*/

#include "map_processor.hpp"
#include "map_interface.hpp"

// Checks in the cardinal directions around a node to see if it is a wall
int checkCardinalNeighbours(int x, int y, map_interface* map) {
	int adjacentObstacles = 0;

	if (map->withinBounds(x + 1, y) && (map->at(x + 1, y) == OBSTACLE || map->at(x + 1, y) == WALL)) {
		adjacentObstacles++;
		if (map->withinBounds(x + 2, y) && (map->at(x + 2, y) == OBSTACLE || map->at(x + 2, y) == WALL)) 
			adjacentObstacles++;
	}
	if (map->withinBounds(x, y - 1) && (map->at(x, y - 1) == OBSTACLE || map->at(x, y - 1) == WALL)) {
		adjacentObstacles++; 
		if (map->withinBounds(x, y - 2) && (map->at(x, y - 2) == OBSTACLE || map->at(x, y - 2) == WALL)) 
			adjacentObstacles++; 
	}
	if (map->withinBounds(x - 1, y) && (map->at(x - 1, y) == OBSTACLE || map->at(x - 1, y) == WALL)) {
		adjacentObstacles++; 
		if (map->withinBounds(x - 2, y) && (map->at(x - 2, y) == OBSTACLE || map->at(x - 2, y) == WALL)) 
			adjacentObstacles++; 
	}
	if (map->withinBounds(x, y + 1) && (map->at(x, y + 1) == OBSTACLE || map->at(x, y + 1) == WALL)) {
		adjacentObstacles++; 
		if (map->withinBounds(x, y + 2) && (map->at(x, y + 2) == OBSTACLE || map->at(x, y + 2) == WALL))
			adjacentObstacles++; 
	}

	return adjacentObstacles;
}


// This function checks a box 5x5 around a node to see if it is a wall
int checkAllNeighbours(int x, int y, map_interface* map) {
	int adjacentObstacles = -1; // so that we don't count the node itself
	for (int width = x - 2; width <= x + 2; width++) {
		for (int height = y - 2; height <= y + 2; height++) {
			if (map->withinBounds(width, height) && (map->at(width, height) == OBSTACLE || map->at(width, height) == WALL))
				adjacentObstacles++;
		}
	}
	return adjacentObstacles;
}

// Map Pre-Processor
map_interface* processMap(map_interface* map, int (*checkNeighbours)(int, int, map_interface*)) {
	for (int x = 0; x < map->width(); x++) {
		for (int y = 0; y < map->height(); y++) {
			if (map->at(x, y) == OBSTACLE) {
				int adjacentObstacles = checkNeighbours(x, y, map);

				// Map square borders enough obstacles to be considered a wall.
				if (adjacentObstacles >= WALL_CUTOFF)
					map->set(x, y, WALL);
			}
		}
	}
	return map;
}
