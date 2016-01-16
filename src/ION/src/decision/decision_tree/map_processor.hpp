/*Created by Aaron Mishkin - January 14, 2016 */

/*
	This is a preprocessor for a one dimensional vector map that implements the map_interface class defined in decision_tree.hpp. 
	It will identify and change the value of walls in the map in order to distinguish them from obstacles.
*/

#ifndef __MAP_PROCESSOR__
#define __MAP_PROCESSOR__

class map_interface;

// Constants for map processing
static const int WALL = 2;
static const int OBSTACLE = 1;

static const int WALL_CUTOFF = 4;

int checkCardinalNeighbours(int x, int y, map_interface* map); 

int checkAllNeighbours(int x, int y, map_interface* map); 

map_interface* processMap(map_interface* map, int (*checkNeighbours)(int, int, map_interface*));

#endif