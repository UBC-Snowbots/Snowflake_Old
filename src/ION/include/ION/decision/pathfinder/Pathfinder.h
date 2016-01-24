#pragma once
#include <vector>
#include <algorithm>
#include <cmath>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose2D.h>


namespace ION{
	namespace decision{
		namespace pathfinder{

			#define AI_INFINITY DOUBLE_MAX
			#define STANDARD_COST 10.0
			#define SQRT2 1.4142135623

			struct Tile{
				const unsigned int x, y;

				/*	A tile in A* keeps track of 4 important information

					G - Total cost accumulated so far since we've moved from the starting point
					
					H - An estimation -doesn't have to be 100% accurate- of how much it 
						would cost to move from here to the goal point.

					As we approach the goal tile, G will increase to the actual cost to traverse from start to goal,
					while H will decrease since the distance to the goal is shortening.

					Cost - The cost of moving to this tile, a low cost is more favourable than a high cost.
							If the cost is PF_INFINITY then it is considered a wall/obsticle.

							We could use a higher cost tiles to make certain areas of the map less preferable,
							such as tiles located very close to the obsticals, so the resultant path doesn't 
							get too close to these tiles unless they have to.				

					Parent - A way to keep track of the path back to the starting point.
							(Think of Linked-List keeping pointers to find the next Node)
				*/
				double g, h, cost;
				Tile* parent;
				
				/*
					The following two booleans are for making this specific A* implementation simple,
					other implementations of A* will vary.

					isClose - A boolean flag to indicate whether this tile is currently inside the
							CLOSED list, once a tile is inside it should never be removed from this list.
							This prevents A* from analysising the same tile over and over again.

					isOpen - A boolean flag to indicate whether this tile is currently inside the
							OPEN list. Tiles can freely be inserted and removed from this list.
				*/
				bool isClose;
				bool isOpen;


				Tile();
				Tile(unsigned int x, unsigned int y, double cost);
				Tile(const Tile &copyTile);

				//For convient, but naive, debugging purposes
				void info() const;


				//To be used with the make_heap(), push_heap(), and pop_heap() functions
				const struct Compare{

					//Returns the tile with the lowest g and h value
					bool operator()(const Tile* left, const Tile* right) const {

						//Because the *_heap() functions operate as a MAX heap, we can turn it into a MIN heap
						//by reversing the comparision, such that it sorts in the desecending order where
						//the left element is larger than the right element.
						return (left->g + left->h) > (right->g + right->h);
					}
				};

			};

			class Grid{
			private:
				/*
					Gridmap - Contains the Tile as a 2D grid via a 1D array: given a pair of (x, y)
							coordinates the corresponding index is f(x, y) = y * width + x
				*/
				std::vector<Tile> gridmap;

				Tile* start;
				Tile* goal;
				double length, width;
				

				//Estimate -doesn't have to be 100% accurate- the total cost to move the goal tile, 
				//starting at the current input tile.
				void calculateHeuristics(Tile*& tile);

				//Calculates the cost to move between two adjacent tiles.
				double calculateCost(Tile*& tileA, Tile*& tileB) const;

				//Determines if a certain tile is inside the map.
				bool withinMap(unsigned int x, unsigned int y) const;

				//Gives a pointer to the tile at (x, y), provided that it exist in the map.
				Tile* getTileAt(unsigned int x, unsigned int y);

			public:
				Grid(nav_msgs::OccupancyGrid::ConstPtr& map, geometry_msgs::Pose2D::ConstPtr& start);
				
				std::vector<Tile> computeShortestPath(geometry_msgs::Pose2D::ConstPtr& goal);
			};
		}
	}
}