#pragma once

#include <vector>
#include <algorithm>
#include <cmath>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose2D.h>

#define AI_INFINITY INT_MAX
#define STANDARD_COST 10.0
#define SQRT2 1.4142135623

struct Tile{
	const unsigned int x, y;
	double g, h, cost;

	Tile* parent;
	bool isClose;
	bool isOpen;

	Tile();
	Tile(unsigned int x, unsigned int y, double cost);
	Tile(const Tile &copyTile);

	void info() const;

	struct Compare{
		bool operator()(const Tile* left, const Tile* right) const {
			return (left->g + left->h) > (right->g + right->h);
		}
	};

};

class Grid{
	private:
		std::vector<Tile> gridmap;

		Tile* start;
		Tile* goal;
		double length, width;
	
		void calculateHeuristics(Tile*& tile);
		double calculateCost(Tile*& tileA, Tile*& tileB) const;
		bool withinMap(unsigned int x, unsigned int y) const;
		Tile* getTileAt(unsigned int x, unsigned int y);

	public:
		Grid(nav_msgs::OccupancyGrid map, 
		geometry_msgs::Pose2D start, 
		geometry_msgs::Pose2D goal);
	
		void computeShortestPath(bool reduce);
};


