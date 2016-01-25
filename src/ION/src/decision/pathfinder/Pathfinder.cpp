#include <ION/decision/pathfinder/Pathfinder.h>
#include <ros/ros.h>

namespace ION{
	namespace decision{
		namespace pathfinder{

			Grid::Grid(
				nav_msgs::OccupancyGrid map, 
				geometry_msgs::Pose2D start_pos, 
				geometry_msgs::Pose2D goal_pos)
			{
				this->length = map.info.height;
				this->width = map.info.width;
				gridmap.reserve(length * width);
	
				for (unsigned int y = 0; y < length; y++){
					for (unsigned int x = 0; x < width; x++){
						gridmap.push_back(Tile(x, y, (double) map.data[y*width + x]));
					}
				}
	
				//Need to know resolution of map to translate pose2d to cell values
				start = getTileAt(0,0);
				goal = getTileAt(1,1);
			}


			void Grid::computeShortestPath(bool reduce=false){
				//Set-up the algorithm by creating OPEN list and placing our starting point inside
				std::vector<Tile*> open;
				std::vector<Tile*> path;

				goal->isOpen = true;
				open.push_back(goal);

				//This is the main part of the algorithm
				while (open.size() != 0){

					//CURRENT is a Tile* inside OPEN with the lowest G + H scores
					pop_heap(open.begin(), open.end(), Tile::Compare());
					Tile* current = open.back();
					open.pop_back();

					//Close the CURRENT tile to prevent it from being re-analysed again by another tile
					current->isClose = true;
		
					//ROS_INFO("Current tile: \n");
					//current->info();

					if (current == start){
						ROS_INFO("A path has been found!\n");

						while (start != 0){
							path.push_back(start);
							start = start->parent;
						}
						break;
					}

					for (int dy = -1; dy <= 1; dy++){
						for (int dx = -1; dx <= 1; dx++){
							if (abs(dy) + abs(dx) != 0){

								//Get the pointer to a neighbouring tile
								Tile* neighbour = getTileAt(current->x + dx, current->y + dy);
					
								//Determine if this neighbouring tile should be analysed
								if (neighbour == 0 || neighbour->isClose || neighbour->cost == AI_INFINITY){
									continue;
								}

								if (abs(dy) + abs(dx) == 2){
									/*
										This tile is located diagonally from CURRENT
							
										However, not all diagonal tiles can be reached without hitting a sharp corner,
										we need to test if this tile is reachable
									*/
									Tile* a = getTileAt(current->x, current->y + dy);
									Tile* b = getTileAt(current->x + dx, current->x);

									if (a != 0 && a->cost == AI_INFINITY || b != 0 && b->cost == AI_INFINITY){
										continue;
									}
								}

								//Calculate the new g value
								double newGValue = current->g + calculateCost(current, neighbour);

								//Determine if this tile is currently "inside" the OPEN list
								bool isCurrentlyOpen = neighbour->isOpen;

								if (!isCurrentlyOpen || newGValue < neighbour->g){
									//(NEIGHBOUR is not in the OPEN list) OR (it's already in OPEN but we found a shorter path to it)

									//Open or reopen this neighbouring tile
									neighbour->isOpen = true;

									//Update the NEIGHBOUR'S cost and parent pointer
									neighbour->parent = current;
									neighbour->g = newGValue;
									calculateHeuristics(neighbour);

									//Use the previously stored value to determine what to do with the updated neighbour
									if (!isCurrentlyOpen){
										//Since NEIGHBOUR was not inside OPEN beforehand, we add it to OPEN
										open.push_back(neighbour);
										push_heap(open.begin(), open.end(), Tile::Compare());
									}
									else {
										//NEIGHBOUR is already inside OPEN, we need to update it's priority in the queue
										make_heap(open.begin(), open.end(), Tile::Compare());

									}

								}
							}

						}

					}
				}


				ROS_FATAL("Unable to compute a path");
			}

			bool Grid::withinMap(unsigned int x, unsigned int y) const{
				//Determines if the coordinate is within the bounds of the grid map
				return y >= 0 && y < length && x >= 0 && x < width;
			}

			void Grid::calculateHeuristics(Tile*& tile){
				//This is a calculation used to estimate the distance a tile and the goal tile

				unsigned int dx = labs(tile->x - start->x);
				unsigned int dy = labs(tile->y - start->y);

				if (dx > dy){
					std::swap(dx, dy);
				}

				tile->h = ((SQRT2 - 1) * dx + dy) * STANDARD_COST;
			}

			double Grid::calculateCost(Tile*& tileA, Tile*& tileB) const{
				if (tileA->cost == AI_INFINITY || tileB->cost == AI_INFINITY){
					//Should prevent an obvious binary overflow
					return AI_INFINITY;
				}

				if (labs(tileA->x - tileB->x) + labs(tileA->y - tileB->y) == 2){
					//These two tiles are diagonally adjacent to each other

					//Note: We assume that the map isn't big enough to make cost approach AI_INFINITY/2,
					//becareful of potential binary overflow here.
					return SQRT2 * (tileA->cost + tileB->cost) / 2;
				}

				return (tileA->cost + tileB->cost) / 2;
			}


			Tile* Grid::getTileAt(unsigned int x, unsigned int y){
				return withinMap(x, y) ? &(gridmap[y * width + x]) : NULL;
			}


			Tile::Tile(unsigned int x, unsigned int y, double cost) : 
				x(x), 
				y(y)
			{

				this->cost = cost;
				this->g = 0;
				this->h = 0;

				this->isClose = false;
				this->isOpen = false;

				this->parent = 0;
			}

			Tile::Tile(const Tile &copy) : 
				x(copy.x),
				y(copy.y)
			{
				this->cost = copy.cost;
				this->h = copy.h;
				this->g = copy.g;

				this->isClose = copy.isClose;
				this->isOpen = copy.isOpen;
	
				//Shallow copy
				this->parent = copy.parent;
			}


			void Tile::info() const{
				ROS_INFO("[(%u, %u) C: %lf    H: %lf   G: %lf]\n", 
					this->x, this->y, this->cost, this->h, this->g
				);
			}



		}
	}
}
