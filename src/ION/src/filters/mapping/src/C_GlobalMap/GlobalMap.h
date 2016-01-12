// Header file for global map class
#pragma once
#include "ros/ros.h"
#include <nav_msgs/OccupancyGrid.h>

class GlobalMap {
  
  private:
    int mapSize;  
  
  public: 
    // Constructor
    GlobalMap(int x, int y);
    
    // Get set functions
    int GetMapSize();
};
