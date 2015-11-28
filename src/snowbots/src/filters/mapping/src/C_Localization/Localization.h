// Header file for localization class
#pragma once
#include "ros/ros.h"
#include <nav_msgs/OccupancyGrid.h>
#include "../C_GlobalMap/GlobalMap.h"

class Localization {
  
  private:
    GlobalMap * globalMap;
  
  public: 
    Localization(GlobalMap * map);
    void LocalMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map);
};