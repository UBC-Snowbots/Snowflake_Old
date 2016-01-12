#include "Localization.h"
#include <iostream>

Localization::Localization(GlobalMap * map)
{
  globalMap = map;
}

void Localization::LocalMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map)
{
  return;
}


