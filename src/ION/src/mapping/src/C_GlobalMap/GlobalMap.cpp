#include "GlobalMap.h"
#include <iostream>

GlobalMap::GlobalMap(int x, int y)
{
  mapSize = x*y;
}

int GlobalMap::GetMapSize()
{
  return mapSize;
}

