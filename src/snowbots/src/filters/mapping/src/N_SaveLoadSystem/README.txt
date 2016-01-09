Snowbots 2015-2016

Author: Jagjot Jhajj

Description: 
This is the system that saves the most recently generated global map and loads the map at the start of the run if there are any existing maps.

Server Operations:
- save(global_map)
- load()



Current Status: I have figured out how to save a map into yaml format. It has worked successfully 
with a 2x2 test grid. When trying to load, the command executes but the map doesn't seem to be actually
loading in my test program. It's likely that my test subscriber just isn't done right so I need to figure
out how to make it work.

---------------------------------------------------------------------------------------------
Saving maps:
---------------------------------------------------------------------------------------------
rosrun map_server map_saver -f mymap

This will save a mymap.pgm and mymap.yaml file, name can be edited
Not exactly sure where it will be saved
can also just use rosrun map_server, default names are map.pgm and map.yaml

map_saver subscribes to the OccupancyGrid topic. 
IMPORTANT: the OccupancyGrid must be published to a topic called "map", it doesn't work for
any other topic name as far as I can tell.

---------------------------------------------------------------------------------------------
Loading maps: 
---------------------------------------------------------------------------------------------
map_server <map.yaml>

Example command:
rosrun map_server map_server mymap.yaml

publishes to OccupancyGrid AND MapMetaData


Service: static_map GetMap
http://docs.ros.org/api/nav_msgs/html/srv/GetMap.html

Parameters
~frame_id (string, default: "map")

    The frame to set in the header of the published map. 


