Add map storage code here (must provide services to allow filters to provide/contest obstacles, and a current robot pose)

Inputs, to be broadcasted from filters:
---------------------------------------

 - Obstacles to be merged in (message of type nav_msgs/OccupancyGrid, probably should be latched for performance)
 
 - Robot pose (message of type geometry_msgs::PoseStamped)

Outputs (latched message, changes when input changes)
-----------------------------------------------------

 - Full occupancy grid (message of type nav_msgs/OccupancyGrid, latched for performance)
 
 - Mapping transform (message of type tf/Transform), so other nodes can convert map coordinates to robot-relative coordinates
 such as those needed for twist messages, etc...
