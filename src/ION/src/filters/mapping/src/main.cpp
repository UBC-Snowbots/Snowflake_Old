#include "ros/ros.h"
#include "std_msgs/String.h"
#include "C_GlobalMap/GlobalMap.h"
#include "C_Localization/Localization.h"
#include "names.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, MAIN_NODE);
  ros::NodeHandle n;
  GlobalMap globalMap(10,10);
  Localization localize(&globalMap);
  
  ros::Subscriber sub = n.subscribe(LOCAL_MAP_TOPIC, 2, &Localization::LocalMapCallback, &localize);

  ros::Rate loop_rate(10);
  
  while (ros::ok())
  {
    ROS_INFO("Map size: %d.", globalMap.GetMapSize());
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
