/*
    Advertises a fake gps message for testing
*/

#include "ros/ros.h"
#include "math.h"
#include "sb_messages/gps.h"
#include "geometry_msgs/Pose2D.h"


int main(int argc, char **argv){
  ros::init(argc, argv, "gps_decision_test");
  ros::NodeHandle nh;
  ros::Publisher gps_pub = nh.advertise<sb_messages::gps>("GPS", 10);

  ros::Rate loop_rate(10);

  while(ros::ok()){
      sb_messages::gps gps_msg;
      gps_msg.lon = -83.024025;
      gps_msg.lat = 42.212682;
      gps_msg.head = 101.985; 
      gps_pub.publish(gps_msg);
      ros::spinOnce();
      loop_rate.sleep();
  }

  return 0;
}
