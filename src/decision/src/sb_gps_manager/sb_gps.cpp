/*
* Node: sb_gps under decision 
* Purpose: To make calculations using longitude and latitude data collected from serial from
*          serial driver node 
* Author: Vincent Yuan
* Date: March 7, 2016
*/
#include "sb_gps.h" 


int main(int argc, char **argv){
  ros::init(argc, argv, ROS_NODE_NAME);
  ros::NodeHandle nh; 
  ros::Rate loop_rate(ROS_LOOP_RATE);   
  ros::Subscriber driver_sub = nh.subscribe(INPUT_TOPIC, 20, gpsSubHandle);
  ros::ServiceServer service = nh.advertiseService(SERVICE_NAME, waypointHandle);
  
  while (ros::ok()){}
  return 0; 
}
 
bool waypointHandle(decision::gps_waypoint::Request &req, decision::gps_waypoint::Response &res){
  if (req.next){
    ROS_INFO("Requesting next waypoint");
    res.dx = nextWaypoint.lon; 
    res.dy = nextWaypoint.lat; 
  }
  return true; 
}


void gpsSubHandle(const std_msgs::String::ConstPtr& msg){
  /* Copied function from old NMEA parser written last year
   * Input: Message picked up by subscriber 
   * Output: Void 
   * Note: DO NOT SHOW FINN; Will clean up and re-write later - (Vincent)*/
    
  char a[64]; 
  char temp[8];
  int x,y,i = 0, j; 
  string str = msg->data; 
  
  while (str[i] != '\0'){a[i] = str[i]; i++;}
  if(a[0] != 'B' && a[1] != ','){cout << "no B" << endl; return;}
  if(a[10] != 'e' && a[24] != 'e'){cout << "no e" << endl; return;}
  if(a[14] != ',' && a[28] != ','){cout << "no comma" << endl; return;}
  if(a[33] != '.'){cout << "no compass" << endl; return;} 
  
  for(i = 2, j = 0; i < 9 || j < 7; i++,j++){
    if(a[i]=='.') j--;
    else temp[j] = a[i];
  }
  LastWaypoint.lon = CurrentWaypoint.lon; 
  CurrentWaypoint.lon = atoi(temp); 
  
  for(i = 15, j = 0; i < 10 || j < 8; i++, j++){
    if (a[i] == '.') j--;
    else temp[j] = a[i];
  }
  LastWaypoint.lat = CurrentWaypoint.lat; 
  CurrentWaypoint.lat = atoi(temp); 
  
  return;
} 
