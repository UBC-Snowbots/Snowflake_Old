/*
    Basic GPS Driver, gets raw data from gps and publishes it as gps msg (custom msg type)
    Author: Vincent Yuan / Gareth Ellis
    Date: May 18th, 2016
*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sb_messages/gps.h>

using namespace std;


class gpsDriver {
    public:
        gpsDriver();
                   // The raw data from the gps
        void gpsRawCallBack(const std_msgs::String::ConstPtr& msg);
    private:
        ros::Publisher gps_pub;
        ros::Subscriber gps_raw_sub;
        sb_messages::gps present_location;
};

gpsDriver::gpsDriver(){
    ros::NodeHandle nh;
    gps_raw_sub = nh.subscribe("gps_raw_topic", 1, &gpsDriver::gpsRawCallBack, this);
    string topic = nh.resolveName("gps");
    uint32_t queue_size = 1;
    ros::Rate loop_rate(5);
    gps_pub = nh.advertise<sb_messages::gps>(topic, queue_size);
}

void gpsDriver::gpsRawCallBack(const std_msgs::String::ConstPtr& msg){
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
    this->present_location.lon = atoi(temp);

    for(i = 15, j = 0; i < 10 || j < 8; i++, j++){
      if (a[i] == '.') j--;
      else temp[j] = a[i];
    }
    this->present_location.lat = atoi(temp);

    gps_pub.publish(present_location);
    return;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "gps_driver");

  gpsDriver gps_driver;

  ros::spin();

  return 0;
}
