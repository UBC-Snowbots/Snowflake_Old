#include <ros/ros.h>
#include <string.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <math.h>

using namespace pcl;
using namespace std;

class LaserScanToPointCloud {
public:
    LaserScanToPointCloud();
    void scanCallBack(const sensor_msgs::LaserScan::ConstPtr& scan, std::string topic);
private:
    void scan_topic_parser();

    laser_geometry::LaserProjection projector_;

    ros::NodeHandle node_;
    ros::Publisher pointcloud_publisher_;
    ros::Subscriber laserscan_subscriber_;

    string scan_topic;
    string cloud_topic;

    sensor_msgs::LaserScan scan;
    
};

LaserScanToPointCloud::LaserScanToPointCloud() {
    // Get paramaters
    ros::NodeHandle nh("~");
    nh.getParam("scan_topic", scan_topic);
    nh.getParam("cloud_topic", cloud_topic);
    // Start the publisher
    pointcloud_publisher_ = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>(cloud_topic.c_str(), 1);
     
    
    this->scan_topic_parser();
}

void LaserScanToPointCloud::scan_topic_parser(){
    // Get all topics from master
    ros::master::V_TopicInfo topics;
    ros::master::getTopics(topics);

    for (int i=0; i < topics.size(); i++){
        if ((scan_topic.compare(topics[i].name) == 0) && (topics[i].datatype.compare("sensor_msgs/LaserScan") == 0)){
            laserscan_subscriber_ = node_.subscribe<sensor_msgs::LaserScan> (scan_topic.c_str(), 1, boost::bind(&LaserScanToPointCloud::scanCallBack, this, _1, scan_topic));
            ROS_INFO("Subscribed to topic: %s", scan_topic.c_str());
        }
    }
}

void LaserScanToPointCloud::scanCallBack(const sensor_msgs::LaserScan::ConstPtr& scan, std::string topic){
    // Convert laserscan to a pointcloud and publish it
    // Convert using laser_geometry library
    /*
    sensor_msgs::PointCloud2 cloud;
    projector_.projectLaser(*scan, cloud);
    pointcloud_publisher_.publish(cloud);
    */
    // Convert using in house conversion method 
    pcl::PointCloud<pcl::PointXYZ> cloud;
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(&cloud);  
     
    cloud.clear();

    //Create the cloud and its points
    ros::Time time_stamp = ros::Time::now();
    cloud.header.stamp = time_stamp.toNSec()/1e3;
    cloud.header.frame_id = "laser";
    cloud.width = scan->ranges.size();
    cloud.height = 1;
    cloud.is_dense = false;
    cloud.points.resize(cloud.width * cloud.height);

    for (int i =0; i < scan->ranges.size(); i++){
        //ROS_INFO("%d", i);
        // Make sure the scan point is within the acceptable ranges of the scan
        if ((scan->ranges[i] > scan->range_min) && (scan->ranges[i] < scan->range_max)){
            // Create a new point and add it to the cloud
            pcl::PointXYZ point;
            // YES THESE CALCULATIONS ARE SUPPOSED TO BE INVERTED, it's just how ROS's coordinate system works
            float theta = scan->angle_min + (i * scan->angle_increment);
            point.x = (float) scan->ranges[i] * cos(theta);
            point.y = (float) scan->ranges[i] * sin(theta);
            point.z = (float) 0.1;
            cloud.push_back(point);
        }
    }
    pointcloud_publisher_.publish(cloud.makeShared());
}

int main(int argc, char** argv){
    ros::init(argc, argv, "laserscan_to_pointcloud");

    LaserScanToPointCloud _laserscan_to_pointcloud;

    ros::spin();

    return 0;
}
