#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <string.h>
#include "pcl_ros/point_cloud.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ros/conversions.h>
#include <cstdint>

using namespace std;
using namespace pcl;

class PointCloudMerger {
    public:
        PointCloudMerger();
        void pointCloudCallBack(const sensor_msgs::PointCloud2::ConstPtr& tmp_cloud, std::string topic);
    private:
        void pointcloud_topic_parser();

        ros::NodeHandle node_;
        ros::Publisher pointcloud_publisher_;
        ros::Subscriber cloud_A_subscriber;
        ros::Subscriber cloud_B_subscriber;

        string cloud_A_name;
        string cloud_B_name;
        string merged_cloud_name;

        pcl::PCLPointCloud2 cloud_A;
        pcl::PCLPointCloud2 cloud_B;
        pcl::PCLPointCloud2 merged_cloud;
};

PointCloudMerger::PointCloudMerger(){
    ros::NodeHandle nh("~");
    // Get parameters
    nh.getParam("cloud_a", cloud_A_name);
    nh.getParam("cloud_b", cloud_B_name);
    nh.getParam("merged_cloud", merged_cloud_name);
    // Start Publisher
    pointcloud_publisher_ = node_.advertise<sensor_msgs::PointCloud2> (merged_cloud_name.c_str(), 1, false);

    this->pointcloud_topic_parser();
}

// Gets the request "cloud_a" and "cloud_b" topics from master
void PointCloudMerger::pointcloud_topic_parser() {
    // Get all topics from master
    ros::master::V_TopicInfo topics;
    ros::master::getTopics(topics);

    for (int i=0; i < topics.size(); i++) {
        if ((cloud_A_name.compare(topics[i].name) == 0) && (topics[i].datatype.compare("sensor_msgs/PointCloud2") == 0)) {
            cloud_A_subscriber = node_.subscribe<sensor_msgs::PointCloud2> (cloud_A_name.c_str(), 1, boost::bind(&PointCloudMerger::pointCloudCallBack, this, _1, cloud_A_name));
            ROS_INFO("Subscribed to topic A: %s", cloud_A_name.c_str());
        } else if ((cloud_B_name.compare(topics[i].name) == 0) && (topics[i].datatype.compare("sensor_msgs/PointCloud2") == 0)) {
            cloud_B_subscriber = node_.subscribe<sensor_msgs::PointCloud2> (cloud_B_name.c_str(), 1, boost::bind(&PointCloudMerger::pointCloudCallBack, this, _1, cloud_B_name));
            ROS_INFO("Subscribed to topic B: %s", cloud_B_name.c_str());
        }
    }
}

void PointCloudMerger::pointCloudCallBack(const sensor_msgs::PointCloud2::ConstPtr& tmp_cloud, std::string topic) {
    pcl::PCLPointCloud2 cloud;
    pcl_conversions::toPCL (*tmp_cloud, cloud);
    // Update appropriate cloud
    if (topic == cloud_A_name){
        cloud_A = cloud;
    } else if (topic == cloud_B_name){
        cloud_B = cloud;
    }
    ROS_INFO("Updated: %s", topic.c_str());
    pcl::concatenatePointCloud (cloud_A, cloud_B, merged_cloud);
   
    ros::Time time_stamp = ros::Time::now();
    cloud.header.stamp = time_stamp.toNSec()/1e3;
    cloud.header.frame_id="laser";
    pointcloud_publisher_.publish(merged_cloud);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "point_cloud_merger");

    PointCloudMerger _point_cloud_merger;

    ros::spin();

    return 0;
}