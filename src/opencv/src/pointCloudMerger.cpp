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
        void mergeClouds();

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
        uint32_t seq; // Sequence ID: consecutively increasing
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

    seq = 0;
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
    // Update appropriate cloud
    pcl::PCLPointCloud2 cloud;
    pcl_conversions::toPCL (*tmp_cloud, cloud);
    if (topic == cloud_A_name){
        cloud_A = cloud;
    } else if (topic == cloud_B_name){
        cloud_B = cloud;
    }
    // If both scans come in, merge them
    if ((cloud_A.height != 0) && (cloud_B.height != 0)){
        mergeClouds();
    } else {
        if (cloud_A.height != 0) {
            merged_cloud = cloud_A;
        } else if (cloud_B.height != 0){
            merged_cloud = cloud_B;
        }
    }
    pointcloud_publisher_.publish(merged_cloud);
}

// Merges the first three fields (x,y,z) of clouds A and B
// both clouds must have matching field datatypes and a count of 1
void PointCloudMerger::mergeClouds() {
    if (cloud_A.fields.size() < 3){
        ROS_INFO("Error: %s has less then three fields", cloud_A_name.c_str());
        return;
    } else if (cloud_B.fields.size() < 3){
        ROS_INFO("Error: %s has less then three fields", cloud_B_name.c_str());
        return;
    } else {
        for (int i=0; i < 3; i++){
            if (cloud_A.fields[i].datatype != cloud_B.fields[i].datatype){
                ROS_INFO("Error: given clouds have fields with different datatypes");
                return;
            } else if (cloud_A.fields[i].count != 1){
                ROS_INFO("Error: %s has a field with count > 1", cloud_A_name.c_str());
                return;
            } else if (cloud_B.fields[i].count != 1){
                ROS_INFO("Error: %s has a field with count > 1", cloud_B_name.c_str());
                return;
            }
        }
    }
    merged_cloud.header.seq = seq;
    merged_cloud.header.frame_id = 1;
    ros::Time time_st = ros::Time::now ();
    merged_cloud.header.stamp = time_st.toNSec()/1e3;
    merged_cloud.height = 1;
    merged_cloud.width = cloud_A.width + cloud_B.width;
    sensor_msgs::PointField fields[3];
    for (int i=0; i < 3; i++){
        merged_cloud.fields.push_back(cloud_A.fields[i]);
    }
    merged_cloud.is_bigendian = false;
    if (cloud_A.fields.size() == 3){
        merged_cloud.point_step = cloud_A.point_step;
    } else {
        merged_cloud.point_step = cloud_A.fields[3].offset;
    }
    merged_cloud.row_step = merged_cloud.point_step * merged_cloud.width;
    for (int i=0; i < cloud_A.width; i++){
        for (int j=0; j < merged_cloud.point_step; j++){
            merged_cloud.data.push_back(cloud_A.data[(i * cloud_A.point_step) + j]);
        }
    }
    for (int i=0; i < cloud_A.width; i++){
        for (int j=0; j < merged_cloud.point_step; j++){
            merged_cloud.data.push_back(cloud_B.data[(i * cloud_B.point_step) + j]);
        }
    }
    merged_cloud.is_dense = false;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "point_cloud_merger");

    PointCloudMerger _point_cloud_merger;

    ros::spin();

    return 0;
}
