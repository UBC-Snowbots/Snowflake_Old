#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

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
    pcl::PCLPointCloud2 cloud;
};

LaserScanToPointCloud::LaserScanToPointCloud() {
    // Get paramaters
    ros::NodeHandle nh("~");
    nh.getParam("scan_topic", scan_topic);
    nh.getParam("cloud_topic", cloud_topic);
    // Start the publisher
    pointcloud_publisher_ = node_.advertise<sensor_msgs::PointCloud2> (merged_cloud_name.c_str(), 1, false);

    this->scan_topic_parser();
}

LaserScanToPointCloud::scan_topic_parser(){
    // Get all topics from master
    ros::master::V_TopicInfo topics;
    ros::master::getTopics(topics);

    for (int i=0; i < topics.size(); i++){
        if ((scan_topic.compare(topics[i]) == 0) && (topics[i].datatype.compare("sensor_msgs/LaserScan") == 0)){
            laserscan_subscriber_ = node_.subscribe<sensor_msgs::LaserScan> (scan_topic.c_str(), 1, boost::bind(&PointCloudMerger::scanCallback, this, _1, scan_topic));
            ROS_INFO("Subscribed to topic: %s", scan_topic);
        }
    }
}

LaserScanToPointCloud::scanCallBack(const sensor_msgs::LaserScan::ConstPtr& scan, std::string topic){
    // Convert laserscan to a pointcloud and publish it
    sensor_msgs::PointCloud cloud;
    projector_.projectLaser(*scan, cloud);
    pointcloud_publisher_.publish(cloud);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "laserscan_to_pointcloud");

    LaserScanToPointCloud _laserscan_to_pointcloud;

    ros::spin();

    return 0;
}
