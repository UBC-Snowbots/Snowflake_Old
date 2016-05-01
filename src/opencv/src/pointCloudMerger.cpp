#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>


class PointCloudMerger {
    public:
        PointCloudMerger;
    private:
        void pointcloud_topic_parser();

        ros::NodeHandle node_;
        ros::Publisher pointcloud_publisher_;
        ros::Subscriber cloud_A_subscriber;
        ros::Subscriber cloud_B_subscriber;

        string cloud_A_name;
        string cloud_B_name;
        string destination_cloud_name;

        sensor_msgs::PointCloud2 cloud_A;
        sensor_msgs::PointCloud2 cloud_B;
        sensor_msgs::PointCloud2 merged_cloud;


    PointCloudMerger::PointCloudMerger(){
        ros::NodeHandle nh("~");
        // Get parameters
        nh.getParam("cloud_a", cloud_A_name);
        nh.getParam("cloud_b", cloud_B_name);
        nh.getParam("destination_cloud", destination_cloud_name);
        // Start Publisher
        pointcloud_publisher_ = node_.advertise<sensor_msgs::PointCloud2> (destination_cloud_name.c_str(), 1, false);

        this->pointcloud_topic_parser();
    }

    // Gets the request "cloud_a" and "cloud_b" topics from master
    void PointCloudMerger::pointcloud_topic_parser() {
        // Get all topics from master
        ros::master::V_TopicInfo topics;
        ros::master::getTopics(topics);

        for (int i=0; i < topics.size(); i++) {
            if ((cloud_A_name.compare(topics[i].name) == 0) && (topics[i].datatype.compare("sensor_msgs/PointCloud2") == 0)) {
                cloud_A_subscriber =  node_.subscribe<sensor_msgs::PointCloud2> (cloud_A_name.c_str(), 1, pointCloudCallBack);
                ROS_INFO("Subscribed to topic A: %s", cloud_A_name);
            } else if ((cloud_B_name.compare(topics[i].name) == 0) && (topics[i].datatype.compare("sensor_msgs/PointCloud2") == 0)) {
                cloud_B_subscriber = node_.subscribe<sensor_msgs::PointCloud2> (cloud_B_name.c_str(), 1, pointCloudCallBack);
                ROS_INFO("Subscribed to topic B: %s", cloud_B_name);
            }
        }
    }

    void pointCloudCallBack(const sensor_msgs::PointCloud2::ConstPtr& cloud, std::string topic) {
        // Update appropriate cloud
        if (topic == cloud_A_name){
            cloud_A = cloud;
        } else if (topic == cloud_B_name){
            cloud_B = cloud;
        }
        destination_cloud = pcl::concatenateFields (cloud_A, cloud_B, merged_cloud);
        pointcloud_publisher_.publish(merged_cloud);
    }

    int main(int argc, char** argv){
        ros::init(argc, argv, "point_cloud_merger");

        PointCloudMerger _point_cloud_merger;

        ros::spin();

        return 0;
    }
}
