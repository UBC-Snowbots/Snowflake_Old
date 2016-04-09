#include <ros/ros.h>
#include <string.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include "sensor_msgs/LaserScan.h"
#include "pcl_ros/point_cloud.h"
#include <Eigen/Dense>
#include <dynamic_reconfigure/server.h>
#include <ira_laser_tools/laserscan_multi_mergerConfig.h>

using namespace std;
using namespace pcl;
using namespace laserscan_multi_merger;

class LaserscanMerger
{
public:
    LaserscanMerger();
    void laserscanCallback(const sensor_msgs::LaserScan::ConstPtr& scan, std::string topic);
    void pointcloudCallback(const sensor_msgs::PointCloud::ConstPtr& input_pointcloud, std::string topic);
    void pointcloud_to_laserscan(Eigen::MatrixXf points, pcl::PCLPointCloud2 *merged_cloud);
    void reconfigureCallback(laserscan_multi_mergerConfig &config, uint32_t level);

private:
    ros::NodeHandle node_;
    laser_geometry::LaserProjection projector_;
    tf::TransformListener tfListener_;

    ros::Publisher point_cloud_publisher_;
    ros::Publisher laser_scan_publisher_;
    vector<ros::Subscriber> laserscan_subscribers;
    vector<ros::Subscriber> pointcloud_subscribers;
    vector<bool> clouds_modified;

    vector<pcl::PCLPointCloud2> clouds;
    vector<string> laserscan_input_topics;
    vector<string> pointcloud_input_topics;

    void topic_parser();

    double angle_min;
    double angle_max;
    double angle_increment;
    double time_increment;
    double scan_time;
    double range_min;
    double range_max;

    string destination_frame;
    string cloud_destination_topic;
    string scan_destination_topic;
    string input_topics;
};


void LaserscanMerger::reconfigureCallback(laserscan_multi_mergerConfig &config, uint32_t level)
{
	this->angle_min = config.angle_min;
	this->angle_max = config.angle_max;
	this->angle_increment = config.angle_increment;
	this->time_increment = config.time_increment;
	this->scan_time = config.scan_time;
	this->range_min = config.range_min;
	this->range_max = config.range_max;
}


void LaserscanMerger::topic_parser()
{
	// Get all topics presently being published to master
	ros::master::V_TopicInfo topics;
	ros::master::getTopics(topics);

    // Parse out the requested topic names from the strings passed in as params
  istringstream iss(input_topics);
	vector<string> tokens;
	copy(istream_iterator<string>(iss), istream_iterator<string>(), back_inserter<vector<string> >(tokens));

	vector<string> tmp_laserscan_input_topics;
  vector<string> tmp_pointcloud_input_topics;

    // Look through all the topics presently being published to master, and find each requested topic name,
    // adding it to a temporary list of pointcloud/laserscan input topics, depending on what topic type the
    // topic on master is
	for(int i=0;i<tokens.size();++i)
	{
	        for(int j=0;j<topics.size();++j)
		{
			if( (tokens[i].compare(topics[j].name) == 0) && (topics[j].datatype.compare("sensor_msgs/LaserScan") == 0) )
			{
				tmp_laserscan_input_topics.push_back(topics[j].name);
			}

            else if( (tokens[i].compare(topics[j].name) == 0) && (topics[j].datatype.compare("sensor_msgs/PointCloud") == 0) )
			{
				tmp_pointcloud_input_topics.push_back(topics[j].name);
			}
		}
	}

    // Get iterators for laserscan and pointcloud input topics
	sort(tmp_laserscan_input_topics.begin(),tmp_laserscan_input_topics.end());
	std::vector<string>::iterator laserscan_last = std::unique(tmp_laserscan_input_topics.begin(), tmp_laserscan_input_topics.end());
	tmp_laserscan_input_topics.erase(laserscan_last, tmp_laserscan_input_topics.end());

  sort(tmp_pointcloud_input_topics.begin(), tmp_pointcloud_input_topics.end());
  std::vector<string>::iterator pointcloud_last = std::unique(tmp_pointcloud_input_topics.begin(), tmp_pointcloud_input_topics.end());
  tmp_pointcloud_input_topics.erase(pointcloud_last, tmp_pointcloud_input_topics.end());

	// This section deals with subscribing/resubscribing to given laserscan topics
	// Do not re-subscribe if laserscan topics are the same
    if(!equal(tmp_laserscan_input_topics.begin(),tmp_laserscan_input_topics.end(),tmp_laserscan_input_topics.begin()))
	{

		// Unsubscribe from previous topics
		for(int i=0; i<laserscan_subscribers.size(); ++i)
			laserscan_subscribers[i].shutdown();

		laserscan_input_topics = tmp_laserscan_input_topics;
		if(laserscan_input_topics.size() > 0)
		{
      laserscan_subscribers.resize(laserscan_input_topics.size());
			clouds_modified.resize(laserscan_input_topics.size() + pointcloud_input_topics.size());
			clouds.resize(laserscan_input_topics.size() + pointcloud_input_topics.size());
      ROS_INFO("Subscribing to %d topics", laserscan_subscribers.size());
            //ROS_INFO("Subscribing to topics\t%ld", laserscan_subscribers.size());
			for(int i=0; i<laserscan_input_topics.size(); ++i)
			{
                // TODO: THE BELOW LINE STILL NEEDS WORK - LASERSCANS AND POINTCLOUDS NOW
        laserscan_subscribers[i] = node_.subscribe<sensor_msgs::LaserScan>
          (laserscan_input_topics[i].c_str(), 1,
          boost::bind(&LaserscanMerger::laserscanCallback,this, _1, laserscan_input_topics[i]));
				clouds_modified[i] = false;
				cout << laserscan_input_topics[i] << " ";
			}
		}
		else ROS_INFO("Not subscribed to any laserscan topic.");
	}

	// This section deals with subscribing/resubscribing to given pointcloud topics
	// Do not re-subscribe if pointcloud topics are the same
    if(!equal(tmp_pointcloud_input_topics.begin(),tmp_pointcloud_input_topics.end(),tmp_pointcloud_input_topics.begin()))
	{

		// Unsubscribe from previous topics
		for(int i=0; i<pointcloud_subscribers.size(); ++i)
			pointcloud_subscribers[i].shutdown();

		pointcloud_input_topics = tmp_pointcloud_input_topics;
		if(pointcloud_input_topics.size() > 0)
		{
      pointcloud_subscribers.resize(pointcloud_input_topics.size());
			clouds_modified.resize(pointcloud_input_topics.size() + pointcloud_input_topics.size());
			clouds.resize(pointcloud_input_topics.size() + pointcloud_input_topics.size());
      ROS_INFO("Subscribing to %d topics", pointcloud_subscribers.size());
            //ROS_INFO("Subscribing to topics\t%ld", pointcloud_subscribers.size());
			for(int i=0; i<pointcloud_input_topics.size(); ++i)
			{
                // TODO: THE BELOW LINE STILL NEEDS WORK - LASERSCANS AND POINTCLOUDS NOW
        pointcloud_subscribers[i] = node_.subscribe<sensor_msgs::PointCloud>
          (pointcloud_input_topics[i].c_str(), 1,
          boost::bind(&LaserscanMerger::pointcloudCallback,this, _1, pointcloud_input_topics[i]));
				clouds_modified[i] = false;
				cout << pointcloud_input_topics[i] << " ";
			}
		}
		else ROS_INFO("Not subscribed to any pointcloud topic.");
	}

}


LaserscanMerger::LaserscanMerger()
{
	ros::NodeHandle nh("~");

	nh.getParam("destination_frame", destination_frame);
	nh.getParam("cloud_destination_topic", cloud_destination_topic);
	nh.getParam("scan_destination_topic", scan_destination_topic);
  nh.getParam("scan_and_pointcloud_topics", input_topics);

    this->topic_parser();

	point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud2> (cloud_destination_topic.c_str(), 1, false);
	laser_scan_publisher_ = node_.advertise<sensor_msgs::LaserScan> (scan_destination_topic.c_str(), 1, false);

	//tfListener_.setExtrapolationLimit(ros::Duration(0.1));
}


void LaserscanMerger::laserscanCallback(const sensor_msgs::LaserScan::ConstPtr& scan, std::string topic)
{
  std::cout << "Trying to convert a laserscan!" << std::endl;
	sensor_msgs::PointCloud tmpCloud1,tmpCloud2;
	sensor_msgs::PointCloud2 tmpCloud3;

    // Verify that TF knows how to transform from the received scan to the destination scan frame
	tfListener_.waitForTransform(scan->header.frame_id.c_str(), destination_frame.c_str(), scan->header.stamp, ros::Duration(1));

	projector_.transformLaserScanToPointCloud(scan->header.frame_id, *scan, tmpCloud1, tfListener_);
	try
	{
		tfListener_.transformPointCloud(destination_frame.c_str(), tmpCloud1, tmpCloud2);
	}catch (tf::TransformException ex){ROS_ERROR("%s",ex.what());return;}

	for(int i=0; i<laserscan_input_topics.size(); ++i)
	{
		if(topic.compare(laserscan_input_topics[i]) == 0)
		{
			sensor_msgs::convertPointCloudToPointCloud2(tmpCloud2,tmpCloud3);
			pcl_conversions::toPCL(tmpCloud3, clouds[i]);
			clouds_modified[i] = true;
		}
	}

    // Count how many scans we have
	int totalClouds = 0;
	for(int i=0; i<clouds_modified.size(); ++i)
		if(clouds_modified[i])
			++totalClouds;

    // Go ahead only if all subscribed scans/pointclouds have arrived
	if(totalClouds == clouds_modified.size())
	{
		pcl::PCLPointCloud2 merged_cloud = clouds[0];
		clouds_modified[0] = false;

		for(int i=1; i<clouds_modified.size(); ++i)
		{
			pcl::concatenatePointCloud(merged_cloud, clouds[i], merged_cloud);
			clouds_modified[i] = false;
		}

		point_cloud_publisher_.publish(merged_cloud);

		Eigen::MatrixXf points;
		getPointCloudAsEigen(merged_cloud,points);

		pointcloud_to_laserscan(points, &merged_cloud);
	}
}


void LaserscanMerger::pointcloudCallback(const sensor_msgs::PointCloud::ConstPtr& input_pointcloud, std::string topic)
{
  std::cout << "Trying to convert a pointcloud!" << std::endl;
	sensor_msgs::PointCloud tmpCloud1,tmpCloud2;
	sensor_msgs::PointCloud2 tmpCloud3;

    // Verify that TF knows how to transform from the received pointcloud  to the destination scan frame
	tfListener_.waitForTransform(input_pointcloud->header.frame_id.c_str(), destination_frame.c_str(), input_pointcloud->header.stamp, ros::Duration(1));

	for(int i=0; i<pointcloud_input_topics.size(); ++i)
	{
		if(topic.compare(pointcloud_input_topics[i]) == 0)
		{
			sensor_msgs::convertPointCloudToPointCloud2(tmpCloud2,tmpCloud3);
			pcl_conversions::toPCL(tmpCloud3, clouds[i]);
			clouds_modified[i] = true;
		}
	}

    // Count how many scans we have
	int totalClouds = 0;
	for(int i=0; i<clouds_modified.size(); ++i)
		if(clouds_modified[i])
			++totalClouds;

    // Go ahead only if all subscribed scans/pointclouds have arrived
	if(totalClouds == clouds_modified.size())
	{
		pcl::PCLPointCloud2 merged_cloud = clouds[0];
		clouds_modified[0] = false;

		for(int i=1; i<clouds_modified.size(); ++i)
		{
			pcl::concatenatePointCloud(merged_cloud, clouds[i], merged_cloud);
			clouds_modified[i] = false;
		}

		point_cloud_publisher_.publish(merged_cloud);

		Eigen::MatrixXf points;
		getPointCloudAsEigen(merged_cloud,points);

		pointcloud_to_laserscan(points, &merged_cloud);
	}
}

void LaserscanMerger::pointcloud_to_laserscan(Eigen::MatrixXf points, pcl::PCLPointCloud2 *merged_cloud)
{
	sensor_msgs::LaserScanPtr output(new sensor_msgs::LaserScan());
	output->header = pcl_conversions::fromPCL(merged_cloud->header);
	output->header.frame_id = destination_frame.c_str();
	output->header.stamp = ros::Time::now();  //fixes #265
	output->angle_min = this->angle_min;
	output->angle_max = this->angle_max;
	output->angle_increment = this->angle_increment;
	output->time_increment = this->time_increment;
	output->scan_time = this->scan_time;
	output->range_min = this->range_min;
	output->range_max = this->range_max;

	uint32_t ranges_size = std::ceil((output->angle_max - output->angle_min) / output->angle_increment);
	output->ranges.assign(ranges_size, output->range_max + 1.0);

	for(int i=0; i<points.cols(); i++)
	{
		const float &x = points(0,i);
		const float &y = points(1,i);
		const float &z = points(2,i);

		if ( std::isnan(x) || std::isnan(y) || std::isnan(z) )
		{
			ROS_DEBUG("rejected for nan in point(%f, %f, %f)\n", x, y, z);
			continue;
		}

		double range_sq = y*y+x*x;
		double range_min_sq_ = output->range_min * output->range_min;
		if (range_sq < range_min_sq_) {
			ROS_DEBUG("rejected for range %f below minimum value %f. Point: (%f, %f, %f)", range_sq, range_min_sq_, x, y, z);
			continue;
		}

		double angle = atan2(y, x);
		if (angle < output->angle_min || angle > output->angle_max)
		{
			ROS_DEBUG("rejected for angle %f not in range (%f, %f)\n", angle, output->angle_min, output->angle_max);
			continue;
		}
		int index = (angle - output->angle_min) / output->angle_increment;


		if (output->ranges[index] * output->ranges[index] > range_sq)
			output->ranges[index] = sqrt(range_sq);
	}

	laser_scan_publisher_.publish(output);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "laser_multi_merger");

    LaserscanMerger _laser_merger;

    dynamic_reconfigure::Server<laserscan_multi_mergerConfig> server;
    dynamic_reconfigure::Server<laserscan_multi_mergerConfig>::CallbackType f;

    f = boost::bind(&LaserscanMerger::reconfigureCallback,&_laser_merger, _1, _2);
	server.setCallback(f);

	ros::spin();

	return 0;
}
