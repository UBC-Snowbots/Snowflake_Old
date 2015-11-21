#include <ros/ros.h>
#include <laser_geometry/laser_geometry.h>

class LidarFilter{
	private:
	ros::Publisher points_publisher;
	ros::Subscriber scan_subscriber;
	laser_geometry::LaserProjection projection;
	public:
	LidarFilter(){
		ros::NodeHandle node;
		scan_subscriber = node.subscribe<sensor_msgs::LaserScan> ("/lidar", 100, &LidarFilter::lidarDataCallback, this);
		points_publisher = node.advertise<sensor_msgs::PointCloud2> ("/map_points", 100, false);
	}
	void lidarDataCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
		sensor_msgs::PointCloud2 output_cloud;
		projection.projectLaser(*scan, output_cloud);
		points_publisher.publish(output_cloud);
	}
};

int main(int argc, char** argv){
	ros::init(argc, argv, "lidar_filter");
	LidarFilter filter;
	ros::spin();
	return 0;
}
