/**
 * A node which creates a point cloud from a binary image
 * Author: Valerian Ratu
 * contact: leri.ratu@gmail.com
 *
 * Subscribes to: image
 * Publishes to: point_cloud
 */

//OpenCv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

//stdlibs
#include <iostream>
#include <unistd.h>

//PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>


using namespace cv;
using namespace std;

const int NUM_ELTS_SKIPPED = 2;
const int MAX_HEIGHT = 50;

class ImageToCloud {
    public:
        ImageToCloud();
        void imageCallBack(const sensor_msgs::ImageConstPtr& msg, std::string topic);
    private:
        Mat inputImage = Mat::zeros(480, 640, CV_8UC1);
	      ros::Publisher pub;
        image_transport::Subscriber sub;
        int x_scale_factor = 1;
        int y_scale_factor = 1;
        int z_scale_factor = 1;
};

ImageToCloud::ImageToCloud(){
	ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    // Get params
    private_nh.getParam("x_scale_factor", x_scale_factor);
    private_nh.getParam("y_scale_factor", y_scale_factor);
    private_nh.getParam("z_scale_factor", z_scale_factor);
    image_transport::ImageTransport it(nh);
    sub = it.subscribe("image", 1, boost::bind(&ImageToCloud::imageCallBack, this, _1, "image"));
	string topic = nh.resolveName("point_cloud");
	uint32_t queue_size = 1;
	ros::Rate loop_rate(5);
	pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>(topic, queue_size);
    bool isCalibratingManually = false;
}


void ImageToCloud::imageCallBack(const sensor_msgs::ImageConstPtr& msg, std::string topic)
{
    try
    {
        inputImage = cv_bridge::toCvShare(msg, "mono8")->image;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'mono8'.", msg->encoding.c_str());
    }

    pcl::PointCloud<pcl::PointXYZ> cloud;
    //Isn't needed? Questionable
    cloud.clear();

    //Creating the cloud and its points
    ros::Time time_stamp = ros::Time::now();
    cloud.header.stamp = time_stamp.toNSec()/1e3;
    cloud.header.frame_id = "laser";
    cloud.width = inputImage.cols * inputImage.rows;
    cloud.height = 1;
    cloud.is_dense = false;
    cloud.points.resize(cloud.width * cloud.height);

    for (int row = 0; row < inputImage.rows; row = row + NUM_ELTS_SKIPPED){
        for (int col = 0; col < inputImage.cols; col = col + NUM_ELTS_SKIPPED){
            if (inputImage.at<uchar>(row, col) > 0){
                for (int i = 0; i < MAX_HEIGHT; i++){
                    pcl::PointXYZ point;
                    point.y = (float) -(col - (inputImage.cols/2)) / y_scale_factor;
                    point.x = (float) -(row - inputImage.rows)/ y_scale_factor;
                    point.z = (float) i/z_scale_factor;
                    cloud.push_back(point);
                }
            }
        }
    }

    //Publishes the cloud
    pub.publish(cloud.makeShared());
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "image_to_cloud");

    ImageToCloud _image_to_cloud;

    ros::spin();

    return 0;
}
