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
#include <iostream>

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
const int MAX_HEIGHT = 20;
const int SCALE_FACTOR = 100;

Mat inputImage;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    inputImage = cv_bridge::toCvShare(msg, "mono8")->image;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'mono8'.", msg->encoding.c_str());
  }
}

int main (int argc, char** argv){

    //Ros initialization
	ros::init(argc, argv, "image_to_cloud_node");
	ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("image", 1, imageCallback);
	string topic = nh.resolveName("point_cloud");
	uint32_t queue_size = 1;
	ros::Rate loop_rate(5);
	ros::Publisher pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>(topic, queue_size);

    //Creating the viewer object
    pcl::visualization::PCLVisualizer viewer("3D Viewer");
    viewer.setBackgroundColor (0, 0, 0);
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "input");
    viewer.addCoordinateSystem (1.0);
    viewer.initCameraParameters ();
    
    bool cloudSet = false;

    //Cloud and pointer initialization
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(&cloud);

    //Empty Image initializatoin
    inputImage = Mat::zeros(480, 640, CV_8UC1);
	
    while (nh.ok()){

        //Isn't needed? Questionable
        cloud.clear();

        //Creating the cloud and its points
		cloud.header.frame_id = "map";
		cloud.width = inputImage.cols * inputImage.rows;
		cloud.height = 1;
		cloud.is_dense = false;
		cloud.points.resize(cloud.width * cloud.height);

		for (int row = 0; row < inputImage.rows; row = row + NUM_ELTS_SKIPPED){
			for (int col = 0; col < inputImage.cols; col = col + NUM_ELTS_SKIPPED){
				if (inputImage.at<uchar>(row, col) > 0){
                    for (int i = 0; i < MAX_HEIGHT; i++){
                        pcl::PointXYZ point;
                        point.y = (float) -(col - (inputImage.cols/2)) /SCALE_FACTOR;
                        point.x = (float) -(row - inputImage.rows)/SCALE_FACTOR;
					    point.z = (float) i/SCALE_FACTOR;
                        cloud.push_back(point);
                    }
				} 
			}
		}

        //Publishes the cloud
		//cloud.header.stamp = ros::Time::now().toNSec();
		pub.publish(cloud_ptr);
		
    
		//Escape key to finish program
        int a = waitKey(20);
        if (a == 27){
            cout << "Escaped by user" << endl;
            break;
        }

        //Show the cloud in the viewer
        if (!cloudSet){
            viewer.addPointCloud(cloud_ptr, "input");
            cloudSet = true;
        } else{
            viewer.updatePointCloud(cloud_ptr, "input"); 
        }

        //Needed to update the viewer and to keep ros working
		viewer.spinOnce();
		ros::spinOnce();
	}

	/* Iterators hard to get x,y value
	MatIterator_<uchar> it = image.begin<uchar>();
	MatIterator_<uchar> itend = image.end<uchar>();

	for (;it != itend; it++){
		int a = (int) (*it);
		cout << a << " ";
	}
	*/



	return 0;
}	
