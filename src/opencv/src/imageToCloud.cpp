
//OpenCv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

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

//Filter
#include "filter.h"
#include "IPM.h"


using namespace cv; 
using namespace std; 

const int NUM_ELTS_SKIPPED = 2;

boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}


int main (int argc, char** argv){

	VideoCapture cap("/home/valerian/Documents/src/opencv/course.mov");

	if (!cap.isOpened()){
		cout << "File can't be opened" << endl;
		return -1;
	}

  	//Window Names
    string inputWindow = "Input Image";
    string ipmOutputWindow = "IPM Output";
    string filterOutputWindow = "Filter Output";
    namedWindow(inputWindow, CV_WINDOW_AUTOSIZE);
    //namedWindow(ipmOutputWindow, CV_WINDOW_AUTOSIZE);
    namedWindow(filterOutputWindow, CV_WINDOW_AUTOSIZE);

    //Calibration Variables
    bool isCalibratingManually = false;

    //Working Variables
    Mat inputImage;
    Mat ipmOutput;
    Mat filterOutput;
    Mat workingInput;
    inputImage = Mat::zeros(480, 640, CV_32FC4);

    cap.read(inputImage);

    int width = inputImage.cols;
    int height = inputImage.rows;

    //Manually Setting up the IPM points
    vector<Point2f> origPoints;
    origPoints.push_back( Point2f(0, height));
    origPoints.push_back( Point2f(width, height));
    origPoints.push_back( Point2f(width/2+100, 200));
    origPoints.push_back( Point2f(width/2-100, 200));

    vector<Point2f> dstPoints;
    dstPoints.push_back( Point2f(0, height) );
    dstPoints.push_back( Point2f(width, height) );
    dstPoints.push_back( Point2f(width, 0) );
    dstPoints.push_back( Point2f(0, 0) );

	ros::init(argc, argv, "image_to_cloud_node");
	ros::NodeHandle nh;
	string topic = nh.resolveName("point_cloud");
	uint32_t queue_size = 1;
	ros::Rate loop_rate(5);
	ros::Publisher pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>(topic, queue_size);

	//Creating the binary filter
    snowbotsFilter filter(80, 128, 13, 125, 185, 255);

    //Creating the IPM transformer
    IPM ipm(Size(width, height), Size(width,height), origPoints, dstPoints);


    //Creating the viewer object
    pcl::visualization::PCLVisualizer viewer("3D Viewer");
    viewer.setBackgroundColor (0, 0, 0);
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer.addCoordinateSystem (1.0);
    viewer.initCameraParameters ();

    bool cloudSet = false;

    //Cloud and pointer initialization
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(&cloud);
	
    while (nh.ok()){

		cap.read(inputImage);

		//Image is empty so quit
        if (inputImage.empty()) continue;

        inputImage.copyTo(workingInput);
        imshow(inputWindow, workingInput);

        //Applies the filter to the image
        filter.filterImage(workingInput, filterOutput);
        imshow(filterOutputWindow, filterOutput);       

        //Calibration
        if (isCalibratingManually){
            filter.manualCalibration();
        } else {
            filter.stopManualCalibration();
        }

		cloud.header.frame_id = "map";
		cloud.width = inputImage.cols * inputImage.rows;
		cloud.height = 1;
		cloud.is_dense = false;
		cloud.points.resize(cloud.width * cloud.height);

		for (int row = 0; row < filterOutput.rows; row = row + NUM_ELTS_SKIPPED){
			for (int col = 0; col < filterOutput.cols; col = col + NUM_ELTS_SKIPPED){
				cloud.points[col+row*filterOutput.cols].x = col;
				cloud.points[col+row*filterOutput.cols].y = row;
				if (filterOutput.at<uchar>(row, col) > 0){
					cloud.points[col+row*filterOutput.cols].z = 20;
				} else {
					cloud.points[col+row*filterOutput.cols].z = 0;
				}
			}
		}

		cloud.header.stamp = ros::Time::now().toNSec();
		pub.publish(cloud);
		
    
		//Escape key to finish program
        int a = waitKey(20);
        if (a == 27){
            cout << "Escaped by user" << endl;
            break;
        }
        //Press 'm' to calibrate manually
        else if (a == 109){ 
            if (!isCalibratingManually){
                cout << "Beginning manual calibration" << endl;     
            } else {
                cout << "Ending manual calibration" << endl;
            }
            isCalibratingManually = !isCalibratingManually;
            filter.printValues();
        }

        if (!cloudSet){
            viewer.addPointCloud(cloud_ptr, "input");
            cloudSet = true;
        } else{
            viewer.updatePointCloud(cloud_ptr, "input"); 
        }
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