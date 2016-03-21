#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <stdio.h>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include "filter.h"
#include "IPM.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

using namespace cv;
using namespace std;

Mat inputImage;


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    inputImage = cv_bridge::toCvShare(msg, "bgr8")->image;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char** argv){
    
    //Window Names
    string inputWindow = "Input Image";
    string ipmOutputWindow = "IPM Output";
    string ipmPoints = "IPM Points";
    string filterOutputWindow = "Filter Output";

    namedWindow(inputWindow, CV_WINDOW_AUTOSIZE);
    namedWindow(ipmOutputWindow, CV_WINDOW_AUTOSIZE);
    namedWindow(filterOutputWindow, CV_WINDOW_AUTOSIZE);

    int width = 640;
    int height = 480;

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


    inputImage = Mat::zeros(480, 640, CV_32FC4);
    Mat filterOutput;
    Mat ipmOutput;
    Mat workingImage;
    ros::init(argc, argv, "vision_node");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("camera/midcam/image_raw", 1, imageCallback);
    ros::Rate loop_rate(5);

    snowbotsFilter filter(98, 130, 30, 165, 129, 255);
    IPM ipm(Size(width, height), Size(width,height), origPoints, dstPoints);

    while(nh.ok()){
    	if (inputImage.empty()) break;
    	inputImage.copyTo(workingImage);
    	imshow(inputWindow, inputImage);
    	//cvtColor(workingImage, filterOutput, CV_BGR2HSV);
    	//inRange(filterOutput, cv::Scalar(98,30,129), cv::Scalar(130,165,255), ipmOutput);
    	//imshow("Orig Input", workingImage);
    	ipm.applyHomography(workingImage, ipmOutput);
    	imshow(ipmOutputWindow, ipmOutput);
    	ipm.drawPoints(origPoints, workingImage);
    	imshow(ipmPoints, workingImage);
    	filter.filterImage(ipmOutput, filterOutput);
    	imshow(filterOutputWindow, filterOutput);
    	int a = waitKey(20);
    	ros::spinOnce();
    }

    return 0;
}