/*
 * Analyzes an image and transforms it into a binary image
 * given some sort of colour specifiction
 * 
 * Current method: Conversion to HSV (Hue, Saturation, Value) and filtering
 * the numbers appropriately. Currently trying out methods to find 
 * which HSV thresholds are the best and see if it can't be
 * automated properly.
 * Ref: https://raw.githubusercontent.com/kylehounslow/opencv-tuts/master/auto-colour-filter/AutoColourFilter.cpp
 *          - Just the HSV value finding rectangle part, not the object tracking part.
 *          - Sketchy, works better if you make a rectangle on the ground
 * Usage: Point to a video file/camera, press m to start/stop calibrating.
 *
 * Subscribes to: camera/midcam/image_raw
 * Publishes to: vision/binary_image
 */   

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
    string filterOutputWindow = "Filter Output";
    namedWindow(inputWindow, CV_WINDOW_AUTOSIZE);
    namedWindow(ipmOutputWindow, CV_WINDOW_AUTOSIZE);
    namedWindow(filterOutputWindow, CV_WINDOW_AUTOSIZE);

    //Calibration Variables
    bool isCalibratingManually = false;

    //Working Variables
    Mat ipmOutput;
    Mat filterOutput;
    Mat workingInput;
    inputImage = Mat::zeros(480, 640, CV_32FC4);
    cout << inputImage.depth() << endl;

    //ROS
    ros::init(argc, argv, "vision_node");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("camera/midcam/image_raw", 1, imageCallback);
    image_transport::Publisher pub = it.advertise("vision/binary_image", 1);
    ros::Rate loop_rate(5); 
    //Manual initialization, maybe automate through parameter server
    int width = 640;
    int height = 480;
    cout << "Frame size: " << width << " x " << height << endl;

    //Creating Mutex Lock to test things out

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
    
    //Creating the binary filter
    snowbotsFilter filter(98, 130, 30, 165, 129, 255);

    //Creating the IPM transformer
    IPM ipm(Size(width, height), Size(width,height), origPoints, dstPoints);

    while(nh.ok()){ //rosOK

        //Image is empty so quit
        if (inputImage.empty()) break;
        inputImage.copyTo(workingInput);
        //Applies the IPM to the image
                
        ipm.applyHomography(workingInput, ipmOutput);
        ipm.drawPoints(origPoints, workingInput);
        imshow(inputWindow, workingInput);
        imshow(ipmOutputWindow, ipmOutput);

        //Applies the filter to the image
        filter.filterImage(ipmOutput, filterOutput);
        imshow(filterOutputWindow, filterOutput);
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", filterOutput).toImageMsg();
        pub.publish(msg);        

        //Calibration
        if (isCalibratingManually){
            filter.manualCalibration();
        } else {
            filter.stopManualCalibration();
        }

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
        ros::spinOnce();
    }
    return 0;
}
    
