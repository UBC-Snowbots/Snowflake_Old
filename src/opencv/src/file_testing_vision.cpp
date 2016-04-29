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
 * 			- Sketchy, works better if you make a rectangle on the ground
 * Usage: Point to a video file/camera, press m to start/stop calibrating. 
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

    //Camera Initialization
    VideoCapture cap("/home/valerian/Documents/src/opencv/course.mov");
 
    if (!cap.isOpened()){
        cout << "Error opening camera" << endl;
        return -1;
    }

    //Camera information for user
    int width = cap.get(CV_CAP_PROP_FRAME_WIDTH);
    int height = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
    cout << "Frame size: " << width << " x " << height << endl;

    //Working Variables
    Mat inputImage;
    Mat ipmOutput;
    Mat filterOutput;


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

    while(1){ //rosOK

        //Reads image from camera
        bool isRead = cap.read(inputImage);
        if (!isRead){
            cout << "Failed to read image from camera" << endl;
            cap.set(CV_CAP_PROP_POS_AVI_RATIO, 0); //when reading from a file
            continue; 
            //break;
        }

        //Applies the IPM to the image
        ipm.applyHomography(inputImage, ipmOutput);
        ipm.drawPoints(origPoints, inputImage);
        imshow(inputWindow, inputImage);
        imshow(ipmOutputWindow, ipmOutput);

        //Applies the filter to the image
        filter.filterImage(ipmOutput, filterOutput);
        imshow(filterOutputWindow, filterOutput);

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
    }
    return 0;
}
	
