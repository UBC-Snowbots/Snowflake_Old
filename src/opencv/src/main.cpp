/*
 * Project:  Inverse Perspective Mapping
 *
 * File:     main.cpp
 *
 * Contents: Creation, initialisation and usage of IPM object
 *           for the generation of Inverse Perspective Mappings of images or videos
 *
 * Author:   Marcos Nieto <marcos.nieto.doncel@gmail.com>
 * Date:	 22/02/2014
 * Homepage: http://marcosnietoblog.wordpress.com/
 * 
 * Modified liberally to give Elsa some eyes
 * TODO: Better documentation
 * TODO: ROS integration
 */


#include "opencv2/core/core.hpp"
#include <stdio.h>
#include <ctime>
#include "IPM.h"
 // Use the image_transport classes instead.
#include <ros/ros.h>
//#include <image_transport/image_transport.h>
    
using namespace cv;
using namespace std;

int main( int _argc, char** _argv )
{
    //ros::init(); 
    //ros::NodeHandle nh("ipm");
    //ros::Rate loop_rate(30);
    //image_transport::ImageTransport it(nh);
    //image_transport::Subscriber sub = it.subscribe("in_image_base_topic", 1, imageCallback);
    //image_transport::Publisher pub = it.advertise("out_image_base_topic", 1);

    VideoCapture cap(0); // open the video camera no. 0

    if (!cap.isOpened())  // if not success, exit program
    {
        cout << "Cannot open the video cam" << endl;
        return -1;
    }

    Mat inputImage;
    Mat outputImage;

    int width = cap.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
    int height = cap.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video

    cout << "Frame size : " << width << " x " << height << endl;

    namedWindow("MyVideo",CV_WINDOW_AUTOSIZE); //create a window called "MyVideo"
    namedWindow("ipm", CV_WINDOW_AUTOSIZE);

    vector<Point2f> origPoints;
    origPoints.push_back( Point2f(0, height));
    origPoints.push_back( Point2f(width, height));
    origPoints.push_back( Point2f(width/2+100, 200));
    origPoints.push_back( Point2f(width/2-100, 200));

    // The 4-points correspondences in the destination image
    
    //These doesn't need to be changed
    vector<Point2f> dstPoints;
    dstPoints.push_back( Point2f(0, height) );
    dstPoints.push_back( Point2f(width, height) );
    dstPoints.push_back( Point2f(width, 0) );
    dstPoints.push_back( Point2f(0, 0) );
    
    IPM ipm(Size(width, height), Size(width,height), origPoints, dstPoints);
    
    while (1)//ros::ok())
    {
        bool bSuccess = cap.read(inputImage); // read a new frame from video

        if (!bSuccess) //if not success, break loop
        {
            cout << "Cannot read a frame from video stream" << endl;
            break;
        }
        ipm.applyHomography(inputImage, outputImage);
        ipm.drawPoints(origPoints, inputImage);
        imshow("MyVideo", inputImage); //show the frame in "MyVideo" window
        imshow("ipm", outputImage);
        if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
        {
            cout << "esc key is pressed by user" << endl;
            break; 
        }
    }
    return 0;
}
