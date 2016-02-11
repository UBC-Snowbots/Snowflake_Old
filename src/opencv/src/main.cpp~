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
 */

#include "opencv2/core/core.hpp"
#include <stdio.h>
#include <ctime>
#include "IPM.h"
 // Use the image_transport classes instead.
 #include <ros/ros.h>
#include <image_transport/image_transport.h>
    
using namespace cv;
using namespace std;

int main( int _argc, char** _argv )
{

   ros::NodeHandle nh;
   //image_transport::ImageTransport it(nh);
   //image_transport::Subscriber sub = it.subscribe("in_image_base_topic", 1, imageCallback);
   //image_transport::Publisher pub = it.advertise("out_image_base_topic", 1);

/*
    VideoCapture cap(0); // open the video camera no. 0

    if (!cap.isOpened())  // if not success, exit program
    {
cout << "Cannot open the video cam" << endl;
        return -1;
    }

   int width = cap.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
   int height = cap.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video

    cout << "Frame size : " << width << " x " << height << endl;
/*
    namedWindow("MyVideo",CV_WINDOW_AUTOSIZE); //create a window called "MyVideo"

    while (1)
    {
        Mat frame;

        bool bSuccess = cap.read(frame); // read a new frame from video

if (!bSuccess) //if not success, break loop
{
cout << "Cannot read a frame from video stream" << endl;
break;
}

        imshow("MyVideo", frame); //show the frame in "MyVideo" window

        if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
{
cout << "esc key is pressed by user" << endl;
break; 
}
    }
    return 0;

}
*/
	// Images
	Mat inputImg, inputImgGray;
	Mat outputImg;
	Mat frame;
	/*
	if( _argc != 2 )
	{
		cout << "Usage: ipm.exe <videofile>" << endl;
		return 1;
	}

	// Video
	string videoFileName = _argv[1];	
	cv::VideoCapture video;
	if( !video.open(videoFileName) ) {
		cout <<"Video Error" << endl;
		return 1;
	}
*/
	// Show video information
	//int width = 0, height = 0, fps = 0, fourcc = 0;	
	//width = static_cast<int>(video.get(CV_CAP_PROP_FRAME_WIDTH));
	//height = static_cast<int>(video.get(CV_CAP_PROP_FRAME_HEIGHT));
	//fps = static_cast<int>(video.get(CV_CAP_PROP_FPS));
	//fourcc = static_cast<int>(video.get(CV_CAP_PROP_FOURCC));

	//cout << "Input video: (" << width << "x" << height << ") at " << fps << ", fourcc = " << fourcc << endl;
	
	// The 4-points at the input image	

	int width = 1920;
	int height = 1080;

	vector<Point2f> origPoints;
	origPoints.push_back( Point2f(0, height) );
	origPoints.push_back( Point2f(width, height) );
	//origPoints.push_back( Point2f(width/2+450, 580) );
	//origPoints.push_back( Point2f(width/2-250, 580) );
	origPoints.push_back( Point2f(width/2+450, 620) );
	origPoints.push_back( Point2f(width/2-250, 620) );

	// The 4-points correspondences in the destination image
	vector<Point2f> dstPoints;
	dstPoints.push_back( Point2f(0, height) );
	dstPoints.push_back( Point2f(width, height) );
	dstPoints.push_back( Point2f(width, 0) );
	dstPoints.push_back( Point2f(0, 0) );
		
	// IPM object
	IPM ipm( Size(width, height), Size(width, height), origPoints, dstPoints );
	
	// Main loop
	int frameNum = 0;
	while(1)
	{
		printf("FRAME #%6d ", frameNum);
		fflush(stdout);
		frameNum++;

		// Get current image
		//cap.read(frame);	

		//********
		frame = imread("/home/jannicke/Pictures/Image2.jpg", 1);
		
		//********

		if( frame.empty() )
			break;

		 // Color Conversion
		 if(frame.channels() == 3)		 
			 cvtColor(frame, inputImgGray, CV_BGR2GRAY);				 		 
		 else	 
			 frame.copyTo(inputImgGray);			 		 

		 // Process
		 clock_t begin = clock();
		 ipm.applyHomography( frame, outputImg );		 
		 clock_t end = clock();
		 double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
		 printf("%.2f (ms)\r", 1000*elapsed_secs);
		 ipm.drawPoints(origPoints, frame );

		 // View	
		 namedWindow("Input", CV_WINDOW_NORMAL);
		 namedWindow("Output", CV_WINDOW_NORMAL);
		 cvMoveWindow("Input", 0, 0);
		 cvMoveWindow("Output", 500, 0);	
		 imshow("Input", frame);
		 imshow("Output", outputImg);
		 imwrite( "/home/jannicke/Pictures/Image4.jpg", frame);
		 imwrite( "/home/jannicke/Pictures/Image3.jpg", outputImg);
                 //pub.publish(outputImg);
		 waitKey(1);
	}

	return 0;	
}		
