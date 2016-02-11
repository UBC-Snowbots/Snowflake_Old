#include <iostream>
#include <cv.h>
#include <highgui.h>
 

using namespace std;
using namespace cv;
char key;
int main()
{
	VideoCapture cap(0); // open the default camera
	if(!cap.isOpened())  // check if we succeeded
	    return -1;
    	cap.set(CV_CAP_PROP_FRAME_WIDTH, 400);
        cap.set(CV_CAP_PROP_FRAME_HEIGHT, 300);
        Mat edges;
	namedWindow("Red", WINDOW_AUTOSIZE);
	namedWindow("Green", WINDOW_AUTOSIZE);
	namedWindow("Blue", WINDOW_AUTOSIZE);
	namedWindow("original", WINDOW_AUTOSIZE);
	while (1)
	{
	    Mat frame;
            std::vector<cv::Mat> planes;	
	    cap >> frame; // get a new frame from camera
	    //cvtColor(frame, edges, CV_BGR2GRAY);
            cv::split(frame, planes);
	    GaussianBlur(frame, edges, Size(7,7), 1.5, 1.5);
	    //Canny(frame, edges, 0, 30, 3);
	    imshow("Blue", planes[0]);
	    imshow("Green", planes[1]);
	    imshow("Red", planes[2]);
	    imshow("original", frame);
	    if(waitKey(30) >= 0) break;
	}
	// the camera will be deinitialized automatically in VideoCapture destructor
	return 0;
}
