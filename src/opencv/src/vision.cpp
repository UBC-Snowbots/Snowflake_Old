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
 * Usage: Point to a video file, press C to start calibrating. Then click and drag a rectangle on the calibration window
 * 			You should see H S V values being set afterwards.
 */   

#include <opencv2/core/core.hpp>
#include <stdio.h>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include "filter.h"

using namespace cv;
using namespace std;


int main(int argc, char** argv){
    
    string inputWindow = "Input Image";
    string outputWindow = "Output Image";

    //Takes ownership of camera
    VideoCapture cap("/home/valerian/Documents/src/opencv/course.mov");
    if (!cap.isOpened()){
        cout << "Error opening camera" << endl;
        return -1;
    }

    Mat inputImage;
    Mat outputImage; 
   
    //Camera information for user
    
    int width = cap.get(CV_CAP_PROP_FRAME_WIDTH);
    int height = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
    cout << "Frame size: " << width << " x " << height << endl;
    
    namedWindow(inputWindow, CV_WINDOW_AUTOSIZE);
    namedWindow(outputWindow, CV_WINDOW_AUTOSIZE);
    cout << "Making it" << endl;
    cap.read(inputImage);
    cout << "Just read image" << endl;
    imshow(inputWindow, inputImage);
    cout << "After this function" << endl;
    snowbotsFilter filter(98, 130, 30, 165, 129, 255);

    while(1){

        //Reads image from camera
        
        bool isRead = cap.read(inputImage);
        if (!isRead){
            cout << "Failed to read image from camera" << endl;
            cap.set(CV_CAP_PROP_POS_AVI_RATIO, 0);
            continue; 
            //break;
        }
        imshow(inputWindow, inputImage);

        //filter.filterImage(inputImage, outputImage);

        imshow(outputWindow, outputImage);

        //Escape key to finish program
        int a = waitKey(30); 
        if (a == 27){
            cout << "Escaped by user" << endl;
            break;
        }
    }
    return 0;
}
