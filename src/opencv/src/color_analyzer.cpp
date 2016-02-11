/*
 * Analyzes an image and transforms it into a binary image
 * given some sort of colour specifiction
 * 
 */   

#include <opencv2/core/core.hpp>
#include <stdio.h>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>


using namespace cv;
using namespace std;

int main(int argc, char** argv){
    
    //Takes ownership of camera
    VideoCapture cap(0);
    if (!cap.isOpened()){
        cout << "Error opening camera" << endl;
        return -1;
    }

    Mat inputImage;
    
    //Camera information for user
    int width = cap.get(CV_CAP_PROP_FRAME_WIDTH);
    int height = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
    cout << "Frame size: " << width << " x " << height << endl;
    
    namedWindow("input", CV_WINDOW_AUTOSIZE);


    while(1){
    
        //Reads image from camera
        bool isRead = cap.read(inputImage);
        if (!isRead){
            cout << "Failed to read image from camera" << endl;
            break;
        }

        imshow("input", inputImage);

        //Escape key to finish program
        if (waitKey(30) == 27){
            cout << "Escaped by user" << endl;
            break;
        }
    }
    return 0;
}
