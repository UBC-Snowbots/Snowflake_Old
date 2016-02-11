/*
 * Analyzes an image and transforms it into a binary image
 * given some sort of colour specifiction
 * 
 * Current method: Simple binary b/w separator from a set threshold
 * Could attempt more sophisticated methods with more color channels
 * or hues to deal with shade issue 
 */   

#include <opencv2/core/core.hpp>
#include <stdio.h>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;

void convertToBinary(const Mat &input, Mat &output);

int main(int argc, char** argv){
    
    //Takes ownership of camera
    VideoCapture cap(0);
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
    
    namedWindow("input", CV_WINDOW_AUTOSIZE);
    namedWindow("output", CV_WINDOW_AUTOSIZE);

    while(1){
    
        //Reads image from camera
        bool isRead = cap.read(inputImage);
        if (!isRead){
            cout << "Failed to read image from camera" << endl;
            break;
        }
        
        convertToBinary(inputImage, outputImage);
        
        imshow("input", inputImage);
        imshow("output", outputImage);
        //Escape key to finish program
        if (waitKey(30) == 27){
            cout << "Escaped by user" << endl;
            break;
        }
    }
    return 0;
}

void convertToBinary(const Mat &input, Mat &output){
    cvtColor(input, output, CV_BGR2GRAY);
} 
