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

//Disgusting global variables for testing purposes
int iLowH = 0;
int iHighH = 179;

int iLowS = 0;
int iHighS = 255;

int iLowV = 0;
int iHighV = 255;


void convertToBinary(const Mat &input, Mat &output);

int main(int argc, char** argv){
    
    //Takes ownership of camera
    
    VideoCapture cap("/home/valerian/Documents/src/opencv/test2.mov");
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
    
    //Testing Window
    namedWindow("Control", CV_WINDOW_AUTOSIZE);    
    //Good Values: lowh = 98, highh = 130, lows = 30, highs = 165, lowv = 129, highv = 255
    cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
    cvCreateTrackbar("HighH", "Control", &iHighH, 179);

    cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
    cvCreateTrackbar("HighS", "Control", &iHighS, 255);

    cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
    cvCreateTrackbar("HighV", "Control", &iHighV, 255);


    while(1){
        
        //Reads image from camera
        
        bool isRead = cap.read(inputImage);
        if (!isRead){
            cout << "Failed to read image from camera" << endl;
            cap.set(CV_CAP_PROP_POS_AVI_RATIO, 0);
            continue; 
            //break;
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
    cvtColor(input, output, COLOR_BGR2HSV);
    
    inRange(output, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), output);
    //Morphological Opening (removes small objects from foreground)
    erode(output, output, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
    dilate(output, output, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));

    //Morphological Closing (fill small holes in the foreground)
    dilate(output, output, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
    erode(output, output, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );   
    
    
} 
